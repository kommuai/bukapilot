#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <rk_mpi.h>
#ifndef MODULE_TAG
#define MODULE_TAG "ka2-dmabuf-probe"
#endif
#include <mpp_buffer.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {
constexpr size_t kWidth = 1920;
constexpr size_t kHeight = 1200;
constexpr size_t kNv12Bytes = kWidth * kHeight * 3 / 2;
constexpr size_t kMaximumAllocation = 4 * 1024 * 1024;

bool sync_dma_buf(int fd, uint64_t flags, const char *label) {
  dma_buf_sync sync{};
  sync.flags = flags;
  const int rc = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
  std::printf("dma_buf_sync_%s_rc=%d errno=%d (%s)\n", label, rc, errno, std::strerror(errno));
  return rc == 0;
}
}

int main() {
  std::printf("probe=single_system_heap_dmabuf_mpp_import\n");
  std::printf("frame_width=%zu frame_height=%zu nv12_bytes=%zu max_allocation=%zu\n",
              kWidth, kHeight, kNv12Bytes, kMaximumAllocation);
  if (kNv12Bytes > kMaximumAllocation) {
    std::fprintf(stderr, "planned allocation exceeds hard ceiling\n");
    return 2;
  }

  int heap_fd = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
  if (heap_fd < 0) {
    std::fprintf(stderr, "open_system_heap_failed errno=%d (%s)\n", errno, std::strerror(errno));
    return 3;
  }

  dma_heap_allocation_data allocation{};
  allocation.len = kNv12Bytes;
  allocation.fd_flags = O_RDWR | O_CLOEXEC;
  if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &allocation) != 0) {
    std::fprintf(stderr, "dma_heap_alloc_failed errno=%d (%s)\n", errno, std::strerror(errno));
    close(heap_fd);
    return 4;
  }
  close(heap_fd);
  const int dma_fd = static_cast<int>(allocation.fd);
  std::printf("heap=/dev/dma_heap/system dma_fd=%d allocated_bytes=%llu\n",
              dma_fd, static_cast<unsigned long long>(allocation.len));

  char link_path[64]{};
  std::snprintf(link_path, sizeof(link_path), "/proc/self/fd/%d", dma_fd);
  char link_target[256]{};
  const ssize_t link_len = readlink(link_path, link_target, sizeof(link_target) - 1);
  if (link_len < 0) {
    std::fprintf(stderr, "dma_fd_readlink_failed errno=%d (%s)\n", errno, std::strerror(errno));
    close(dma_fd);
    return 5;
  }
  link_target[link_len] = '\0';
  std::printf("dma_fd_identity=%s\n", link_target);

  struct stat st{};
  if (fstat(dma_fd, &st) != 0) {
    std::fprintf(stderr, "dma_fd_fstat_failed errno=%d (%s)\n", errno, std::strerror(errno));
    close(dma_fd);
    return 6;
  }
  std::printf("dma_fd_mode=%#o dma_fd_device=%llu dma_fd_inode=%llu\n",
              static_cast<unsigned>(st.st_mode), static_cast<unsigned long long>(st.st_dev),
              static_cast<unsigned long long>(st.st_ino));

  void *mapping = mmap(nullptr, kNv12Bytes, PROT_READ | PROT_WRITE, MAP_SHARED, dma_fd, 0);
  if (mapping == MAP_FAILED) {
    std::fprintf(stderr, "dma_fd_mmap_failed errno=%d (%s)\n", errno, std::strerror(errno));
    close(dma_fd);
    return 7;
  }
  if (!sync_dma_buf(dma_fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW, "start")) {
    munmap(mapping, kNv12Bytes);
    close(dma_fd);
    return 8;
  }
  auto *bytes = static_cast<unsigned char *>(mapping);
  for (size_t i = 0; i < kNv12Bytes; ++i) {
    bytes[i] = static_cast<unsigned char>((i * 37u + 19u) & 0xffu);
  }
  if (!sync_dma_buf(dma_fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW, "end")) {
    munmap(mapping, kNv12Bytes);
    close(dma_fd);
    return 9;
  }

  MppCtx ctx = nullptr;
  MppApi *api = nullptr;
  const MPP_RET create_ret = mpp_create(&ctx, &api);
  std::printf("mpp_create=%d ctx=%s api=%s\n", create_ret, ctx ? "present" : "null",
              api ? "present" : "null");
  if (create_ret != MPP_OK || ctx == nullptr || api == nullptr) {
    if (ctx != nullptr) {
      std::printf("mpp_destroy_after_create_failure=%d\n", mpp_destroy(ctx));
    }
    munmap(mapping, kNv12Bytes);
    close(dma_fd);
    return 10;
  }

  const MPP_RET init_ret = mpp_init(ctx, MPP_CTX_ENC, MPP_VIDEO_CodingHEVC);
  std::printf("mpp_init_encoder_hevc=%d\n", init_ret);
  if (init_ret != MPP_OK) {
    std::printf("mpp_destroy_after_init_failure=%d\n", mpp_destroy(ctx));
    munmap(mapping, kNv12Bytes);
    close(dma_fd);
    return 11;
  }

  MppBuffer imported = nullptr;
  MppBufferInfo info{};
  info.type = MPP_BUFFER_TYPE_EXT_DMA;
  info.fd = dma_fd;
  info.size = kNv12Bytes;
  const MPP_RET fd_import_ret = mpp_buffer_import(&imported, &info);
  std::printf("fd_size_import_ret=%d imported=%s\n", fd_import_ret,
              imported ? "present" : "null");

  MPP_RET ptr_import_ret = MPP_NOK;
  if (fd_import_ret != MPP_OK) {
    if (imported != nullptr) {
      std::printf("put_partial_fd_import=%d\n", mpp_buffer_put(imported));
      imported = nullptr;
    }
    info.ptr = mapping;
    ptr_import_ret = mpp_buffer_import(&imported, &info);
    std::printf("ptr_retry_import_ret=%d imported=%s\n", ptr_import_ret,
                imported ? "present" : "null");
  } else {
    std::printf("ptr_retry_import=not_needed\n");
  }

  const bool import_ok = (fd_import_ret == MPP_OK || ptr_import_ret == MPP_OK) && imported != nullptr;
  if (imported != nullptr) {
    std::printf("mpp_buffer_put=%d\n", mpp_buffer_put(imported));
  }
  const MPP_RET destroy_ret = mpp_destroy(ctx);
  std::printf("mpp_destroy=%d\n", destroy_ret);
  std::printf("frames_submitted=0 packets_received=0 output_bytes=0\n");
  const bool unmap_ok = munmap(mapping, kNv12Bytes) == 0;
  const int close_ret = close(dma_fd);
  std::printf("munmap_ok=%s close_dma_fd=%d\n", unmap_ok ? "true" : "false", close_ret);

  if (!import_ok) return 12;
  if (destroy_ret != MPP_OK || !unmap_ok || close_ret != 0) return 13;
  return 0;
}
