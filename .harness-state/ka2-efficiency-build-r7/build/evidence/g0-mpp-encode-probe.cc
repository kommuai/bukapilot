#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <rk_mpi.h>
#ifndef MODULE_TAG
#define MODULE_TAG "ka2-mpp-encode-probe"
#endif
#include <mpp_buffer.h>
#include <mpp_frame.h>
#include <mpp_packet.h>
#include <rk_venc_cfg.h>
#include <rk_venc_cmd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace {
constexpr RK_U32 kWidth = 1920;
constexpr RK_U32 kHeight = 1200;
constexpr RK_U32 kStrideWidth = 1920;
constexpr RK_U32 kStrideHeight = 1200;
constexpr size_t kYSize = static_cast<size_t>(kStrideWidth) * kStrideHeight;
constexpr size_t kFrameBytes = kYSize * 3 / 2;
constexpr size_t kMaxStreamBytes = 8 * 1024 * 1024;

bool dma_sync(int fd, uint64_t flags, const char *name) {
  dma_buf_sync sync{};
  sync.flags = flags;
  errno = 0;
  const int rc = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
  std::printf("dma_buf_sync_%s_rc=%d errno=%d\n", name, rc, rc == 0 ? 0 : errno);
  return rc == 0;
}

bool append_packet(std::vector<uint8_t> &out, MppPacket packet) {
  if (packet == nullptr) return false;
  auto *pos = static_cast<const uint8_t *>(mpp_packet_get_pos(packet));
  const size_t len = mpp_packet_get_length(packet);
  if (pos != nullptr && len > 0) {
    if (out.size() + len > kMaxStreamBytes) return false;
    out.insert(out.end(), pos, pos + len);
    return true;
  }

  const auto *segments = mpp_packet_get_segment_info(packet);
  auto *base = static_cast<const uint8_t *>(mpp_packet_get_data(packet));
  if (segments == nullptr || base == nullptr) return false;
  for (auto *segment = segments; segment != nullptr; segment = segment->next) {
    if (segment->len == 0) continue;
    if (out.size() + segment->len > kMaxStreamBytes) return false;
    out.insert(out.end(), base + segment->offset, base + segment->offset + segment->len);
  }
  return true;
}

bool write_all(int fd, const uint8_t *data, size_t len) {
  size_t written = 0;
  while (written < len) {
    const ssize_t rc = write(fd, data + written, len - written);
    if (rc < 0 && errno == EINTR) continue;
    if (rc <= 0) return false;
    written += static_cast<size_t>(rc);
  }
  return true;
}
}

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IONBF, 0);
  std::printf("probe=single_synthetic_dmabuf_hevc_encode\n");
  std::printf("geometry=%ux%u stride=%ux%u uv_offset=%zu nv12_bytes=%zu\n",
              kWidth, kHeight, kStrideWidth, kStrideHeight, kYSize, kFrameBytes);
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s output_stream_path\n", argv[0]);
    return 2;
  }

  int result = 1;
  int heap_fd = -1;
  int dma_fd = -1;
  int stream_fd = -1;
  void *mapping = MAP_FAILED;
  MppCtx ctx = nullptr;
  MppApi *api = nullptr;
  MppEncCfg cfg = nullptr;
  MppFrame frame = nullptr;
  MppFrame eos_frame = nullptr;
  MppPacket header_packet = nullptr;
  MppPacket packet = nullptr;
  MppBuffer imported = nullptr;
  std::vector<uint8_t> stream;
  uint8_t header_data[1024]{};
  MPP_RET cleanup_ret = MPP_OK;

  heap_fd = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
  if (heap_fd < 0) {
    std::printf("open_system_heap_failed errno=%d\n", errno);
    goto cleanup;
  }
  {
    dma_heap_allocation_data allocation{};
    allocation.len = kFrameBytes;
    allocation.fd_flags = O_RDWR | O_CLOEXEC;
    if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &allocation) != 0) {
      std::printf("dma_heap_alloc_failed errno=%d\n", errno);
      goto cleanup;
    }
    dma_fd = static_cast<int>(allocation.fd);
    std::printf("heap=/dev/dma_heap/system fd=%d allocated_bytes=%llu\n", dma_fd,
                static_cast<unsigned long long>(allocation.len));
  }
  close(heap_fd);
  heap_fd = -1;

  mapping = mmap(nullptr, kFrameBytes, PROT_READ | PROT_WRITE, MAP_SHARED, dma_fd, 0);
  if (mapping == MAP_FAILED) {
    std::printf("dma_buf_mmap_failed errno=%d\n", errno);
    goto cleanup;
  }
  if (!dma_sync(dma_fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW, "start")) goto cleanup;
  {
    auto *bytes = static_cast<uint8_t *>(mapping);
    for (RK_U32 y = 0; y < kHeight; ++y) {
      for (RK_U32 x = 0; x < kWidth; ++x) {
        bytes[static_cast<size_t>(y) * kStrideWidth + x] =
          static_cast<uint8_t>(32 + ((x * 173u / kWidth + y * 97u / kHeight) % 192u));
      }
    }
    for (size_t i = kYSize; i < kFrameBytes; i += 2) {
      bytes[i] = 128;
      bytes[i + 1] = 128;
    }
  }
  if (!dma_sync(dma_fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW, "end")) goto cleanup;

  {
    char link_path[64]{};
    char link_target[256]{};
    std::snprintf(link_path, sizeof(link_path), "/proc/self/fd/%d", dma_fd);
    const ssize_t len = readlink(link_path, link_target, sizeof(link_target) - 1);
    if (len < 0) {
      std::printf("dma_buf_readlink_failed errno=%d\n", errno);
      goto cleanup;
    }
    link_target[len] = '\0';
    std::printf("dma_buf_identity=%s\n", link_target);
  }

  {
    const MPP_RET ret = mpp_create(&ctx, &api);
    std::printf("mpp_create=%d\n", ret);
    if (ret != MPP_OK || ctx == nullptr || api == nullptr) goto cleanup;
  }
  {
    const MPP_RET ret = mpp_init(ctx, MPP_CTX_ENC, MPP_VIDEO_CodingHEVC);
    std::printf("mpp_init_hevc=%d\n", ret);
    if (ret != MPP_OK) goto cleanup;
  }
  {
    MppBufferInfo info{};
    info.type = MPP_BUFFER_TYPE_EXT_DMA;
    info.fd = dma_fd;
    info.size = kFrameBytes;
    MPP_RET ret = mpp_buffer_import(&imported, &info);
    std::printf("fd_size_import_ret=%d imported=%s\n", ret, imported ? "present" : "null");
    if (ret != MPP_OK) {
      if (imported != nullptr) {
        mpp_buffer_put(imported);
        imported = nullptr;
      }
      info.ptr = mapping;
      ret = mpp_buffer_import(&imported, &info);
      std::printf("ptr_retry_import_ret=%d imported=%s\n", ret, imported ? "present" : "null");
    } else {
      std::printf("ptr_retry_import=not_needed\n");
    }
    if (ret != MPP_OK || imported == nullptr) goto cleanup;
  }

  {
    MPP_RET ret = mpp_enc_cfg_init(&cfg);
    std::printf("mpp_enc_cfg_init=%d\n", ret);
    if (ret != MPP_OK) goto cleanup;
    ret = api->control(ctx, MPP_ENC_GET_CFG, cfg);
    std::printf("MPP_ENC_GET_CFG=%d\n", ret);
    if (ret != MPP_OK) goto cleanup;

    const struct { const char *name; RK_S32 value; } signed_cfg[] = {
      {"split:mode", MPP_ENC_SPLIT_NONE},
      {"rc:qp_init", 26}, {"rc:qp_max", 51}, {"rc:qp_min", 10},
      {"rc:qp_max_i", 46}, {"rc:qp_min_i", 10}, {"rc:qp_ip", 2},
      {"prep:width", static_cast<RK_S32>(kWidth)},
      {"prep:height", static_cast<RK_S32>(kHeight)},
      {"prep:hor_stride", static_cast<RK_S32>(kStrideWidth)},
      {"prep:ver_stride", static_cast<RK_S32>(kStrideHeight)},
      {"prep:format", MPP_FMT_YUV420SP},
      {"rc:bps_target", 10000000}, {"rc:bps_max", 10100000}, {"rc:bps_min", 9900000},
    };
    for (const auto &entry : signed_cfg) {
      ret = mpp_enc_cfg_set_s32(cfg, entry.name, entry.value);
      if (ret != MPP_OK) {
        std::printf("mpp_enc_cfg_set_s32_failed key=%s ret=%d\n", entry.name, ret);
        goto cleanup;
      }
    }
    const struct { const char *name; RK_U32 value; } unsigned_cfg[] = {
      {"codec:type", MPP_VIDEO_CodingHEVC},
      {"rc:fps_in_num", 20}, {"rc:fps_out_num", 20},
      {"rc:mode", MPP_ENC_RC_MODE_CBR}, {"rc:gop", 90},
    };
    for (const auto &entry : unsigned_cfg) {
      ret = mpp_enc_cfg_set_u32(cfg, entry.name, entry.value);
      if (ret != MPP_OK) {
        std::printf("mpp_enc_cfg_set_u32_failed key=%s ret=%d\n", entry.name, ret);
        goto cleanup;
      }
    }
    ret = api->control(ctx, MPP_ENC_SET_CFG, cfg);
    std::printf("MPP_ENC_SET_CFG=%d\n", ret);
    if (ret != MPP_OK) goto cleanup;
  }
  {
    MppEncHeaderMode header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
    const MPP_RET mode_ret = api->control(ctx, MPP_ENC_SET_HEADER_MODE, &header_mode);
    std::printf("MPP_ENC_SET_HEADER_MODE=%d\n", mode_ret);
    if (mpp_packet_init(&header_packet, header_data, sizeof(header_data)) != MPP_OK) goto cleanup;
    mpp_packet_set_length(header_packet, 0);
    const MPP_RET ret = api->control(ctx, MPP_ENC_GET_HDR_SYNC, header_packet);
    const size_t header_len = mpp_packet_get_length(header_packet);
    std::printf("MPP_ENC_GET_HDR_SYNC=%d header_bytes=%zu\n", ret, header_len);
    if (ret == MPP_OK && header_len > 0 && header_len <= sizeof(header_data)) {
      stream.insert(stream.end(), header_data, header_data + header_len);
    }
    if (header_len > sizeof(header_data)) goto cleanup;
  }
  {
    RK_S64 output_timeout = MPP_TIMEOUT_NON_BLOCK;
    const MPP_RET ret = api->control(ctx, MPP_SET_OUTPUT_TIMEOUT, &output_timeout);
    std::printf("MPP_SET_OUTPUT_TIMEOUT=%d\n", ret);
  }

  if (mpp_frame_init(&frame) != MPP_OK) goto cleanup;
  mpp_frame_set_width(frame, kWidth);
  mpp_frame_set_height(frame, kHeight);
  mpp_frame_set_hor_stride(frame, kStrideWidth);
  mpp_frame_set_ver_stride(frame, kStrideHeight);
  mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
  mpp_frame_set_pts(frame, 1);
  mpp_frame_set_buffer(frame, imported);
  {
    const MPP_RET ret = api->encode_put_frame(ctx, frame);
    std::printf("encode_put_frame=%d\n", ret);
    if (ret != MPP_OK) goto cleanup;
  }
  if (mpp_frame_init(&eos_frame) != MPP_OK) goto cleanup;
  mpp_frame_set_width(eos_frame, kWidth);
  mpp_frame_set_height(eos_frame, kHeight);
  mpp_frame_set_hor_stride(eos_frame, kStrideWidth);
  mpp_frame_set_ver_stride(eos_frame, kStrideHeight);
  mpp_frame_set_fmt(eos_frame, MPP_FMT_YUV420SP);
  mpp_frame_set_eos(eos_frame, 1);
  {
    const MPP_RET ret = api->encode_put_frame(ctx, eos_frame);
    std::printf("encode_put_eos=%d\n", ret);
    if (ret != MPP_OK) goto cleanup;
  }

  {
    bool saw_eos = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
    while (!saw_eos && std::chrono::steady_clock::now() < deadline) {
      const MPP_RET poll_ret = api->poll(ctx, MPP_PORT_OUTPUT, static_cast<MppPollType>(50));
      if (poll_ret != MPP_OK) continue;
      packet = nullptr;
      const MPP_RET get_ret = api->encode_get_packet(ctx, &packet);
      std::printf("encode_get_packet=%d packet=%s", get_ret, packet ? "present" : "null");
      if (packet != nullptr) {
        const size_t packet_bytes = mpp_packet_get_length(packet);
        saw_eos = mpp_packet_get_eos(packet) != 0;
        std::printf(" bytes=%zu eos=%d\n", packet_bytes, saw_eos ? 1 : 0);
        if (packet_bytes > 0 && !append_packet(stream, packet)) {
          mpp_packet_deinit(&packet);
          packet = nullptr;
          goto cleanup;
        }
        mpp_packet_deinit(&packet);
        packet = nullptr;
      } else {
        std::printf("\n");
      }
    }
    std::printf("saw_eos=%s packet_and_header_bytes=%zu\n", saw_eos ? "true" : "false", stream.size());
    if (!saw_eos || stream.empty()) goto cleanup;
  }

  stream_fd = open(argv[1], O_WRONLY | O_TRUNC | O_CLOEXEC);
  if (stream_fd < 0 || !write_all(stream_fd, stream.data(), stream.size()) || fsync(stream_fd) != 0) {
    std::printf("stream_write_failed errno=%d\n", errno);
    goto cleanup;
  }
  close(stream_fd);
  stream_fd = -1;
  std::printf("stream_write_ok bytes=%zu\n", stream.size());
  result = 0;

cleanup:
  if (packet != nullptr) mpp_packet_deinit(&packet);
  if (header_packet != nullptr) mpp_packet_deinit(&header_packet);
  if (eos_frame != nullptr) mpp_frame_deinit(&eos_frame);
  if (frame != nullptr) mpp_frame_deinit(&frame);
  if (imported != nullptr) {
    cleanup_ret = mpp_buffer_put(imported);
    std::printf("mpp_buffer_put=%d\n", cleanup_ret);
    if (cleanup_ret != MPP_OK) result = 3;
  }
  if (cfg != nullptr) {
    cleanup_ret = mpp_enc_cfg_deinit(cfg);
    std::printf("mpp_enc_cfg_deinit=%d\n", cleanup_ret);
    if (cleanup_ret != MPP_OK) result = 3;
  }
  if (ctx != nullptr) {
    cleanup_ret = mpp_destroy(ctx);
    std::printf("mpp_destroy=%d\n", cleanup_ret);
    if (cleanup_ret != MPP_OK) result = 3;
  }
  if (stream_fd >= 0 && close(stream_fd) != 0) result = 3;
  if (mapping != MAP_FAILED && munmap(mapping, kFrameBytes) != 0) result = 3;
  if (dma_fd >= 0 && close(dma_fd) != 0) result = 3;
  if (heap_fd >= 0 && close(heap_fd) != 0) result = 3;
  std::printf("frames_submitted=1 eos_markers=1 output_stream_bytes=%zu result=%d\n", stream.size(), result);
  return result;
}
