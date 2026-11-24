#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/memfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <array>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "msgq/visionipc/visionipc_server.h"

namespace {

constexpr size_t WIDTH = 1920;
constexpr size_t HEIGHT = 1200;
constexpr size_t STRIDE = (WIDTH + 15) & ~size_t{15};
constexpr size_t UV_OFFSET = STRIDE * HEIGHT;
constexpr size_t BUFFER_SIZE = UV_OFFSET * 3 / 2;
constexpr size_t FRAME_ID_SIZE = sizeof(uint64_t);
constexpr size_t ALLOCATION_SIZE = BUFFER_SIZE + FRAME_ID_SIZE;
constexpr size_t BUFFERS_PER_STREAM = 4;
constexpr const char *DMA_HEAP_PATH = "/dev/dma_heap/system";

constexpr std::array<VisionStreamType, 3> STREAMS = {
  VISION_STREAM_ROAD,
  VISION_STREAM_DRIVER,
  VISION_STREAM_WIDE_ROAD,
};

void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}

int retry_ioctl(int fd, unsigned long request, void *arg) {
  int ret;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret < 0 && errno == EINTR);
  return ret;
}

void sync_dma_buf(int fd, uint64_t flags) {
  dma_buf_sync sync = {.flags = flags};
  require(retry_ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) == 0,
          "DMA_BUF_IOCTL_SYNC failed: " + std::string(strerror(errno)));
}

int allocate_dma_buf(size_t len) {
  const int heap_fd = open(DMA_HEAP_PATH, O_RDONLY | O_CLOEXEC);
  require(heap_fd >= 0, "open(" + std::string(DMA_HEAP_PATH) + ") failed: " + strerror(errno));

  dma_heap_allocation_data allocation = {};
  allocation.len = len;
  allocation.fd_flags = O_RDWR | O_CLOEXEC;
  const int ret = retry_ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &allocation);
  const int saved_errno = errno;
  close(heap_fd);
  errno = saved_errno;
  require(ret == 0, "DMA_HEAP_IOCTL_ALLOC failed: " + std::string(strerror(errno)));
  return static_cast<int>(allocation.fd);
}

int allocate_shm_buf(size_t len) {
  const int fd = static_cast<int>(syscall(SYS_memfd_create, "loggerd-test-visionipc-shm", MFD_CLOEXEC));
  require(fd >= 0, "memfd_create failed: " + std::string(strerror(errno)));
  if (ftruncate(fd, static_cast<off_t>(len)) != 0) {
    const int saved_errno = errno;
    close(fd);
    errno = saved_errno;
    throw std::runtime_error("ftruncate memfd failed: " + std::string(strerror(errno)));
  }
  return fd;
}

class ExternalVisionBuf {
public:
  explicit ExternalVisionBuf(bool shm_backed) {
    fd = shm_backed ? allocate_shm_buf(ALLOCATION_SIZE) : allocate_dma_buf(ALLOCATION_SIZE);
    addr = mmap(nullptr, ALLOCATION_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
      const int saved_errno = errno;
      close(fd);
      fd = -1;
      errno = saved_errno;
      throw std::runtime_error("mmap dma-buf failed: " + std::string(strerror(errno)));
    }

    buf.fd = fd;
    buf.addr = addr;
    buf.len = BUFFER_SIZE;
    buf.mmap_len = ALLOCATION_SIZE;
    buf.frame_id_in_buf = true;
    buf.frame_id = reinterpret_cast<uint64_t *>(static_cast<uint8_t *>(addr) + BUFFER_SIZE);
    buf.init_yuv(WIDTH, HEIGHT, STRIDE, UV_OFFSET);
  }

  ExternalVisionBuf(const ExternalVisionBuf &) = delete;
  ExternalVisionBuf &operator=(const ExternalVisionBuf &) = delete;

  ~ExternalVisionBuf() {
    if (addr != MAP_FAILED && addr != nullptr) munmap(addr, ALLOCATION_SIZE);
    if (fd >= 0) close(fd);
  }

  VisionBuf buf;
  int fd = -1;
  void *addr = nullptr;
};

using StreamBuffers = std::vector<std::unique_ptr<ExternalVisionBuf>>;
using AllBuffers = std::array<StreamBuffers, STREAMS.size()>;

uint8_t y_value(VisionStreamType stream, uint32_t frame_id) {
  return static_cast<uint8_t>(16 + ((frame_id * 13 + static_cast<uint32_t>(stream) * 29) % 200));
}

uint8_t u_value(VisionStreamType stream, uint32_t frame_id) {
  return static_cast<uint8_t>(32 + ((frame_id * 7 + static_cast<uint32_t>(stream) * 17) % 190));
}

uint8_t v_value(VisionStreamType stream, uint32_t frame_id) {
  return static_cast<uint8_t>(32 + ((frame_id * 11 + static_cast<uint32_t>(stream) * 23) % 190));
}

void write_pattern(VisionBuf &buf, uint32_t frame_id, bool dma_buf) {
  const auto stream = buf.type;
  const uint8_t y = y_value(stream, frame_id);
  const uint8_t u = u_value(stream, frame_id);
  const uint8_t v = v_value(stream, frame_id);

  if (dma_buf) sync_dma_buf(buf.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
  std::memset(buf.y, y, UV_OFFSET);
  for (size_t i = 0; i < BUFFER_SIZE - UV_OFFSET; i += 2) {
    buf.uv[i] = u;
    buf.uv[i + 1] = v;
  }
  buf.set_frame_id(frame_id);
  if (dma_buf) sync_dma_buf(buf.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
}

struct BufferResource {
  int fd;
  uintptr_t address;
  dev_t device;
  ino_t inode;
};

bool buffer_fd_identity_is_open(dev_t device, ino_t inode) {
  DIR *dir = opendir("/proc/self/fd");
  require(dir != nullptr, "opendir(/proc/self/fd) failed");
  const int scan_fd = dirfd(dir);
  bool found = false;
  while (dirent *entry = readdir(dir)) {
    if (entry->d_name[0] == '.') continue;
    char *end = nullptr;
    const long fd = strtol(entry->d_name, &end, 10);
    if (end == entry->d_name || *end != '\0' || fd == scan_fd) continue;
    struct stat stat_buf = {};
    if (fstat(static_cast<int>(fd), &stat_buf) == 0 &&
        stat_buf.st_dev == device && stat_buf.st_ino == inode) {
      found = true;
      break;
    }
  }
  closedir(dir);
  return found;
}

bool mapping_exists(uintptr_t address) {
  std::ifstream maps("/proc/self/maps");
  std::string line;
  while (std::getline(maps, line)) {
    uintptr_t start = 0;
    uintptr_t end = 0;
    char dash = 0;
    std::istringstream row(line);
    row >> std::hex >> start >> dash >> end;
    if (row && dash == '-' && address >= start && address < end) return true;
  }
  return false;
}

AllBuffers allocate_stream_buffers(VisionIpcServer &server, bool shm_backed) {
  AllBuffers all;
  for (size_t stream_idx = 0; stream_idx < STREAMS.size(); ++stream_idx) {
    std::vector<VisionBuf *> external;
    external.reserve(BUFFERS_PER_STREAM);
    all[stream_idx].reserve(BUFFERS_PER_STREAM);
    for (size_t i = 0; i < BUFFERS_PER_STREAM; ++i) {
      auto buffer = std::make_unique<ExternalVisionBuf>(shm_backed);
      external.push_back(&buffer->buf);
      all[stream_idx].push_back(std::move(buffer));
    }
    server.register_external_buffers(STREAMS[stream_idx], external);
  }
  return all;
}

void send_frame(VisionIpcServer &server, VisionStreamType stream, uint32_t frame_id,
                uint64_t timestamp_sof, uint64_t timestamp_eof, size_t buffer_idx, bool shm_backed) {
  VisionBuf *buffer = server.get_buffer(stream, static_cast<int>(buffer_idx));
  write_pattern(*buffer, frame_id, !shm_backed);
  VisionIpcBufExtra extra = {
    .frame_id = frame_id,
    .timestamp_sof = timestamp_sof,
    .timestamp_eof = timestamp_eof,
    .valid = true,
  };
  server.send(buffer, &extra, false);
}

void run_server(bool shm_backed) {
  std::vector<BufferResource> resources;
  AllBuffers buffers;
  {
    VisionIpcServer server("camerad");
    buffers = allocate_stream_buffers(server, shm_backed);
    for (const auto &stream : buffers) {
      for (const auto &buffer : stream) {
        struct stat stat_buf = {};
        require(fstat(buffer->fd, &stat_buf) == 0, "fstat DMA-heap buffer failed");
        resources.push_back({buffer->fd, reinterpret_cast<uintptr_t>(buffer->addr),
                             stat_buf.st_dev, stat_buf.st_ino});
      }
    }
    server.start_listener();
    std::cout << "READY streams=road,driver,wideRoad width=" << WIDTH << " height=" << HEIGHT
              << " stride=" << STRIDE << " uv_offset=" << UV_OFFSET << " buffers="
              << BUFFERS_PER_STREAM << " backing=" << (shm_backed ? "shm" : "dma_heap") << std::endl;

    std::string line;
    while (std::getline(std::cin, line)) {
      if (line == "STOP") break;
      std::istringstream command(line);
      std::string verb;
      int stream_value = -1;
      uint32_t frame_id = 0;
      uint64_t timestamp_sof = 0;
      uint64_t timestamp_eof = 0;
      command >> verb >> stream_value >> frame_id >> timestamp_sof >> timestamp_eof;
      if (!command || verb != "FRAME" || stream_value < VISION_STREAM_ROAD || stream_value > VISION_STREAM_WIDE_ROAD) {
        std::cout << "ERROR malformed command" << std::endl;
        continue;
      }
      const auto stream = static_cast<VisionStreamType>(stream_value);
      const size_t buffer_idx = frame_id % BUFFERS_PER_STREAM;
      send_frame(server, stream, frame_id, timestamp_sof, timestamp_eof, buffer_idx, shm_backed);
      std::cout << "SENT " << stream_value << " " << frame_id << std::endl;
    }
  }

  buffers = {};
  for (const auto &resource : resources) {
    require(!buffer_fd_identity_is_open(resource.device, resource.inode),
            "DMA-heap buffer fd leaked after publisher shutdown");
    require(!mapping_exists(resource.address), "DMA-heap mapping leaked after publisher shutdown");
  }
  // VisionIPC's listener LOGD lazily starts swaglog's process-global ZeroMQ
  // context. Its eventfd/epoll descriptors remain open until process exit, so
  // cleanup is checked against the fixture's actual DMA-BUF identities/maps.
  std::cout << "CLEANUP_OK\n";
}

}  // namespace

int main(int argc, char **argv) {
  try {
    require(argc == 2, "usage: visionipc_dmabuf_publisher --serve|--serve-shm");
    const std::string mode = argv[1];
    if (mode == "--serve") {
      run_server(false);
    } else if (mode == "--serve-shm") {
      run_server(true);
    } else {
      throw std::runtime_error("unknown mode: " + mode);
    }
  } catch (const std::exception &e) {
    std::cerr << "VisionIPC DMA-BUF fixture failed: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
