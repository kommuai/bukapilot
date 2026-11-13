#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "cereal/messaging/messaging.h"
#include "msgq/ipc.h"
#include <rk_mpi.h>
#include <rk_venc_cfg.h>
#include <rk_venc_cmd.h>
#include "system/loggerd/encoder/encoder.h"
#include "system/loggerd/loggerd.h"
#include "rga/rga.h"
#include "rga/im2d.h"
#define private public
#include "system/loggerd/encoder/mpp_encoder.h"
#undef private

namespace {
constexpr size_t W = 1920;
constexpr size_t H = 1200;
constexpr size_t FRAME_BYTES = W * H * 3 / 2;
constexpr size_t BUFFER_COUNT = 24;
constexpr int FPS = 20;
constexpr int FRAME_COUNT = 100;
constexpr size_t OUTPUT_MAX_BYTES = 8 * 1024 * 1024;

struct InputBuffer {
  VisionBuf vision{};
  void *mapping = MAP_FAILED;
};

bool dma_sync(int fd, uint64_t flags) {
  dma_buf_sync sync{};
  sync.flags = flags;
  return ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) == 0;
}

bool append_data(std::vector<uint8_t> &out, kj::ArrayPtr<const capnp::byte> data) {
  if (data.size() == 0) return true;
  if (out.size() + data.size() > OUTPUT_MAX_BYTES) return false;
  out.insert(out.end(), data.begin(), data.end());
  return true;
}

struct OutputStats {
  uint64_t road_packets = 0;
  uint64_t road_bytes = 0;
  uint64_t qroad_packets = 0;
  uint64_t qroad_bytes = 0;
  std::vector<uint32_t> road_frame_ids;
  std::vector<uint32_t> qroad_frame_ids;
  std::vector<uint8_t> road_stream;
};

class OutputCollector {
public:
  OutputCollector() {
    context = Context::create();
    road_socket = SubSocket::create(context, "roadEncodeData", "127.0.0.1", false, true,
                                    services.at("roadEncodeData").queue_size);
    qroad_socket = SubSocket::create(context, "qRoadEncodeData", "127.0.0.1", false, true,
                                     services.at("qRoadEncodeData").queue_size);
    if (context != nullptr && road_socket != nullptr && qroad_socket != nullptr) {
      poller = Poller::create();
      poller->registerSocket(road_socket);
      poller->registerSocket(qroad_socket);
    }
  }

  ~OutputCollector() {
    delete poller;
    delete road_socket;
    delete qroad_socket;
    delete context;
  }

  bool collect(OutputStats &stats, bool capture_road, int timeout_ms = 0) {
    if (poller == nullptr) return false;
    auto ready = poller->poll(timeout_ms);
    for (SubSocket *socket : ready) {
      while (Message *message = socket->receive(true)) {
        kj::ArrayPtr<capnp::word> words(
          reinterpret_cast<capnp::word *>(message->getData()), message->getSize() / sizeof(capnp::word));
        capnp::FlatArrayMessageReader reader(words);
        cereal::Event::Reader event = reader.getRoot<cereal::Event>();
        if (socket == road_socket) {
          auto encoded = event.getRoadEncodeData();
          auto idx = encoded.getIdx();
          auto data = encoded.getData();
          auto header = encoded.getHeader();
          stats.road_packets++;
          stats.road_bytes += data.size();
          stats.road_frame_ids.push_back(idx.getFrameId());
          if (capture_road && (!append_data(stats.road_stream, header) || !append_data(stats.road_stream, data))) {
            delete message;
            return false;
          }
        } else {
          auto encoded = event.getQRoadEncodeData();
          auto idx = encoded.getIdx();
          auto data = encoded.getData();
          stats.qroad_packets++;
          stats.qroad_bytes += data.size();
          stats.qroad_frame_ids.push_back(idx.getFrameId());
        }
        delete message;
      }
    }
    return true;
  }

  bool ready() const { return poller != nullptr; }

private:
  Context *context = nullptr;
  SubSocket *road_socket = nullptr;
  SubSocket *qroad_socket = nullptr;
  Poller *poller = nullptr;
};

bool allocate_inputs(std::vector<InputBuffer> &buffers) {
  buffers.reserve(BUFFER_COUNT);
  for (size_t i = 0; i < BUFFER_COUNT; ++i) {
    int heap = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (heap < 0) return false;
    dma_heap_allocation_data allocation{};
    allocation.len = FRAME_BYTES + sizeof(uint64_t);
    allocation.fd_flags = O_RDWR | O_CLOEXEC;
    if (ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &allocation) != 0) {
      close(heap);
      return false;
    }
    close(heap);

    InputBuffer input;
    input.vision.fd = static_cast<int>(allocation.fd);
    input.vision.len = FRAME_BYTES;
    input.vision.mmap_len = FRAME_BYTES + sizeof(uint64_t);
    input.mapping = mmap(nullptr, input.vision.mmap_len, PROT_READ | PROT_WRITE, MAP_SHARED, input.vision.fd, 0);
    if (input.mapping == MAP_FAILED) {
      close(input.vision.fd);
      return false;
    }
    if (!dma_sync(input.vision.fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW)) {
      munmap(input.mapping, input.vision.mmap_len);
      close(input.vision.fd);
      return false;
    }
    auto *pixels = static_cast<uint8_t *>(input.mapping);
    for (size_t y = 0; y < H; ++y) {
      for (size_t x = 0; x < W; ++x) {
        pixels[y * W + x] = static_cast<uint8_t>(32 + ((x * 7 + y * 11 + i * 17) % 192));
      }
    }
    std::memset(pixels + W * H, 128, W * H / 2);
    if (!dma_sync(input.vision.fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW)) {
      munmap(input.mapping, input.vision.mmap_len);
      close(input.vision.fd);
      return false;
    }
    input.vision.addr = input.mapping;
    input.vision.width = W;
    input.vision.height = H;
    input.vision.stride = W;
    input.vision.uv_offset = W * H;
    input.vision.frame_id_in_buf = false;
    buffers.push_back(input);
  }
  return true;
}

void free_inputs(std::vector<InputBuffer> &buffers) {
  for (auto &input : buffers) {
    if (input.mapping != MAP_FAILED) munmap(input.mapping, input.vision.mmap_len);
    if (input.vision.fd >= 0) close(input.vision.fd);
    input.mapping = MAP_FAILED;
    input.vision.fd = -1;
  }
}

uint64_t cpu_ns() {
  timespec ts{};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}
}

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IONBF, 0);
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s copy|zero road_output.hevc|-/dev/null\n", argv[0]);
    return 2;
  }
  const bool zero_copy = std::strcmp(argv[1], "zero") == 0;
  if (!zero_copy && std::strcmp(argv[1], "copy") != 0) return 2;
  setenv("ENCODER_ZERO_COPY", zero_copy ? "1" : "0", 1);
  std::printf("class_harness_mode=%s binary_class=MppEncoder dma_heap=system streams=road+qRoad fps=%d frames=%d buffers=%zu\n",
              zero_copy ? "zero" : "copy", FPS, FRAME_COUNT, BUFFER_COUNT);

  std::vector<InputBuffer> buffers;
  if (!allocate_inputs(buffers)) {
    std::fprintf(stderr, "allocate_inputs_failed errno=%d (%s)\n", errno, std::strerror(errno));
    free_inputs(buffers);
    return 3;
  }

  OutputStats output;
  std::vector<int64_t> road_submit_us;
  std::vector<int64_t> qroad_submit_us;
  road_submit_us.reserve(FRAME_COUNT);
  qroad_submit_us.reserve(FRAME_COUNT);
  int road_accepted = 0;
  int qroad_accepted = 0;
  int road_failures = 0;
  int qroad_failures = 0;
  bool route_valid = false;
  bool capture_road = std::strcmp(argv[2], "-") != 0;
  std::unique_ptr<OutputCollector> collector;
  uint64_t cpu_before = 0, cpu_after = 0;
  const auto wall_before = std::chrono::steady_clock::now();
  {
    MppEncoder road(main_road_encoder_info, W, H);
    MppEncoder qroad(qcam_encoder_info, W, H);
    collector = std::make_unique<OutputCollector>();
    if (!collector->ready()) {
      std::fprintf(stderr, "output_collector_connect_failed\n");
      free_inputs(buffers);
      return 11;
    }
    road.set_segment_num(0);
    qroad.set_segment_num(0);
    road.encoder_open();
    qroad.encoder_open();
    const auto sample_start = std::chrono::steady_clock::now();
    cpu_before = cpu_ns();
    auto next_frame = sample_start;
    for (int n = 0; n < FRAME_COUNT; ++n) {
      next_frame += std::chrono::milliseconds(1000 / FPS);
      VisionIpcBufExtra extra{};
      extra.frame_id = static_cast<uint32_t>(n + 1);
      extra.timestamp_sof = static_cast<uint64_t>(n) * 1000000000ULL / FPS;
      extra.timestamp_eof = extra.timestamp_sof + 1000000ULL;
      extra.valid = true;
      VisionBuf *input = &buffers[static_cast<size_t>(n) % buffers.size()].vision;

      auto t0 = std::chrono::steady_clock::now();
      const int road_ret = road.encode_frame(input, &extra);
      auto t1 = std::chrono::steady_clock::now();
      road_submit_us.push_back(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
      if (road_ret == 1) road_accepted++;
      else road_failures++;

      t0 = std::chrono::steady_clock::now();
      const int qroad_ret = qroad.encode_frame(input, &extra);
      t1 = std::chrono::steady_clock::now();
      qroad_submit_us.push_back(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
      if (qroad_ret == 1) qroad_accepted++;
      else qroad_failures++;

      if (!collector->collect(output, capture_road)) {
        std::fprintf(stderr, "temporary_output_ceiling_exceeded\n");
        road.encoder_close();
        qroad.encoder_close();
        free_inputs(buffers);
        return 4;
      }
      std::this_thread::sleep_until(next_frame);
    }
    std::printf("road_route_configured_zero_copy=%d road_imported_fds=%zu road_import_failures=%u road_fallback_logged=%d qroad_route_configured_zero_copy=%d qroad_imported_fds=%zu qroad_import_failures=%u qroad_fallback_logged=%d\n",
                road.use_zero_copy, road.imported_buffers.size(), road.zero_copy_import_failures,
                road.import_fallback_logged, qroad.use_zero_copy, qroad.imported_buffers.size(),
                qroad.zero_copy_import_failures, qroad.import_fallback_logged);
    route_valid = zero_copy
      ? road.use_zero_copy && road.imported_buffers.size() == buffers.size() &&
        road.zero_copy_import_failures == 0 && !road.import_fallback_logged &&
        !qroad.use_zero_copy && qroad.imported_buffers.empty()
      : !road.use_zero_copy && road.imported_buffers.empty() &&
        !qroad.use_zero_copy && qroad.imported_buffers.empty();
    road.encoder_close();
    qroad.encoder_close();
    const auto drain_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while ((output.road_packets < static_cast<uint64_t>(road_accepted) ||
            output.qroad_packets < static_cast<uint64_t>(qroad_accepted)) &&
           std::chrono::steady_clock::now() < drain_deadline) {
      if (!collector->collect(output, capture_road, 10)) break;
    }
    cpu_after = cpu_ns();
  }
  const auto wall_after = std::chrono::steady_clock::now();
  const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(wall_after - wall_before).count();

  if (capture_road) {
    std::ofstream out(argv[2], std::ios::binary | std::ios::trunc);
    if (!out || output.road_stream.size() > OUTPUT_MAX_BYTES) {
      std::fprintf(stderr, "road_stream_write_failed bytes=%zu\n", output.road_stream.size());
      free_inputs(buffers);
      return 5;
    }
    out.write(reinterpret_cast<const char *>(output.road_stream.data()), output.road_stream.size());
  }
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  const auto print_stats = [](const char *name, const std::vector<int64_t> &values) {
    std::vector<int64_t> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    const auto pct = [&](double p) { return sorted.empty() ? 0LL : sorted[static_cast<size_t>((sorted.size() - 1) * p)]; };
    std::printf("%s_submit_us_median=%lld p95=%lld p99=%lld n=%zu\n", name,
                pct(0.50), pct(0.95), pct(0.99), sorted.size());
  };
  print_stats("road", road_submit_us);
  print_stats("qroad", qroad_submit_us);
  std::printf("road_accepted=%d road_failures=%d road_packets=%llu road_payload_bytes=%llu road_stream_bytes=%zu\n",
              road_accepted, road_failures,
              static_cast<unsigned long long>(output.road_packets),
              static_cast<unsigned long long>(output.road_bytes), output.road_stream.size());
  std::printf("qroad_accepted=%d qroad_failures=%d qroad_packets=%llu qroad_payload_bytes=%llu\n",
              qroad_accepted, qroad_failures,
              static_cast<unsigned long long>(output.qroad_packets),
              static_cast<unsigned long long>(output.qroad_bytes));
  std::printf("cpu_sample_ns=%llu wall_total_ms=%lld peak_rss_kb=%ld\n",
              static_cast<unsigned long long>(cpu_after - cpu_before), wall_ms, usage.ru_maxrss);
  free_inputs(buffers);
  return route_valid && road_failures == 0 && qroad_failures == 0 &&
         output.road_packets == static_cast<uint64_t>(road_accepted) &&
         output.qroad_packets == static_cast<uint64_t>(qroad_accepted) ? 0 : 10;
}
