#include <thread>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <string>
#include <vector>
#include <unistd.h>

#include "catch2/catch.hpp"

#include "msgq/visionipc/frame_lease.h"
#include "msgq/visionipc/visionipc_server.h"
#include "msgq/visionipc/visionipc_client.h"


static void zmq_sleep(int milliseconds=1000){
  if (messaging_use_zmq()){
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  }
}

TEST_CASE("Connecting"){
  VisionIpcServer server("camerad");
  server.create_buffers(VISION_STREAM_ROAD, 1, 100, 100);
  server.start_listener();

  VisionIpcClient client = VisionIpcClient("camerad", VISION_STREAM_ROAD, false);
  REQUIRE(client.connect());

  REQUIRE(client.connected);
}

TEST_CASE("getAvailableStreams"){
  VisionIpcServer server("camerad");
  server.create_buffers(VISION_STREAM_ROAD, 1, 100, 100);
  server.create_buffers(VISION_STREAM_WIDE_ROAD, 1, 100, 100);
  server.start_listener();
  auto available_streams = VisionIpcClient::getAvailableStreams("camerad");
  REQUIRE(available_streams.size() == 2);
  REQUIRE(available_streams.count(VISION_STREAM_ROAD) == 1);
  REQUIRE(available_streams.count(VISION_STREAM_WIDE_ROAD) == 1);
}

TEST_CASE("Check buffers"){
  size_t width = 100, height = 200, num_buffers = 5;
  VisionIpcServer server("camerad");
  server.create_buffers(VISION_STREAM_ROAD, num_buffers, width, height);
  server.start_listener();

  VisionIpcClient client = VisionIpcClient("camerad", VISION_STREAM_ROAD, false);
  REQUIRE(client.connect());

  REQUIRE(client.buffers[0].width == width);
  REQUIRE(client.buffers[0].height == height);
  REQUIRE(client.buffers[0].len);
  REQUIRE(client.num_buffers == num_buffers);
}

TEST_CASE("Send single buffer"){
  VisionIpcServer server("camerad");
  server.create_buffers(VISION_STREAM_ROAD, 1, 100, 100);
  server.start_listener();

  VisionIpcClient client = VisionIpcClient("camerad", VISION_STREAM_ROAD, false);
  REQUIRE(client.connect());
  zmq_sleep();

  VisionBuf * buf = server.get_buffer(VISION_STREAM_ROAD);
  REQUIRE(buf != nullptr);

  *((uint64_t*)buf->addr) = 1234;

  VisionIpcBufExtra extra = {0};
  extra.frame_id = 1337;
  buf->set_frame_id(extra.frame_id);

  server.send(buf, &extra);

  VisionIpcBufExtra extra_recv = {0};
  VisionBuf * recv_buf = client.recv(&extra_recv);
  REQUIRE(recv_buf != nullptr);
  REQUIRE(*(uint64_t*)recv_buf->addr == 1234);
  REQUIRE(extra_recv.frame_id == extra.frame_id);
  REQUIRE(recv_buf->get_frame_id() == extra.frame_id);
}


TEST_CASE("Test no conflate"){
  VisionIpcServer server("camerad");
  server.create_buffers(VISION_STREAM_ROAD, 1, 100, 100);
  server.start_listener();

  VisionIpcClient client = VisionIpcClient("camerad", VISION_STREAM_ROAD, false);
  REQUIRE(client.connect());
  zmq_sleep();

  VisionBuf * buf = server.get_buffer(VISION_STREAM_ROAD);
  REQUIRE(buf != nullptr);

  VisionIpcBufExtra extra = {0};
  extra.frame_id = 1;
  server.send(buf, &extra);
  extra.frame_id = 2;
  server.send(buf, &extra);

  VisionIpcBufExtra extra_recv = {0};
  VisionBuf * recv_buf = client.recv(&extra_recv);
  REQUIRE(recv_buf != nullptr);
  REQUIRE(extra_recv.frame_id == 1);

  recv_buf = client.recv(&extra_recv);
  REQUIRE(recv_buf != nullptr);
  REQUIRE(extra_recv.frame_id == 2);
}

TEST_CASE("Test conflate"){
  VisionIpcServer server("camerad");
  server.create_buffers(VISION_STREAM_ROAD, 1, 100, 100);
  server.start_listener();

  VisionIpcClient client = VisionIpcClient("camerad", VISION_STREAM_ROAD, true);
  REQUIRE(client.connect());
  zmq_sleep();

  VisionBuf * buf = server.get_buffer(VISION_STREAM_ROAD);
  REQUIRE(buf != nullptr);

  VisionIpcBufExtra extra = {0};
  extra.frame_id = 1;
  server.send(buf, &extra);
  extra.frame_id = 2;
  server.send(buf, &extra);

  VisionIpcBufExtra extra_recv = {0};
  VisionBuf * recv_buf = client.recv(&extra_recv);
  REQUIRE(recv_buf != nullptr);
  REQUIRE(extra_recv.frame_id == 2);

  recv_buf = client.recv(&extra_recv);
  REQUIRE(recv_buf == nullptr);
}

TEST_CASE("DMA-BUF frame lease session lifecycle") {
  struct PrefixRestore {
    bool had_old_prefix = false;
    std::string old_prefix;
    ~PrefixRestore() {
      if (had_old_prefix) {
        setenv("OPENPILOT_PREFIX", old_prefix.c_str(), 1);
      } else {
        unsetenv("OPENPILOT_PREFIX");
      }
    }
  } prefix_restore;
  const char *old_prefix_value = std::getenv("OPENPILOT_PREFIX");
  prefix_restore.had_old_prefix = old_prefix_value != nullptr;
  prefix_restore.old_prefix = prefix_restore.had_old_prefix ? old_prefix_value : "";
  const std::string test_prefix = "frame_lease_test_" + std::to_string(getpid());
  REQUIRE(setenv("OPENPILOT_PREFIX", test_prefix.c_str(), 1) == 0);

  using EventType = VisionIpcFrameLeaseServer::EventType;
  std::mutex events_mutex;
  std::condition_variable events_cv;
  std::vector<std::pair<EventType, uint64_t>> events;
  bool streams_match = true;
  VisionIpcFrameLeaseServer server;
  REQUIRE(server.start([&](EventType type, VisionStreamType stream, uint32_t, uint64_t old_generation,
                           uint64_t new_generation) {
    std::lock_guard lock(events_mutex);
    streams_match = streams_match && stream == VISION_STREAM_ROAD;
    events.emplace_back(type, type == EventType::Started ? new_generation : old_generation);
    events_cv.notify_all();
  }));

  VisionIpcFrameLeaseClient client;
  const auto connect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!client.connect(VISION_STREAM_ROAD) && std::chrono::steady_clock::now() < connect_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  REQUIRE(client.generation() != 0);
  const uint64_t first_generation = client.generation();
  REQUIRE(client.acknowledge(41, first_generation));
  REQUIRE(client.renew_session());
  const uint64_t second_generation = client.generation();
  REQUIRE(second_generation != 0);
  REQUIRE(second_generation != first_generation);
  REQUIRE(client.acknowledge(42, second_generation));
  REQUIRE(client.stop_cleanly());

  {
    std::unique_lock lock(events_mutex);
    REQUIRE(events_cv.wait_for(lock, std::chrono::seconds(2), [&] { return events.size() == 5; }));
  }
  CHECK(streams_match);
  CHECK(events[0].first == EventType::Started);
  CHECK(events[0].second == first_generation);
  CHECK(events[1].first == EventType::Acknowledged);
  CHECK(events[1].second == first_generation);
  CHECK(events[2].first == EventType::Renewed);
  CHECK(events[2].second == first_generation);
  CHECK(events[3].first == EventType::Acknowledged);
  CHECK(events[3].second == second_generation);
  CHECK(events[4].first == EventType::Stopped);
  CHECK(events[4].second == second_generation);

  uint64_t interrupted_generation = 0;
  {
    VisionIpcFrameLeaseClient interrupted_client;
    const auto reconnect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!interrupted_client.connect(VISION_STREAM_ROAD) &&
           std::chrono::steady_clock::now() < reconnect_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    REQUIRE(interrupted_client.generation() != 0);
    interrupted_generation = interrupted_client.generation();
  }  // Closing without STOP simulates an encoderd crash and must fail closed.

  {
    std::unique_lock lock(events_mutex);
    REQUIRE(events_cv.wait_for(lock, std::chrono::seconds(2), [&] { return events.size() == 7; }));
  }
  CHECK(events[5].first == EventType::Started);
  CHECK(events[5].second == interrupted_generation);
  CHECK(events[6].first == EventType::Disconnected);
  CHECK(events[6].second == interrupted_generation);

  server.stop();
}

TEST_CASE("VisionIPC listeners bound incomplete handshakes") {
  const std::string server_name = "visionipc_stalled_test_" + std::to_string(getpid());
  VisionIpcServer server(server_name);
  server.create_buffers(VISION_STREAM_ROAD, 1, 16, 16);
  server.start_listener();

  int client_fd = -1;
  const auto connect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  const std::string socket_path = get_ipc_path(server_name);
  while (client_fd < 0 && std::chrono::steady_clock::now() < connect_deadline) {
    client_fd = ipc_connect(socket_path.c_str());
    if (client_fd < 0) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  REQUIRE(client_fd >= 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const auto shutdown_start = std::chrono::steady_clock::now();
  server.stop_listener();
  const auto shutdown_time = std::chrono::steady_clock::now() - shutdown_start;
  close(client_fd);
  CHECK(shutdown_time < std::chrono::seconds(2));
}

TEST_CASE("Frame lease listener bounds incomplete handshake") {
  struct PrefixRestore {
    bool had_old_prefix = false;
    std::string old_prefix;
    ~PrefixRestore() {
      if (had_old_prefix) {
        setenv("OPENPILOT_PREFIX", old_prefix.c_str(), 1);
      } else {
        unsetenv("OPENPILOT_PREFIX");
      }
    }
  } prefix_restore;
  const char *old_prefix_value = std::getenv("OPENPILOT_PREFIX");
  prefix_restore.had_old_prefix = old_prefix_value != nullptr;
  prefix_restore.old_prefix = prefix_restore.had_old_prefix ? old_prefix_value : "";
  const std::string test_prefix = "stalled_frame_lease_test_" + std::to_string(getpid());
  REQUIRE(setenv("OPENPILOT_PREFIX", test_prefix.c_str(), 1) == 0);

  VisionIpcFrameLeaseServer server;
  REQUIRE(server.start([](VisionIpcFrameLeaseServer::EventType, VisionStreamType, uint32_t, uint64_t, uint64_t) {}));
  const std::string socket_path = get_ipc_path("camerad_frame_lease");
  int client_fd = -1;
  const auto connect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (client_fd < 0 && std::chrono::steady_clock::now() < connect_deadline) {
    client_fd = ipc_connect(socket_path.c_str());
    if (client_fd < 0) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  REQUIRE(client_fd >= 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const auto shutdown_start = std::chrono::steady_clock::now();
  server.stop();
  const auto shutdown_time = std::chrono::steady_clock::now() - shutdown_start;
  close(client_fd);
  CHECK(shutdown_time < std::chrono::seconds(2));
}
