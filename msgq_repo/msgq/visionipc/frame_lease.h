#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include "msgq/visionipc/visionbuf.h"

class VisionIpcFrameLeaseClient {
public:
  VisionIpcFrameLeaseClient() = default;
  ~VisionIpcFrameLeaseClient();

  VisionIpcFrameLeaseClient(const VisionIpcFrameLeaseClient &) = delete;
  VisionIpcFrameLeaseClient &operator=(const VisionIpcFrameLeaseClient &) = delete;

  bool connect(VisionStreamType camera_stream);
  bool acknowledge(uint32_t frame_id, uint64_t generation);
  bool renew_session();
  bool stop_cleanly();
  uint64_t generation() const { return session_generation; }

private:
  int fd = -1;
  VisionStreamType stream = VISION_STREAM_MAX;
  uint64_t session_generation = 0;
};

class VisionIpcFrameLeaseServer {
public:
  enum class EventType { Started, Acknowledged, Renewed, Stopped, Disconnected };
  using Callback = std::function<void(EventType, VisionStreamType, uint32_t, uint64_t, uint64_t)>;

  VisionIpcFrameLeaseServer() = default;
  ~VisionIpcFrameLeaseServer();

  VisionIpcFrameLeaseServer(const VisionIpcFrameLeaseServer &) = delete;
  VisionIpcFrameLeaseServer &operator=(const VisionIpcFrameLeaseServer &) = delete;

  bool start(Callback callback);
  void stop();

private:
  struct Client {
    int fd = -1;
    VisionStreamType stream = VISION_STREAM_MAX;
    uint64_t generation = 0;
  };

  void run();
  void close_client(Client &client, EventType reason);

  std::atomic<bool> stopping = false;
  int listener_fd = -1;
  uint64_t next_generation = 1;
  std::string socket_path;
  Callback callback;
  std::thread worker;
};
