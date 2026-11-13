#include "msgq/visionipc/frame_lease.h"

#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "common/swaglog.h"
#include "msgq/visionipc/visionipc_server.h"

namespace {

constexpr uint32_t kFrameLeaseMagic = 0x4b413246;  // "KA2F"
constexpr uint16_t kFrameLeaseVersion = 1;

enum class MessageType : uint16_t { Hello = 1, Session = 2, Ack = 3, Renew = 4, Stop = 5, Reject = 6 };

struct LeaseMessage {
  uint32_t magic = kFrameLeaseMagic;
  uint16_t version = kFrameLeaseVersion;
  MessageType type = MessageType::Hello;
  int32_t stream = VISION_STREAM_MAX;
  uint32_t frame_id = 0;
  uint64_t generation = 0;
};

bool send_message(int fd, const LeaseMessage &msg) {
#ifdef MSG_NOSIGNAL
  constexpr int flags = MSG_NOSIGNAL;
#else
  constexpr int flags = 0;
#endif
  const ssize_t sent = send(fd, &msg, sizeof(msg), flags);
  return sent == static_cast<ssize_t>(sizeof(msg));
}

bool receive_message(int fd, LeaseMessage *msg) {
  const ssize_t received = recv(fd, msg, sizeof(*msg), 0);
  return received == static_cast<ssize_t>(sizeof(*msg)) &&
         msg->magic == kFrameLeaseMagic && msg->version == kFrameLeaseVersion;
}

int camera_slot(VisionStreamType stream) {
  switch (stream) {
    case VISION_STREAM_ROAD: return 0;
    case VISION_STREAM_DRIVER: return 1;
    case VISION_STREAM_WIDE_ROAD: return 2;
    default: return -1;
  }
}

int make_listener(const std::string &path) {
#ifdef __APPLE__
  const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
#else
  const int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
#endif
  if (fd < 0) return -1;

  sockaddr_un addr = {};
  addr.sun_family = AF_UNIX;
  if (path.size() >= sizeof(addr.sun_path)) {
    close(fd);
    errno = ENAMETOOLONG;
    return -1;
  }
  std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
  unlink(path.c_str());
  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 || chmod(path.c_str(), 0660) < 0 || listen(fd, 3) < 0) {
    const int err = errno;
    close(fd);
    unlink(path.c_str());
    errno = err;
    return -1;
  }
  return fd;
}

}  // namespace

VisionIpcFrameLeaseClient::~VisionIpcFrameLeaseClient() {
  if (fd >= 0) close(fd);
}

bool VisionIpcFrameLeaseClient::connect(VisionStreamType camera_stream) {
  if (fd >= 0 || camera_slot(camera_stream) < 0) return false;

  const std::string path = get_ipc_path("camerad_frame_lease");
  fd = ipc_connect(path.c_str());
  if (fd < 0) return false;

  LeaseMessage hello = {};
  hello.type = MessageType::Hello;
  hello.stream = camera_stream;
  if (!send_message(fd, hello)) {
    close(fd);
    fd = -1;
    return false;
  }

  LeaseMessage response = {};
  if (!receive_message(fd, &response) || response.type != MessageType::Session ||
      response.stream != camera_stream || response.generation == 0) {
    close(fd);
    fd = -1;
    return false;
  }
  stream = camera_stream;
  session_generation = response.generation;
  return true;
}

bool VisionIpcFrameLeaseClient::acknowledge(uint32_t frame_id, uint64_t generation) {
  if (fd < 0 || generation == 0 || generation != session_generation) return false;
  LeaseMessage msg = {};
  msg.type = MessageType::Ack;
  msg.stream = stream;
  msg.frame_id = frame_id;
  msg.generation = generation;
  return send_message(fd, msg);
}

bool VisionIpcFrameLeaseClient::renew_session() {
  if (fd < 0 || session_generation == 0) return false;
  LeaseMessage msg = {};
  msg.type = MessageType::Renew;
  msg.stream = stream;
  msg.generation = session_generation;
  if (!send_message(fd, msg)) return false;

  LeaseMessage response = {};
  if (!receive_message(fd, &response) || response.type != MessageType::Session ||
      response.stream != stream || response.generation == 0 || response.generation == session_generation) {
    return false;
  }
  session_generation = response.generation;
  return true;
}

bool VisionIpcFrameLeaseClient::stop_cleanly() {
  if (fd < 0) return true;
  LeaseMessage msg = {};
  msg.type = MessageType::Stop;
  msg.stream = stream;
  msg.generation = session_generation;
  const bool sent = send_message(fd, msg);
  shutdown(fd, SHUT_RDWR);
  close(fd);
  fd = -1;
  session_generation = 0;
  stream = VISION_STREAM_MAX;
  return sent;
}

VisionIpcFrameLeaseServer::~VisionIpcFrameLeaseServer() {
  stop();
}

bool VisionIpcFrameLeaseServer::start(Callback cb) {
  if (worker.joinable()) return false;
  socket_path = get_ipc_path("camerad_frame_lease");
  listener_fd = make_listener(socket_path);
  if (listener_fd < 0) {
    LOGE("failed to bind camera frame lease socket %s: %s", socket_path.c_str(), std::strerror(errno));
    return false;
  }
  callback = std::move(cb);
  stopping = false;
  worker = std::thread(&VisionIpcFrameLeaseServer::run, this);
  return true;
}

void VisionIpcFrameLeaseServer::stop() {
  stopping = true;
  if (worker.joinable()) worker.join();
  if (listener_fd >= 0) {
    close(listener_fd);
    listener_fd = -1;
  }
  if (!socket_path.empty()) unlink(socket_path.c_str());
}

void VisionIpcFrameLeaseServer::close_client(Client &client, EventType reason) {
  if (client.fd < 0) return;
  if (reason == EventType::Disconnected && callback) {
    callback(reason, client.stream, 0, client.generation, 0);
  }
  close(client.fd);
  client = {};
}

void VisionIpcFrameLeaseServer::run() {
  std::array<Client, 3> clients = {};
  while (!stopping) {
    std::array<pollfd, 4> fds = {};
    fds[0] = {.fd = listener_fd, .events = POLLIN, .revents = 0};
    for (size_t i = 0; i < clients.size(); ++i) {
      fds[i + 1] = {.fd = clients[i].fd, .events = POLLIN, .revents = 0};
    }

    const int ret = poll(fds.data(), fds.size(), 100);
    if (ret < 0) {
      if (errno == EINTR) continue;
      if (!stopping) LOGE("camera frame lease poll failed: %s", std::strerror(errno));
      break;
    }
    if (stopping) break;

    if (fds[0].revents & POLLIN) {
      const int fd = accept(listener_fd, nullptr, nullptr);
      if (fd >= 0) {
        LeaseMessage hello = {};
        pollfd request = {.fd = fd, .events = POLLIN, .revents = 0};
        const int ready = poll(&request, 1, 1000);
        if (ready <= 0 || !(request.revents & POLLIN) || !receive_message(fd, &hello) ||
            hello.type != MessageType::Hello) {
          close(fd);
        } else {
          const auto stream = static_cast<VisionStreamType>(hello.stream);
          const int slot = camera_slot(stream);
          if (slot < 0 || clients[slot].fd >= 0) {
            LeaseMessage reject = {};
            reject.type = MessageType::Reject;
            reject.stream = hello.stream;
            (void)send_message(fd, reject);
            close(fd);
          } else {
            uint64_t generation = next_generation++;
            if (generation == 0) generation = next_generation++;
            clients[slot] = {.fd = fd, .stream = stream, .generation = generation};
            if (callback) callback(EventType::Started, stream, 0, 0, generation);
            LeaseMessage response = {};
            response.type = MessageType::Session;
            response.stream = stream;
            response.generation = generation;
            if (!send_message(fd, response)) close_client(clients[slot], EventType::Disconnected);
          }
        }
      }
    }

    for (size_t i = 0; i < clients.size(); ++i) {
      Client &client = clients[i];
      const short revents = fds[i + 1].revents;
      if (client.fd < 0 || revents == 0) continue;

      if (revents & POLLIN) {
        LeaseMessage msg = {};
        if (!receive_message(client.fd, &msg)) {
          close_client(client, EventType::Disconnected);
          continue;
        }
        if (msg.stream != client.stream || msg.generation != client.generation) {
          close_client(client, EventType::Disconnected);
          continue;
        }

        if (msg.type == MessageType::Ack) {
          if (callback) callback(EventType::Acknowledged, client.stream, msg.frame_id, client.generation, 0);
        } else if (msg.type == MessageType::Renew) {
          const uint64_t old_generation = client.generation;
          uint64_t new_generation = next_generation++;
          if (new_generation == 0) new_generation = next_generation++;
          client.generation = new_generation;
          if (callback) callback(EventType::Renewed, client.stream, 0, old_generation, new_generation);
          LeaseMessage response = {};
          response.type = MessageType::Session;
          response.stream = client.stream;
          response.generation = new_generation;
          if (!send_message(client.fd, response)) close_client(client, EventType::Disconnected);
        } else if (msg.type == MessageType::Stop) {
          if (callback) callback(EventType::Stopped, client.stream, 0, client.generation, 0);
          close(client.fd);
          client = {};
        } else {
          close_client(client, EventType::Disconnected);
        }
      } else if (revents & (POLLHUP | POLLERR | POLLNVAL)) {
        close_client(client, EventType::Disconnected);
      }
    }
  }

  for (Client &client : clients) {
    if (client.fd >= 0) {
      close(client.fd);
      client = {};
    }
  }
}
