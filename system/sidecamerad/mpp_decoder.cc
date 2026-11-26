// SPDX-License-Identifier: MIT

#include "mpp_decoder_mpi.h"

#include <linux/videodev2.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

volatile sig_atomic_t stopping = 0;

void handle_signal(int) {
  stopping = 1;
}

void install_signal_handlers() {
  struct sigaction action = {};
  action.sa_handler = handle_signal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
  signal(SIGPIPE, SIG_IGN);
}

int xioctl(int fd, unsigned long request, void *arg) {
  int ret;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret < 0 && errno == EINTR && !stopping);
  return ret;
}

void check_ioctl(int fd, unsigned long request, void *arg, const char *what) {
  if (xioctl(fd, request, arg) < 0) {
    throw std::runtime_error(std::string(what) + ": " + strerror(errno));
  }
}

class V4L2Capture {
public:
  V4L2Capture(const char *device, uint32_t width, uint32_t height, uint32_t fps)
      : device_(device) {
    fd_ = open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error(std::string("open ") + device + ": " + strerror(errno));
    }

    try {
      configure(width, height, fps);
    } catch (...) {
      stop();
      close_device();
      throw;
    }
  }

  ~V4L2Capture() {
    stop();
    close_device();
  }

  V4L2Capture(const V4L2Capture &) = delete;
  V4L2Capture &operator=(const V4L2Capture &) = delete;

  bool dequeue(uint32_t *index, const uint8_t **data, size_t *length) {
    struct pollfd pfd = {.fd = fd_, .events = POLLIN};
    int ready;
    do {
      ready = poll(&pfd, 1, 500);
    } while (ready < 0 && errno == EINTR && !stopping);

    if (stopping) return false;
    if (ready < 0) throw std::runtime_error(std::string("poll: ") + strerror(errno));
    if (ready == 0) return false;
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      throw std::runtime_error("V4L2 capture poll reported device error");
    }

    struct v4l2_buffer buffer = {};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
      if (errno == EAGAIN || (errno == EINTR && !stopping)) return false;
      if (stopping && errno == EINTR) return false;
      throw std::runtime_error(std::string("VIDIOC_DQBUF: ") + strerror(errno));
    }
    if (buffer.index >= buffers_.size()) {
      throw std::runtime_error("V4L2 returned an invalid buffer index");
    }
    if (buffer.flags & V4L2_BUF_FLAG_ERROR) {
      queue(buffer.index);
      return false;
    }
    if (buffer.bytesused == 0 || buffer.bytesused > buffers_[buffer.index].length) {
      queue(buffer.index);
      throw std::runtime_error("V4L2 returned an invalid MJPEG frame length");
    }

    *index = buffer.index;
    *data = static_cast<const uint8_t *>(buffers_[buffer.index].address);
    *length = buffer.bytesused;
    return true;
  }

  void queue(uint32_t index) {
    struct v4l2_buffer buffer = {};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    check_ioctl(fd_, VIDIOC_QBUF, &buffer, "VIDIOC_QBUF");
  }

  size_t max_frame_size() const {
    size_t max_size = image_size_;
    for (const auto &buffer : buffers_) max_size = std::max(max_size, buffer.length);
    return max_size;
  }

private:
  struct Buffer {
    void *address = MAP_FAILED;
    size_t length = 0;
  };

  void configure(uint32_t width, uint32_t height, uint32_t fps) {
    struct v4l2_capability capability = {};
    check_ioctl(fd_, VIDIOC_QUERYCAP, &capability, "VIDIOC_QUERYCAP");
    const uint32_t caps = (capability.capabilities & V4L2_CAP_DEVICE_CAPS)
                            ? capability.device_caps : capability.capabilities;
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE) || !(caps & V4L2_CAP_STREAMING)) {
      throw std::runtime_error("V4L2 device must support video capture streaming");
    }

    struct v4l2_format format = {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    check_ioctl(fd_, VIDIOC_S_FMT, &format, "VIDIOC_S_FMT");
    if (format.fmt.pix.width != width || format.fmt.pix.height != height ||
        format.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG || format.fmt.pix.sizeimage == 0) {
      throw std::runtime_error("V4L2 camera rejected the requested MJPEG mode");
    }
    image_size_ = format.fmt.pix.sizeimage;

    struct v4l2_streamparm parameters = {};
    parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parameters.parm.capture.timeperframe.numerator = 1;
    parameters.parm.capture.timeperframe.denominator = fps;
    check_ioctl(fd_, VIDIOC_S_PARM, &parameters, "VIDIOC_S_PARM");

    struct v4l2_requestbuffers request = {};
    request.count = 4;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    check_ioctl(fd_, VIDIOC_REQBUFS, &request, "VIDIOC_REQBUFS");
    if (request.count < 2) throw std::runtime_error("V4L2 allocated fewer than two capture buffers");
    buffers_.resize(request.count);

    for (uint32_t index = 0; index < request.count; ++index) {
      struct v4l2_buffer buffer = {};
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = index;
      check_ioctl(fd_, VIDIOC_QUERYBUF, &buffer, "VIDIOC_QUERYBUF");
      buffers_[index].length = buffer.length;
      buffers_[index].address = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                                      MAP_SHARED, fd_, buffer.m.offset);
      if (buffers_[index].address == MAP_FAILED) {
        throw std::runtime_error(std::string("mmap V4L2 buffer: ") + strerror(errno));
      }
      queue(index);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    check_ioctl(fd_, VIDIOC_STREAMON, &type, "VIDIOC_STREAMON");
    streaming_ = true;
  }

  void stop() {
    if (fd_ >= 0 && streaming_) {
      enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(fd_, VIDIOC_STREAMOFF, &type);
      streaming_ = false;
    }
    for (auto &buffer : buffers_) {
      if (buffer.address != MAP_FAILED) {
        munmap(buffer.address, buffer.length);
        buffer.address = MAP_FAILED;
      }
    }
    buffers_.clear();
  }

  void close_device() {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  std::string device_;
  int fd_ = -1;
  size_t image_size_ = 0;
  bool streaming_ = false;
  std::vector<Buffer> buffers_;
};

class MjpegDecoder {
public:
  MjpegDecoder(uint32_t width, uint32_t height, size_t input_size)
      : frame_size_(static_cast<size_t>(width) * height * 3 / 2) {
    char error[256] = {};
    decoder_ = sidecam_mpp_decoder_create(width, height, input_size, error, sizeof(error));
    if (!decoder_) throw std::runtime_error(error[0] ? error : "could not create MPP decoder");
  }

  ~MjpegDecoder() { sidecam_mpp_decoder_destroy(decoder_); }

  MjpegDecoder(const MjpegDecoder &) = delete;
  MjpegDecoder &operator=(const MjpegDecoder &) = delete;

  const uint8_t *decode(const uint8_t *jpeg, size_t jpeg_size) {
    const uint8_t *frame = nullptr;
    size_t decoded_size = 0;
    char error[256] = {};
    if (sidecam_mpp_decoder_decode(decoder_, jpeg, jpeg_size, &frame, &decoded_size,
                                   error, sizeof(error)) != 0) {
      throw std::runtime_error(error[0] ? error : "MPP decode failed");
    }
    if (decoded_size != frame_size_) throw std::runtime_error("MPP returned an invalid NV12 frame length");
    return frame;
  }

  size_t buffer_bytes() const { return sidecam_mpp_decoder_buffer_bytes(decoder_); }

private:
  SidecamMppDecoder *decoder_ = nullptr;
  size_t frame_size_;
};

int write_all(int fd, const uint8_t *data, size_t length) {
  while (length > 0 && !stopping) {
    const ssize_t written = write(fd, data, length);
    if (written < 0 && errno == EINTR) continue;
    if (written <= 0) return -1;
    data += written;
    length -= static_cast<size_t>(written);
  }
  return stopping ? 1 : 0;
}

uint32_t parse_positive(const char *text, const char *name) {
  char *end = nullptr;
  errno = 0;
  const unsigned long value = strtoul(text, &end, 10);
  if (errno || end == text || *end != '\0' || value == 0 || value > UINT32_MAX) {
    throw std::runtime_error(std::string("invalid ") + name);
  }
  return static_cast<uint32_t>(value);
}

bool jpeg_payload(const uint8_t *buffer, size_t length, const uint8_t **jpeg, size_t *jpeg_length) {
  if (!buffer || length < 4) return false;
  size_t start = length;
  for (size_t i = 0; i + 1 < length; ++i) {
    if (buffer[i] == 0xff && buffer[i + 1] == 0xd8) {
      start = i;
      break;
    }
  }
  if (start == length) return false;

  size_t end = 0;
  for (size_t i = length - 1; i > start; --i) {
    if (buffer[i - 1] == 0xff && buffer[i] == 0xd9) {
      end = i + 1;
      break;
    }
  }
  if (end <= start + 2) return false;
  *jpeg = buffer + start;
  *jpeg_length = end - start;
  return true;
}

int run(const char *device, uint32_t width, uint32_t height, uint32_t fps) {
  V4L2Capture capture(device, width, height, fps);
  MjpegDecoder decoder(width, height, capture.max_frame_size());
  fprintf(stderr, "sidecamerad MPP capture started device=%s width=%u height=%u fps=%u\n",
          device, width, height, fps);

  unsigned consecutive_invalid_jpegs = 0;
  bool reported_buffer_memory = false;
  while (!stopping) {
    uint32_t index = 0;
    const uint8_t *jpeg = nullptr;
    size_t jpeg_size = 0;
    if (!capture.dequeue(&index, &jpeg, &jpeg_size)) continue;

    const uint8_t *jpeg_data = nullptr;
    size_t payload_size = 0;
    if (!jpeg_payload(jpeg, jpeg_size, &jpeg_data, &payload_size)) {
      capture.queue(index);
      if (++consecutive_invalid_jpegs >= 10) {
        throw std::runtime_error("ten consecutive V4L2 buffers did not contain a complete JPEG");
      }
      continue;
    }
    consecutive_invalid_jpegs = 0;

    try {
      const uint8_t *frame = decoder.decode(jpeg_data, payload_size);
      if (!reported_buffer_memory) {
        fprintf(stderr, "sidecamerad MPP buffer bytes=%zu\n", decoder.buffer_bytes());
        reported_buffer_memory = true;
      }
      if (frame && write_all(STDOUT_FILENO, frame, static_cast<size_t>(width) * height * 3 / 2) < 0) {
        throw std::runtime_error(std::string("write NV12 frame: ") + strerror(errno));
      }
    } catch (...) {
      capture.queue(index);
      throw;
    }
    capture.queue(index);
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 5) {
    fprintf(stderr, "usage: %s DEVICE WIDTH HEIGHT FPS\n", argv[0]);
    return 2;
  }

  install_signal_handlers();
  try {
    return run(argv[1], parse_positive(argv[2], "width"), parse_positive(argv[3], "height"),
               parse_positive(argv[4], "fps"));
  } catch (const std::exception &error) {
    fprintf(stderr, "sidecamerad MPP capture failed: %s\n", error.what());
    return stopping ? 0 : 1;
  }
}
