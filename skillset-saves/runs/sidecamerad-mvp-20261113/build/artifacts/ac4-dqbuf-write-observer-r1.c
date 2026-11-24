#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/frame.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <sys/uio.h>
#include <unistd.h>

#define RING_CAP 65536u
#define RING_MAGIC UINT64_C(0x4143345452414345)

typedef struct AVFilterContext AVFilterContext;

enum event_type {
  EVENT_DQBUF = 1,
  EVENT_WRITE = 2,
  EVENT_WRITEV = 3,
  EVENT_DECODER = 4,
  EVENT_FILTER = 5,
  EVENT_MUX = 6,
  EVENT_AVIO = 7,
};

typedef struct {
  uint32_t type;
  uint32_t tid;
  int32_t fd;
  int32_t result;
  uint64_t begin_ns;
  uint64_t end_ns;
  uint64_t a;
  uint64_t b;
  uint64_t c;
  uint64_t d;
} event_t;

typedef struct {
  uint64_t magic;
  volatile uint64_t count;
  volatile uint64_t overflow;
  volatile uint64_t stdout_bytes;
  event_t events[RING_CAP];
} ring_t;

static ring_t *ring = NULL;
static size_t ring_bytes = sizeof(ring_t);

static uint64_t mono_ns(void) {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
  return (uint64_t)ts.tv_sec * UINT64_C(1000000000) + (uint64_t)ts.tv_nsec;
}

static void record_event(const event_t *event) {
  if (ring == NULL || ring->magic != RING_MAGIC) return;
  uint64_t index = __sync_fetch_and_add(&ring->count, 1);
  if (index >= RING_CAP) {
    __sync_fetch_and_add(&ring->overflow, 1);
    return;
  }
  ring->events[index] = *event;
  __sync_synchronize();
}

__attribute__((constructor)) static void observer_init(void) {
  const char *path = getenv("AC4_OBSERVER_RING");
  if (path == NULL || *path == '\0') return;
  int fd = (int)syscall(SYS_openat, AT_FDCWD, path, O_RDWR | O_CREAT | O_TRUNC, 0600);
  if (fd < 0) return;
  if (ftruncate(fd, (off_t)ring_bytes) != 0) {
    syscall(SYS_close, fd);
    return;
  }
  void *memory = mmap(NULL, ring_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  syscall(SYS_close, fd);
  if (memory == MAP_FAILED) return;
  ring = (ring_t *)memory;
  memset(ring, 0, sizeof(*ring));
  ring->magic = RING_MAGIC;
  __sync_synchronize();
}

__attribute__((destructor)) static void observer_fini(void) {
  if (ring != NULL) msync(ring, ring_bytes, MS_SYNC);
}

int ioctl(int fd, unsigned long request, ...) {
  static int (*real_ioctl)(int, unsigned long, void *) = NULL;
  if (real_ioctl == NULL) real_ioctl = dlsym(RTLD_NEXT, "ioctl");
  va_list ap;
  va_start(ap, request);
  void *arg = va_arg(ap, void *);
  va_end(ap);

  uint64_t begin = (request == VIDIOC_DQBUF) ? mono_ns() : 0;
  int result = real_ioctl(fd, request, arg);
  uint64_t end = (request == VIDIOC_DQBUF) ? mono_ns() : 0;
  if (request == VIDIOC_DQBUF && result == 0 && arg != NULL) {
    struct v4l2_buffer *buffer = (struct v4l2_buffer *)arg;
    uint64_t source_ns = (uint64_t)buffer->timestamp.tv_sec * UINT64_C(1000000000) +
                         (uint64_t)buffer->timestamp.tv_usec * UINT64_C(1000);
    event_t event = {
      .type = EVENT_DQBUF,
      .tid = (uint32_t)syscall(SYS_gettid),
      .fd = fd,
      .result = result,
      .begin_ns = begin,
      .end_ns = end,
      .a = buffer->sequence,
      .b = source_ns,
      .c = ((uint64_t)buffer->index << 32) | (uint64_t)buffer->bytesused,
      .d = buffer->flags,
    };
    record_event(&event);
  }
  return result;
}

ssize_t write(int fd, const void *buffer, size_t count) {
  static ssize_t (*real_write)(int, const void *, size_t) = NULL;
  if (real_write == NULL) real_write = dlsym(RTLD_NEXT, "write");
  uint64_t begin = (fd == STDOUT_FILENO) ? mono_ns() : 0;
  ssize_t result = real_write(fd, buffer, count);
  uint64_t end = (fd == STDOUT_FILENO) ? mono_ns() : 0;
  if (fd == STDOUT_FILENO) {
    uint64_t after = 0;
    uint64_t before = 0;
    if (result > 0 && ring != NULL) before = __sync_fetch_and_add(&ring->stdout_bytes, (uint64_t)result);
    after = before + ((result > 0) ? (uint64_t)result : 0);
    event_t event = {
      .type = EVENT_WRITE,
      .tid = (uint32_t)syscall(SYS_gettid),
      .fd = fd,
      .result = (int32_t)result,
      .begin_ns = begin,
      .end_ns = end,
      .a = before,
      .b = after,
      .c = count,
      .d = 0,
    };
    record_event(&event);
  }
  return result;
}

ssize_t writev(int fd, const struct iovec *iov, int iovcnt) {
  static ssize_t (*real_writev)(int, const struct iovec *, int) = NULL;
  if (real_writev == NULL) real_writev = dlsym(RTLD_NEXT, "writev");
  uint64_t requested = 0;
  for (int i = 0; i < iovcnt; ++i) requested += iov[i].iov_len;
  uint64_t begin = (fd == STDOUT_FILENO) ? mono_ns() : 0;
  ssize_t result = real_writev(fd, iov, iovcnt);
  uint64_t end = (fd == STDOUT_FILENO) ? mono_ns() : 0;
  if (fd == STDOUT_FILENO) {
    uint64_t before = 0;
    if (result > 0 && ring != NULL) before = __sync_fetch_and_add(&ring->stdout_bytes, (uint64_t)result);
    event_t event = {
      .type = EVENT_WRITEV,
      .tid = (uint32_t)syscall(SYS_gettid),
      .fd = fd,
      .result = (int32_t)result,
      .begin_ns = begin,
      .end_ns = end,
      .a = before,
      .b = before + ((result > 0) ? (uint64_t)result : 0),
      .c = requested,
      .d = (uint64_t)iovcnt,
    };
    record_event(&event);
  }
  return result;
}

int avcodec_receive_frame(AVCodecContext *context, AVFrame *frame) {
  static int (*real_receive_frame)(AVCodecContext *, AVFrame *) = NULL;
  if (real_receive_frame == NULL) real_receive_frame = dlsym(RTLD_NEXT, "avcodec_receive_frame");
  uint64_t begin = mono_ns();
  int result = real_receive_frame(context, frame);
  uint64_t end = mono_ns();
  if (result == 0 && frame != NULL) {
    event_t event = {
      .type = EVENT_DECODER,
      .tid = (uint32_t)syscall(SYS_gettid),
      .fd = -1,
      .result = result,
      .begin_ns = begin,
      .end_ns = end,
      .a = (uint64_t)frame->pts,
      .b = (uint64_t)frame->best_effort_timestamp,
      .c = (uint64_t)frame->pkt_dts,
      .d = 0,
    };
    record_event(&event);
  }
  return result;
}

int av_buffersink_get_frame_flags(AVFilterContext *context, AVFrame *frame, int flags) {
  static int (*real_get_frame_flags)(AVFilterContext *, AVFrame *, int) = NULL;
  if (real_get_frame_flags == NULL) real_get_frame_flags = dlsym(RTLD_NEXT, "av_buffersink_get_frame_flags");
  uint64_t begin = mono_ns();
  int result = real_get_frame_flags(context, frame, flags);
  uint64_t end = mono_ns();
  if (result == 0 && frame != NULL) {
    event_t event = {
      .type = EVENT_FILTER,
      .tid = (uint32_t)syscall(SYS_gettid),
      .fd = -1,
      .result = result,
      .begin_ns = begin,
      .end_ns = end,
      .a = (uint64_t)frame->pts,
      .b = (uint64_t)frame->best_effort_timestamp,
      .c = (uint64_t)frame->pkt_dts,
      .d = 0,
    };
    record_event(&event);
  }
  return result;
}

int av_interleaved_write_frame(AVFormatContext *format, AVPacket *packet) {
  static int (*real_write_frame)(AVFormatContext *, AVPacket *) = NULL;
  if (real_write_frame == NULL) real_write_frame = dlsym(RTLD_NEXT, "av_interleaved_write_frame");
  int64_t pts = (packet != NULL) ? packet->pts : AV_NOPTS_VALUE;
  int64_t dts = (packet != NULL) ? packet->dts : AV_NOPTS_VALUE;
  int size = (packet != NULL) ? packet->size : 0;
  uint64_t begin = mono_ns();
  int result = real_write_frame(format, packet);
  uint64_t end = mono_ns();
  event_t event = {
    .type = EVENT_MUX,
    .tid = (uint32_t)syscall(SYS_gettid),
    .fd = -1,
    .result = result,
    .begin_ns = begin,
    .end_ns = end,
    .a = (uint64_t)pts,
    .b = (uint64_t)dts,
    .c = (uint64_t)(uint32_t)size,
    .d = 0,
  };
  record_event(&event);
  return result;
}

void avio_write(AVIOContext *context, const unsigned char *buffer, int size) {
  static void (*real_avio_write)(AVIOContext *, const unsigned char *, int) = NULL;
  if (real_avio_write == NULL) real_avio_write = dlsym(RTLD_NEXT, "avio_write");
  uint64_t begin = mono_ns();
  real_avio_write(context, buffer, size);
  uint64_t end = mono_ns();
  event_t event = {
    .type = EVENT_AVIO,
    .tid = (uint32_t)syscall(SYS_gettid),
    .fd = -1,
    .result = 0,
    .begin_ns = begin,
    .end_ns = end,
    .a = (uint64_t)(uint32_t)size,
    .b = 0,
    .c = 0,
    .d = 0,
  };
  record_event(&event);
}
