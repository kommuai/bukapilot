#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/frame.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

typedef struct AVFilterContext AVFilterContext;
int av_buffersrc_add_frame_flags(AVFilterContext *buffer_src, AVFrame *frame, int flags);
int av_buffersink_get_frame_flags(AVFilterContext *buffer_sink, AVFrame *frame, int flags);

#define RING_CAP 16384u
#define RING_MAGIC UINT64_C(0x414334504950454c)

enum event_type {
  EVENT_AV_READ_FRAME = 1,
  EVENT_CODEC_SEND = 2,
  EVENT_CODEC_RECEIVE = 3,
  EVENT_FILTER_SOURCE = 4,
  EVENT_FILTER_SINK = 5,
  EVENT_MUX_WRITE = 6,
  EVENT_AVIO_WRITE = 7,
};

typedef struct {
  uint32_t type;
  uint32_t tid;
  int32_t result;
  int32_t flags;
  uint64_t begin_mono_ns;
  uint64_t end_mono_ns;
  uint64_t begin_cpu_ns;
  uint64_t end_cpu_ns;
  int64_t pts;
  int64_t dts;
  int64_t value;
} pipeline_event_t;

typedef struct {
  uint64_t magic;
  volatile uint64_t count;
  volatile uint64_t overflow;
  pipeline_event_t events[RING_CAP];
} pipeline_ring_t;

static pipeline_ring_t *ring = NULL;
static const size_t ring_bytes = sizeof(pipeline_ring_t);

static uint64_t clock_ns(clockid_t clock_id) {
  struct timespec ts;
  if (clock_gettime(clock_id, &ts) != 0) return 0;
  return (uint64_t)ts.tv_sec * UINT64_C(1000000000) + (uint64_t)ts.tv_nsec;
}

static void record_event(const pipeline_event_t *event) {
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
  const char *path = getenv("AC4_PIPELINE_RING");
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
  ring = (pipeline_ring_t *)memory;
  memset(ring, 0, sizeof(*ring));
  ring->magic = RING_MAGIC;
  __sync_synchronize();
}

__attribute__((destructor)) static void observer_fini(void) {
  if (ring != NULL) msync(ring, ring_bytes, MS_SYNC);
}

int av_read_frame(AVFormatContext *format, AVPacket *packet) {
  static int (*real_call)(AVFormatContext *, AVPacket *) = NULL;
  if (real_call == NULL) real_call = dlsym(RTLD_NEXT, "av_read_frame");
  if (real_call == NULL) return -ENOSYS;
  uint64_t begin_mono = clock_ns(CLOCK_MONOTONIC);
  uint64_t begin_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  int result = real_call(format, packet);
  uint64_t end_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  uint64_t end_mono = clock_ns(CLOCK_MONOTONIC);
  pipeline_event_t event = {.type = EVENT_AV_READ_FRAME, .tid = (uint32_t)syscall(SYS_gettid),
    .result = result, .begin_mono_ns = begin_mono, .end_mono_ns = end_mono,
    .begin_cpu_ns = begin_cpu, .end_cpu_ns = end_cpu,
    .pts = (result >= 0 && packet != NULL) ? packet->pts : AV_NOPTS_VALUE,
    .dts = (result >= 0 && packet != NULL) ? packet->dts : AV_NOPTS_VALUE,
    .value = (result >= 0 && packet != NULL) ? packet->size : 0};
  record_event(&event);
  return result;
}

int avcodec_send_packet(AVCodecContext *context, const AVPacket *packet) {
  static int (*real_call)(AVCodecContext *, const AVPacket *) = NULL;
  if (real_call == NULL) real_call = dlsym(RTLD_NEXT, "avcodec_send_packet");
  if (real_call == NULL) return -ENOSYS;
  uint64_t begin_mono = clock_ns(CLOCK_MONOTONIC);
  uint64_t begin_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  int result = real_call(context, packet);
  uint64_t end_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  uint64_t end_mono = clock_ns(CLOCK_MONOTONIC);
  pipeline_event_t event = {.type = EVENT_CODEC_SEND, .tid = (uint32_t)syscall(SYS_gettid),
    .result = result, .begin_mono_ns = begin_mono, .end_mono_ns = end_mono,
    .begin_cpu_ns = begin_cpu, .end_cpu_ns = end_cpu,
    .pts = packet != NULL ? packet->pts : AV_NOPTS_VALUE,
    .dts = packet != NULL ? packet->dts : AV_NOPTS_VALUE,
    .value = packet != NULL ? packet->size : 0};
  record_event(&event);
  return result;
}

int avcodec_receive_frame(AVCodecContext *context, AVFrame *frame) {
  static int (*real_call)(AVCodecContext *, AVFrame *) = NULL;
  if (real_call == NULL) real_call = dlsym(RTLD_NEXT, "avcodec_receive_frame");
  if (real_call == NULL) return -ENOSYS;
  uint64_t begin_mono = clock_ns(CLOCK_MONOTONIC);
  uint64_t begin_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  int result = real_call(context, frame);
  uint64_t end_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  uint64_t end_mono = clock_ns(CLOCK_MONOTONIC);
  pipeline_event_t event = {.type = EVENT_CODEC_RECEIVE, .tid = (uint32_t)syscall(SYS_gettid),
    .result = result, .begin_mono_ns = begin_mono, .end_mono_ns = end_mono,
    .begin_cpu_ns = begin_cpu, .end_cpu_ns = end_cpu,
    .pts = (result >= 0 && frame != NULL) ? frame->pts : AV_NOPTS_VALUE,
    .dts = AV_NOPTS_VALUE, .value = 0};
  record_event(&event);
  return result;
}

int av_buffersrc_add_frame_flags(AVFilterContext *buffer_src, AVFrame *frame, int flags) {
  static int (*real_call)(AVFilterContext *, AVFrame *, int) = NULL;
  if (real_call == NULL) real_call = dlsym(RTLD_NEXT, "av_buffersrc_add_frame_flags");
  if (real_call == NULL) return -ENOSYS;
  int64_t pts = frame != NULL ? frame->pts : AV_NOPTS_VALUE;
  uint64_t begin_mono = clock_ns(CLOCK_MONOTONIC);
  uint64_t begin_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  int result = real_call(buffer_src, frame, flags);
  uint64_t end_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  uint64_t end_mono = clock_ns(CLOCK_MONOTONIC);
  pipeline_event_t event = {.type = EVENT_FILTER_SOURCE, .tid = (uint32_t)syscall(SYS_gettid),
    .result = result, .flags = flags, .begin_mono_ns = begin_mono, .end_mono_ns = end_mono,
    .begin_cpu_ns = begin_cpu, .end_cpu_ns = end_cpu,
    .pts = pts,
    .dts = AV_NOPTS_VALUE, .value = 0};
  record_event(&event);
  return result;
}

int av_buffersink_get_frame_flags(AVFilterContext *buffer_sink, AVFrame *frame, int flags) {
  static int (*real_call)(AVFilterContext *, AVFrame *, int) = NULL;
  if (real_call == NULL) real_call = dlsym(RTLD_NEXT, "av_buffersink_get_frame_flags");
  if (real_call == NULL) return -ENOSYS;
  uint64_t begin_mono = clock_ns(CLOCK_MONOTONIC);
  uint64_t begin_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  int result = real_call(buffer_sink, frame, flags);
  uint64_t end_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  uint64_t end_mono = clock_ns(CLOCK_MONOTONIC);
  pipeline_event_t event = {.type = EVENT_FILTER_SINK, .tid = (uint32_t)syscall(SYS_gettid),
    .result = result, .flags = flags, .begin_mono_ns = begin_mono, .end_mono_ns = end_mono,
    .begin_cpu_ns = begin_cpu, .end_cpu_ns = end_cpu,
    .pts = (result >= 0 && frame != NULL) ? frame->pts : AV_NOPTS_VALUE,
    .dts = AV_NOPTS_VALUE, .value = 0};
  record_event(&event);
  return result;
}

int av_interleaved_write_frame(AVFormatContext *format, AVPacket *packet) {
  static int (*real_call)(AVFormatContext *, AVPacket *) = NULL;
  if (real_call == NULL) real_call = dlsym(RTLD_NEXT, "av_interleaved_write_frame");
  if (real_call == NULL) return -ENOSYS;
  uint64_t begin_mono = clock_ns(CLOCK_MONOTONIC);
  uint64_t begin_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  int64_t pts = packet != NULL ? packet->pts : AV_NOPTS_VALUE;
  int64_t dts = packet != NULL ? packet->dts : AV_NOPTS_VALUE;
  int64_t size = packet != NULL ? packet->size : 0;
  int result = real_call(format, packet);
  uint64_t end_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  uint64_t end_mono = clock_ns(CLOCK_MONOTONIC);
  pipeline_event_t event = {.type = EVENT_MUX_WRITE, .tid = (uint32_t)syscall(SYS_gettid),
    .result = result, .begin_mono_ns = begin_mono, .end_mono_ns = end_mono,
    .begin_cpu_ns = begin_cpu, .end_cpu_ns = end_cpu,
    .pts = pts, .dts = dts, .value = size};
  record_event(&event);
  return result;
}

void avio_write(AVIOContext *context, const unsigned char *buffer, int size) {
  static void (*real_call)(AVIOContext *, const unsigned char *, int) = NULL;
  if (real_call == NULL) real_call = dlsym(RTLD_NEXT, "avio_write");
  if (real_call == NULL) return;
  uint64_t begin_mono = clock_ns(CLOCK_MONOTONIC);
  uint64_t begin_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  real_call(context, buffer, size);
  uint64_t end_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  uint64_t end_mono = clock_ns(CLOCK_MONOTONIC);
  pipeline_event_t event = {.type = EVENT_AVIO_WRITE, .tid = (uint32_t)syscall(SYS_gettid),
    .result = 0, .begin_mono_ns = begin_mono, .end_mono_ns = end_mono,
    .begin_cpu_ns = begin_cpu, .end_cpu_ns = end_cpu,
    .pts = AV_NOPTS_VALUE, .dts = AV_NOPTS_VALUE, .value = size};
  record_event(&event);
}
