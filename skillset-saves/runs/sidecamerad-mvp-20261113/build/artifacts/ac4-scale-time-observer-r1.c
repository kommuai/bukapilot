#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <libavutil/frame.h>
#include <libswscale/swscale.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#define RING_CAP 8192u
#define RING_MAGIC UINT64_C(0x4143345343414c45)

enum event_type { EVENT_SCALE_FRAME = 1, EVENT_SCALE_SLICE = 2 };

typedef struct {
  uint32_t type;
  uint32_t tid;
  int32_t result;
  int32_t reserved;
  uint64_t begin_mono_ns;
  uint64_t end_mono_ns;
  uint64_t begin_cpu_ns;
  uint64_t end_cpu_ns;
  int64_t src_pts;
  int64_t dst_pts;
} scale_event_t;

typedef struct {
  uint64_t magic;
  volatile uint64_t count;
  volatile uint64_t overflow;
  scale_event_t events[RING_CAP];
} scale_ring_t;

static scale_ring_t *ring = NULL;
static const size_t ring_bytes = sizeof(scale_ring_t);
static __thread unsigned int scale_frame_depth = 0;

static uint64_t clock_ns(clockid_t clock_id) {
  struct timespec ts;
  if (clock_gettime(clock_id, &ts) != 0) return 0;
  return (uint64_t)ts.tv_sec * UINT64_C(1000000000) + (uint64_t)ts.tv_nsec;
}

static void record_event(const scale_event_t *event) {
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
  const char *path = getenv("AC4_SCALE_RING");
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
  ring = (scale_ring_t *)memory;
  memset(ring, 0, sizeof(*ring));
  ring->magic = RING_MAGIC;
  __sync_synchronize();
}

__attribute__((destructor)) static void observer_fini(void) {
  if (ring != NULL) msync(ring, ring_bytes, MS_SYNC);
}

int sws_scale_frame(struct SwsContext *context, AVFrame *dst, const AVFrame *src) {
  static int (*real_scale_frame)(struct SwsContext *, AVFrame *, const AVFrame *) = NULL;
  if (real_scale_frame == NULL) real_scale_frame = dlsym(RTLD_NEXT, "sws_scale_frame");
  if (real_scale_frame == NULL) return -ENOSYS;

  uint64_t begin_mono = clock_ns(CLOCK_MONOTONIC);
  uint64_t begin_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  ++scale_frame_depth;
  int result = real_scale_frame(context, dst, src);
  --scale_frame_depth;
  uint64_t end_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  uint64_t end_mono = clock_ns(CLOCK_MONOTONIC);

  scale_event_t event = {
    .type = EVENT_SCALE_FRAME,
    .tid = (uint32_t)syscall(SYS_gettid),
    .result = result,
    .begin_mono_ns = begin_mono,
    .end_mono_ns = end_mono,
    .begin_cpu_ns = begin_cpu,
    .end_cpu_ns = end_cpu,
    .src_pts = (src != NULL) ? src->pts : INT64_MIN,
    .dst_pts = (dst != NULL) ? dst->pts : INT64_MIN,
  };
  record_event(&event);
  return result;
}

int sws_scale(struct SwsContext *context, const uint8_t *const src_slice[],
              const int src_stride[], int src_slice_y, int src_slice_height,
              uint8_t *const dst[], const int dst_stride[]) {
  static int (*real_scale)(struct SwsContext *, const uint8_t *const[], const int[],
                           int, int, uint8_t *const[], const int[]) = NULL;
  if (real_scale == NULL) real_scale = dlsym(RTLD_NEXT, "sws_scale");
  if (real_scale == NULL) return -ENOSYS;

  uint64_t begin_mono = clock_ns(CLOCK_MONOTONIC);
  uint64_t begin_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  int result = real_scale(context, src_slice, src_stride, src_slice_y, src_slice_height,
                          dst, dst_stride);
  uint64_t end_cpu = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  uint64_t end_mono = clock_ns(CLOCK_MONOTONIC);

  /* sws_scale_frame may invoke sws_scale internally; count only the outer call. */
  if (scale_frame_depth == 0) {
    scale_event_t event = {
      .type = EVENT_SCALE_SLICE,
      .tid = (uint32_t)syscall(SYS_gettid),
      .result = result,
      .begin_mono_ns = begin_mono,
      .end_mono_ns = end_mono,
      .begin_cpu_ns = begin_cpu,
      .end_cpu_ns = end_cpu,
      .src_pts = INT64_MIN,
      .dst_pts = INT64_MIN,
    };
    record_event(&event);
  }
  return result;
}
