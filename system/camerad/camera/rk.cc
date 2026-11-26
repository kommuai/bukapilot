#include "system/camerad/camera/rk.h"

#include <array>
#include <poll.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <cassert>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <vector>

#include "common/swaglog.h"
#include "msgq/visionipc/frame_lease.h"

extern ExitHandler do_exit;

void CameraState::camera_map_bufs() {
  int exported_count = 0;
  for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
    memset(&v4l_buf, 0, sizeof(v4l_buf));
    v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l_buf.memory = V4L2_MEMORY_MMAP;
    v4l_buf.index = i;
    v4l_buf.length = 1;
    v4l_buf.m.planes = planes;

    assert(ioctl(video_fd, VIDIOC_QUERYBUF, &v4l_buf) >= 0);

    buf.camera_bufs[i].mmap_len = v4l_buf.m.planes[0].length;
    buf.camera_bufs[i].len = v4l_buf.m.planes[0].length;
    buf.camera_bufs[i].fd = -1;
    buf.camera_bufs[i].addr = mmap(NULL, v4l_buf.m.planes[0].length,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED,
                                  video_fd, v4l_buf.m.planes[0].m.mem_offset);
    assert(buf.camera_bufs[i].addr != MAP_FAILED);
    buf.camera_bufs[i].init_yuv(buf.rgb_width, buf.rgb_height, buf.rgb_width, (size_t)buf.rgb_width * buf.rgb_height);

    struct v4l2_exportbuffer exp = {};
    exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    exp.index = i;
    exp.plane = 0;
    exp.flags = O_CLOEXEC | O_RDWR;
    if (ioctl(video_fd, VIDIOC_EXPBUF, &exp) == 0) {
      buf.camera_bufs[i].fd = exp.fd;
      buf.camera_bufs[i].frame_id_in_buf = false;
      exported_count++;
    } else {
      LOGE("camera %d: VIDIOC_EXPBUF failed idx=%d errno=%d '%s'; KA2 encoder DMA-BUF is required",
           camera_num, i, errno, strerror(errno));
    }
  }

  rk_zerocopy_active = exported_count == FRAME_BUF_COUNT;
  LOG("camera %d: RK zero-copy active=%d exported=%d/%d",
      camera_num, rk_zerocopy_active, exported_count, FRAME_BUF_COUNT);
  if (!rk_zerocopy_active) {
    for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
      if (buf.camera_bufs[i].fd >= 0) {
        close(buf.camera_bufs[i].fd);
        buf.camera_bufs[i].fd = -1;
      }
      buf.camera_bufs[i].frame_id_in_buf = true;
    }
  }
}

void CameraState::camera_init(VisionIpcServer *v, VisionStreamType yuv_type) {
  if (!enabled) return;
  rk_zerocopy_active = false;

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  fmt.fmt.pix.width = 1920;
  fmt.fmt.pix.height = 1200;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(video_fd, VIDIOC_S_FMT, &fmt) < 0) {
    int err = errno;
    int vfd = video_fd;
    LOGE("camera %d: VIDIOC_S_FMT failed on fd %d (errno=%d '%s'), disabling camera",
         camera_num, vfd, err, strerror(err));
    enabled = false;
    return;
  }

  memset(&req, 0, sizeof(req));
  req.count = FRAME_BUF_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  req.memory = V4L2_MEMORY_MMAP;
  assert(ioctl(video_fd, VIDIOC_REQBUFS, &req) >= 0);

  buf.init(v, FRAME_BUF_COUNT, yuv_type);
  camera_map_bufs();
  if (!rk_zerocopy_active) {
    LOGE("camera %d: refusing to start without all V4L2 DMA-BUF exports", camera_num);
    do_exit = true;
    return;
  }
  buf.setupVipcBuffers(true, encoder_stream_for_camera(yuv_type));
}

void CameraState::camera_open(int camera_num_) {
  camera_num = camera_num_;
  enabled = true;

  ka2 = std::make_unique<Ka2CameraBackend>();
  if (!ka2->open(this)) {
    LOGE("camera %d: KA2 backend open failed", camera_num);
    enabled = false;
    ka2.reset();
  }
}

void CameraState::queue_all_buffers() {
  if (!enabled) return;
  for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
    memset(&v4l_buf, 0, sizeof(v4l_buf));
    memset(planes, 0, sizeof(planes));
    v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l_buf.memory = V4L2_MEMORY_MMAP;
    v4l_buf.length = 1;
    v4l_buf.m.planes = planes;
    v4l_buf.index = i;
    if (ioctl(video_fd, VIDIOC_QBUF, &v4l_buf) < 0) {
      LOGE("camera %d: VIDIOC_QBUF failed during stream start idx=%d errno=%d '%s'",
           camera_num, i, errno, strerror(errno));
      enabled = false;
      return;
    }
  }
}

void CameraState::dequeue_buf() {
  if (!enabled) return;

  memset(&v4l_buf, 0, sizeof(v4l_buf));
  memset(planes, 0, sizeof(planes));
  v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  v4l_buf.memory = V4L2_MEMORY_MMAP;
  v4l_buf.length = 1;
  v4l_buf.m.planes = planes;

  int dq_ret;
  {
    std::lock_guard ioctl_lock(v4l2_ioctl_mutex);
    dq_ret = ioctl(video_fd, VIDIOC_DQBUF, &v4l_buf);
  }
  if (dq_ret < 0) {
    LOGE("camera %d: VIDIOC_DQBUF failed errno=%d '%s'", camera_num, errno, strerror(errno));
    return;
  }

  int idx = v4l_buf.index;
  FrameMetadata &md = buf.camera_bufs_metadata[idx];
  const uint64_t capture_time = static_cast<uint64_t>(v4l_buf.timestamp.tv_sec * 1000000000 + v4l_buf.timestamp.tv_usec * 1000);
  md.timestamp_sof = capture_time;
  md.timestamp_eof = capture_time + (ci ? ci->readout_time_ns : 0);

  md.dequeue_monotonic_ns = monotonic_time_ns();

  if (startup_discard_frames > 0) {
    --startup_discard_frames;
    requeue_buf(idx);
  } else {
    if (!frame_id_initialized) {
      first_published_sequence = v4l_buf.sequence;
      frame_id_initialized = true;
    }
    md.frame_id = v4l_buf.sequence - first_published_sequence + 1;
    md.request_id = md.frame_id;
    if (ka2) ka2->on_dequeue(this, md);
    {
      std::lock_guard lease_lock(lease_mutex);
      md.lease_generation = rk_zerocopy_active ? active_lease_generation : 0;
      if (md.lease_generation != 0) {
        pending_leases[idx] = {
          .pending = true,
          .processing_done = false,
          .encoder_done = false,
          .frame_id = md.frame_id,
          .generation = md.lease_generation,
        };
      }
    }
    const int dropped_idx = buf.queue(idx);
    if (dropped_idx >= 0) cancel_unpublished_frame(dropped_idx);
  }

  buf.record_dequeue_latency(monotonic_time_ns() - md.dequeue_monotonic_ns);
  if (md.frame_id % 120 == 0) {
    LOG("camera %d capture: queue=%zu/%d peak=%zu dropped=%llu dq_to_enqueue_us=%llu max_us=%llu",
        camera_num, buf.queue_size(), 3, buf.queue_peak(),
        (unsigned long long)buf.dropped_frames(),
        (unsigned long long)((monotonic_time_ns() - md.dequeue_monotonic_ns) / 1000),
        (unsigned long long)(buf.max_dequeue_latency() / 1000));
  }
}

void CameraState::requeue_buf(int idx) {
  if (!enabled || idx < 0 || idx >= FRAME_BUF_COUNT) return;

  struct v4l2_buffer buffer = {};
  struct v4l2_plane plane = {};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.length = 1;
  buffer.m.planes = &plane;
  buffer.index = idx;
  std::lock_guard ioctl_lock(v4l2_ioctl_mutex);
  if (ioctl(video_fd, VIDIOC_QBUF, &buffer) < 0) {
    LOGE("camera %d: VIDIOC_QBUF failed idx=%d errno=%d '%s'", camera_num, idx, errno, strerror(errno));
  }
}

void CameraState::start_frame_lease(uint64_t generation) {
  std::lock_guard lease_lock(lease_mutex);
  if (generation == 0 || active_lease_generation != 0) {
    LOGE("camera %d: invalid frame lease start generation=%llu active=%llu",
         camera_num, static_cast<unsigned long long>(generation),
         static_cast<unsigned long long>(active_lease_generation));
    do_exit = true;
    return;
  }
  active_lease_generation = generation;
  LOG("camera %d: encoder DMA-BUF lease session started generation=%llu",
      camera_num, static_cast<unsigned long long>(generation));
}

void CameraState::acknowledge_frame(uint32_t frame_id, uint64_t generation) {
  int requeue_idx = -1;
  bool found = false;
  {
    std::lock_guard lease_lock(lease_mutex);
    for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
      PendingFrameLease &lease = pending_leases[i];
      if (lease.pending && lease.frame_id == frame_id && lease.generation == generation) {
        found = true;
        lease.encoder_done = true;
        if (lease.processing_done) {
          lease = {};
          requeue_idx = i;
        }
        break;
      }
    }
  }
  if (!found) {
    LOGE("camera %d: encoder acknowledged unknown DMA-BUF frame=%u generation=%llu",
         camera_num, frame_id, static_cast<unsigned long long>(generation));
    do_exit = true;
    return;
  }
  if (requeue_idx >= 0) requeue_buf(requeue_idx);
}

void CameraState::renew_frame_lease(uint64_t old_generation, uint64_t new_generation) {
  std::array<int, FRAME_BUF_COUNT> requeue = {};
  size_t requeue_count = 0;
  {
    std::lock_guard lease_lock(lease_mutex);
    if (old_generation == 0 || new_generation == 0 || active_lease_generation != old_generation) {
      LOGE("camera %d: invalid frame lease renewal old=%llu new=%llu active=%llu",
           camera_num, static_cast<unsigned long long>(old_generation),
           static_cast<unsigned long long>(new_generation),
           static_cast<unsigned long long>(active_lease_generation));
      do_exit = true;
      return;
    }
    active_lease_generation = new_generation;
    for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
      PendingFrameLease &lease = pending_leases[i];
      if (lease.pending && lease.generation == old_generation) {
        lease.encoder_done = true;
        if (lease.processing_done) {
          lease = {};
          requeue[requeue_count++] = i;
        }
      }
    }
  }
  for (size_t i = 0; i < requeue_count; ++i) requeue_buf(requeue[i]);
  LOG("camera %d: encoder DMA-BUF lease session renewed generation=%llu",
      camera_num, static_cast<unsigned long long>(new_generation));
}

void CameraState::stop_frame_lease(uint64_t generation) {
  std::array<int, FRAME_BUF_COUNT> requeue = {};
  size_t requeue_count = 0;
  {
    std::lock_guard lease_lock(lease_mutex);
    if (generation == 0 || active_lease_generation != generation) {
      LOGE("camera %d: invalid frame lease stop generation=%llu active=%llu",
           camera_num, static_cast<unsigned long long>(generation),
           static_cast<unsigned long long>(active_lease_generation));
      do_exit = true;
      return;
    }
    active_lease_generation = 0;
    for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
      PendingFrameLease &lease = pending_leases[i];
      if (lease.pending && lease.generation == generation) {
        lease.encoder_done = true;
        if (lease.processing_done) {
          lease = {};
          requeue[requeue_count++] = i;
        }
      }
    }
  }
  for (size_t i = 0; i < requeue_count; ++i) requeue_buf(requeue[i]);
  LOG("camera %d: encoder DMA-BUF lease session stopped generation=%llu",
      camera_num, static_cast<unsigned long long>(generation));
}

void CameraState::fail_frame_lease(uint64_t generation) {
  LOGE("camera %d: encoder lease connection lost generation=%llu; stopping camera to protect DMA-BUF ownership",
       camera_num, static_cast<unsigned long long>(generation));
  do_exit = true;
}

void CameraState::finish_frame_processing(int idx, uint32_t frame_id, uint64_t generation) {
  if (generation == 0) {
    requeue_buf(idx);
    return;
  }

  bool requeue = false;
  {
    std::lock_guard lease_lock(lease_mutex);
    if (idx < 0 || idx >= FRAME_BUF_COUNT) {
      do_exit = true;
      return;
    }
    PendingFrameLease &lease = pending_leases[idx];
    if (!lease.pending || lease.frame_id != frame_id || lease.generation != generation) {
      LOGE("camera %d: missing DMA-BUF lease for buffer=%d frame=%u generation=%llu",
           camera_num, idx, frame_id, static_cast<unsigned long long>(generation));
      do_exit = true;
      return;
    }
    lease.processing_done = true;
    if (lease.encoder_done) {
      lease = {};
      requeue = true;
    }
  }
  if (requeue) requeue_buf(idx);
}

void CameraState::cancel_unpublished_frame(int idx) {
  if (idx >= 0 && idx < FRAME_BUF_COUNT) {
    std::lock_guard lease_lock(lease_mutex);
    pending_leases[idx] = {};
  }
  requeue_buf(idx);
}

void cameras_init(VisionIpcServer *v, MultiCameraState *s) {
  s->vipc_server = v;
  s->driver_cam.camera_init(v, VISION_STREAM_DRIVER);
  s->road_cam.camera_init(v, VISION_STREAM_ROAD);
  s->wide_road_cam.camera_init(v, VISION_STREAM_WIDE_ROAD);
  s->pm = new PubMaster({"roadCameraState", "driverCameraState", "wideRoadCameraState", "thumbnail"});
}

bool cameras_open(MultiCameraState *s) {
  if (!Ka2CameraBackend::prepare_system(s)) return false;
  Ka2CameraBackend::prepare_isp_all(s);
  return true;
}

void CameraState::camera_close() {
  // Stop the AE worker before releasing the V4L2 buffers it may be reading.
  // The worker owns the dequeue-to-requeue interval, so unmapping first can
  // turn a normal shutdown into a use-after-unmap crash.
  if (ka2) {
    ka2->close(this);
    ka2.reset();
  }

  if (video_fd.fd_ >= 0) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(video_fd, VIDIOC_STREAMOFF, &type) < 0) {
      LOGW("camera %d: VIDIOC_STREAMOFF failed errno=%d '%s'", camera_num, errno, strerror(errno));
    }
  }

  if (buf.camera_bufs) {
    for (int i = 0; i < FRAME_BUF_COUNT; i++) {
      if (buf.camera_bufs[i].fd >= 0) {
        close(buf.camera_bufs[i].fd);
        buf.camera_bufs[i].fd = -1;
      }
      if (buf.camera_bufs[i].addr != nullptr && buf.camera_bufs[i].addr != MAP_FAILED && buf.camera_bufs[i].mmap_len > 0) {
        munmap(buf.camera_bufs[i].addr, buf.camera_bufs[i].mmap_len);
        buf.camera_bufs[i].addr = nullptr;
        buf.camera_bufs[i].mmap_len = 0;
      }
    }
  }

  if (ctrl_fd.fd_ >= 0) {
    close(ctrl_fd.fd_);
    ctrl_fd.fd_ = -1;
  }
  if (video_fd.fd_ >= 0) {
    close(video_fd.fd_);
    video_fd.fd_ = -1;
  }
}

void cameras_close(MultiCameraState *s) {
  s->driver_cam.camera_close();
  s->road_cam.camera_close();
  s->wide_road_cam.camera_close();
  delete s->pm;
}

static void process_driver_camera(MultiCameraState *s, CameraState *c) {
  MessageBuilder msg;
  auto framed = msg.initEvent().initDriverCameraState();
  fill_frame_data(framed, c->buf.cur_frame_data);
  s->pm->send("driverCameraState", msg);
}

static void process_road_camera(MultiCameraState *s, CameraState *c) {
  const CameraBuf *b = &c->buf;
  MessageBuilder msg;
  auto framed = c == &s->road_cam ? msg.initEvent().initRoadCameraState() : msg.initEvent().initWideRoadCameraState();
  fill_frame_data(framed, b->cur_frame_data);
  s->pm->send(c == &s->road_cam ? "roadCameraState" : "wideRoadCameraState", msg);
}

void cameras_run(MultiCameraState *s) {
  VisionIpcFrameLeaseServer lease_server;
  const auto camera_for_stream = [s](VisionStreamType stream) -> CameraState * {
    switch (stream) {
      case VISION_STREAM_ROAD: return &s->road_cam;
      case VISION_STREAM_DRIVER: return &s->driver_cam;
      case VISION_STREAM_WIDE_ROAD: return &s->wide_road_cam;
      default: return nullptr;
    }
  };
  if (!lease_server.start([&](VisionIpcFrameLeaseServer::EventType event, VisionStreamType stream,
                              uint32_t frame_id, uint64_t old_generation, uint64_t new_generation) {
        CameraState *camera = camera_for_stream(stream);
        if (!camera) {
          LOGE("frame lease event for unsupported camera stream %d", static_cast<int>(stream));
          do_exit = true;
          return;
        }
        switch (event) {
          case VisionIpcFrameLeaseServer::EventType::Started:
            camera->start_frame_lease(new_generation);
            break;
          case VisionIpcFrameLeaseServer::EventType::Acknowledged:
            camera->acknowledge_frame(frame_id, old_generation);
            break;
          case VisionIpcFrameLeaseServer::EventType::Renewed:
            camera->renew_frame_lease(old_generation, new_generation);
            break;
          case VisionIpcFrameLeaseServer::EventType::Stopped:
            camera->stop_frame_lease(old_generation);
            break;
          case VisionIpcFrameLeaseServer::EventType::Disconnected:
            camera->fail_frame_lease(old_generation);
            break;
        }
      })) {
    do_exit = true;
  }

  std::vector<std::thread> threads;
  if (s->driver_cam.enabled) threads.push_back(start_process_thread(s, &s->driver_cam, process_driver_camera));
  if (s->road_cam.enabled) threads.push_back(start_process_thread(s, &s->road_cam, process_road_camera));
  if (s->wide_road_cam.enabled) threads.push_back(start_process_thread(s, &s->wide_road_cam, process_road_camera));

  Ka2CameraBackend::synced_stream_and_start(s);

  while (!do_exit) {
    pollfd fds[3] = {};
    CameraState *cams[3] = {&s->driver_cam, &s->road_cam, &s->wide_road_cam};
    int video_idx[3];
    int nfds = 0;
    for (int i = 0; i < 3; i++) {
      video_idx[i] = -1;
      if (!cams[i]->enabled) continue;
      video_idx[i] = nfds;
      fds[nfds++] = {.fd = cams[i]->video_fd, .events = POLLPRI | POLLIN, .revents = 0};
    }

    if (nfds == 0) {
      usleep(10000);
      continue;
    }
    int ret = poll(fds, nfds, 1000);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      LOGE("poll failed (%d - %d)", ret, errno);
      break;
    }

    for (int i = 0; i < 3; i++) {
      if (video_idx[i] >= 0 && (fds[video_idx[i]].revents & (POLLPRI | POLLIN))) {
        cams[i]->dequeue_buf();
      }
    }
  }

  for (auto &t : threads) t.join();
  lease_server.stop();
  if (s->vipc_server) s->vipc_server->stop_listener();
  stop_thumbnail_worker();
  cameras_close(s);
}
