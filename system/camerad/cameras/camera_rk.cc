#include "system/camerad/cameras/camera_rk.h"

#include <poll.h>
#include <sys/ioctl.h>

#include <cassert>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <vector>

#include "media/cam_defs.h"
#include "media/cam_isp.h"
#include "media/cam_isp_ife.h"
#include "media/cam_req_mgr.h"
#include "media/cam_sensor_cmn_header.h"
#include "media/cam_sync.h"
#include "common/swaglog.h"
#include "common/timing.h"

extern ExitHandler do_exit;

void CameraState::camera_map_bufs(MultiCameraState *s) {
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

    if (rk_zerocopy_requested) {
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
        LOGW("camera %d: VIDIOC_EXPBUF failed idx=%d errno=%d '%s', disabling rk zerocopy",
             camera_num, i, errno, strerror(errno));
      }
    }
  }

  rk_zerocopy_active = rk_zerocopy_requested && (exported_count == FRAME_BUF_COUNT);
  if (!rk_zerocopy_active) {
    for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
      if (buf.camera_bufs[i].fd >= 0) {
        close(buf.camera_bufs[i].fd);
        buf.camera_bufs[i].fd = -1;
      }
      buf.camera_bufs[i].frame_id_in_buf = true;
    }
  } else {
    LOGD("camera %d: rk zerocopy enabled with %d exported buffers", camera_num, exported_count);
  }
}

void CameraState::camera_init(MultiCameraState *s, VisionIpcServer * v, cl_device_id device_id, cl_context ctx, VisionStreamType yuv_type) {
  if (!enabled) return;
  rk_zerocopy_requested = false;
  rk_zerocopy_active = false;

  LOGD("camera init %d", camera_num);

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

  buf.init(device_id, ctx, this, v, FRAME_BUF_COUNT, yuv_type);
  camera_map_bufs(s);
  buf.setupVipcBuffers(rk_zerocopy_active);
}

void CameraState::camera_open(MultiCameraState *multi_cam_state_, int camera_num_, bool enabled_) {
  multi_cam_state = multi_cam_state_;
  camera_num = camera_num_;
  enabled = enabled_;
  if (!enabled) return;

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

void CameraState::stream_start() {
  if (!enabled) return;
  queue_all_buffers();
  if (!enabled) return;

  if (ioctl(video_fd, VIDIOC_STREAMON, &fmt.type) < 0) {
    LOGE("camera %d: VIDIOC_STREAMON failed errno=%d '%s'", camera_num, errno, strerror(errno));
    enabled = false;
    return;
  }
  if (ka2) ka2->on_stream_start(this);
}

void CameraState::dequeue_buf() {
  if (!enabled) return;

  memset(&v4l_buf, 0, sizeof(v4l_buf));
  memset(planes, 0, sizeof(planes));
  v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  v4l_buf.memory = V4L2_MEMORY_MMAP;
  v4l_buf.length = 1;
  v4l_buf.m.planes = planes;

  if (ioctl(video_fd, VIDIOC_DQBUF, &v4l_buf) < 0) {
    LOGE("camera %d: VIDIOC_DQBUF failed errno=%d '%s'", camera_num, errno, strerror(errno));
    return;
  }

  int idx = v4l_buf.index;
  FrameMetadata &md = buf.camera_bufs_metadata[idx];
  md.frame_id = v4l_buf.sequence;
  md.request_id = v4l_buf.sequence;
  cap_time = static_cast<uint64_t>(v4l_buf.timestamp.tv_sec * 1000000000 + v4l_buf.timestamp.tv_usec * 1000);
  md.timestamp_sof = cap_time;
  md.timestamp_eof = cap_time;

  if (ka2) ka2->on_dequeue(this, md, idx);

  buf.queue(idx);

  if (ioctl(video_fd, VIDIOC_QBUF, &v4l_buf) < 0) {
    LOGE("camera %d: VIDIOC_QBUF failed post-dequeue errno=%d '%s'", camera_num, errno, strerror(errno));
  }
}

void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {
  s->driver_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_DRIVER);
  s->road_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_ROAD);
  s->wide_road_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_WIDE_ROAD);
  s->pm = new PubMaster({"roadCameraState", "driverCameraState", "wideRoadCameraState", "thumbnail"});
}

void cameras_open(MultiCameraState *s) {
  Ka2CameraBackend::prepare_system(s);
  Ka2CameraBackend::prepare_isp_all(s);
}

void CameraState::camera_close() {
  LOG("-- Stop devices %d", camera_num);

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

  if (ka2) {
    ka2->close(this);
    ka2.reset();
  }

  if (ctrl_fd.fd_ >= 0) {
    close(ctrl_fd.fd_);
    ctrl_fd.fd_ = -1;
  }
  if (video_fd.fd_ >= 0) {
    close(video_fd.fd_);
    video_fd.fd_ = -1;
  }
  if (csiphy_fd.fd_ >= 0) {
    close(csiphy_fd.fd_);
    csiphy_fd.fd_ = -1;
  }

  LOGD("destroyed session %d", camera_num);
}

void cameras_close(MultiCameraState *s) {
  s->driver_cam.camera_close();
  s->road_cam.camera_close();
  s->wide_road_cam.camera_close();
  delete s->pm;
}

static void process_driver_camera(MultiCameraState *s, CameraState *c, uint32_t cnt) {
  MessageBuilder msg;
  auto framed = msg.initEvent().initDriverCameraState();
  fill_frame_data(framed, c->buf.cur_frame_data, c);
  s->pm->send("driverCameraState", msg);
}

static void process_road_camera(MultiCameraState *s, CameraState *c, uint32_t cnt) {
  const CameraBuf *b = &c->buf;
  MessageBuilder msg;
  auto framed = c == &s->road_cam ? msg.initEvent().initRoadCameraState() : msg.initEvent().initWideRoadCameraState();
  fill_frame_data(framed, b->cur_frame_data, c);
  LOGT(c->buf.cur_frame_data.frame_id, "%s: Image set", c == &s->road_cam ? "RoadCamera" : "WideRoadCamera");
  s->pm->send(c == &s->road_cam ? "roadCameraState" : "wideRoadCameraState", msg);
}

#define THRESHOLD 10000000
bool check_timestamp_sync(uint64_t *t1, int len1, uint64_t *t2, int len2) {
  int i = 0, j = 0;
  while (i < len1 && j < len2) {
    uint64_t diff = t1[i] > t2[j] ? t1[i] - t2[j] : t2[j] - t1[i];
    if (diff <= THRESHOLD) return true;
    if (t1[i] < t2[j]) i++; else j++;
  }
  return false;
}

#define SYNC_CHECK_LEN 5
#define SYNC_CHECK_COUNT 40
void cameras_run(MultiCameraState *s) {
  LOG("-- Starting threads");
  std::vector<std::thread> threads;
  if (s->driver_cam.enabled) threads.push_back(start_process_thread(s, &s->driver_cam, process_driver_camera));
  if (s->road_cam.enabled) threads.push_back(start_process_thread(s, &s->road_cam, process_road_camera));
  if (s->wide_road_cam.enabled) threads.push_back(start_process_thread(s, &s->wide_road_cam, process_road_camera));

  Ka2CameraBackend::synced_stream_and_start(s);

  uint64_t road_cam_ts[SYNC_CHECK_LEN];
  uint64_t wide_cam_ts[SYNC_CHECK_LEN];
  int count = 0;

  LOG("-- Dequeueing Video events");
  while (!do_exit) {
    std::vector<pollfd> fds;
    CameraState *cams[3] = {&s->driver_cam, &s->road_cam, &s->wide_road_cam};
    int video_idx[3];
    for (int i = 0; i < 3; i++) {
      video_idx[i] = -1;
      if (!cams[i]->enabled) continue;
      video_idx[i] = (int)fds.size();
      fds.push_back({.fd = cams[i]->video_fd, .events = POLLPRI | POLLIN, .revents = 0});
    }

    if (fds.empty()) {
      usleep(10000);
      continue;
    }
    int ret = poll(fds.data(), fds.size(), 1000);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      LOGE("poll failed (%d - %d)", ret, errno);
      break;
    }

    for (int i = 0; i < 3; i++) {
      if (video_idx[i] >= 0 && (fds[video_idx[i]].revents & (POLLPRI | POLLIN))) {
        cams[i]->dequeue_buf();
        if (cams[i] == &s->road_cam) {
          count++;
          if (count <= (SYNC_CHECK_COUNT + SYNC_CHECK_LEN - 1) && count >= SYNC_CHECK_COUNT) {
            road_cam_ts[count % SYNC_CHECK_COUNT] = s->road_cam.cap_time;
          }
        } else if (cams[i] == &s->wide_road_cam) {
          if (count <= (SYNC_CHECK_COUNT + SYNC_CHECK_LEN - 1) && count >= SYNC_CHECK_COUNT) {
            wide_cam_ts[count % SYNC_CHECK_COUNT] = s->wide_road_cam.cap_time;
          }
        }
      }
    }
  }

  LOG("************** STOPPING **************");
  for (auto &t : threads) t.join();
  stop_thumbnail_worker();
  cameras_close(s);
}
