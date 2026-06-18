#include "system/camerad/cameras/camera_rk.h"

#include <poll.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <vector>

#include "media/cam_defs.h"
#include "media/cam_isp.h"
#include "media/cam_isp_ife.h"
#include "media/cam_req_mgr.h"
#include "media/cam_sensor_cmn_header.h"
#include "media/cam_sync.h"
#include "third_party/linux/include/v4l2-controls.h"
#include "common/swaglog.h"
#include "common/timing.h"

// Special defined
#define V4L2_CID_X3C_SENSOR_TEMPERATURE (V4L2_CID_USER_BASE + 0x100)

extern ExitHandler do_exit;

static const bool env_disable_wide_road = (getenv("DISABLE_WIDE_ROAD") != nullptr);
static const bool env_disable_road = (getenv("DISABLE_ROAD") != nullptr);
static const bool env_disable_driver = (getenv("DISABLE_DRIVER") != nullptr);
static const bool env_log_raw_frames = (getenv("LOG_RAW_FRAMES") != nullptr);

static inline bool read_ctrl_fd(int fd, uint32_t id, int *out) {
  struct v4l2_control c = {};
  c.id = id;
  if (ioctl(fd, VIDIOC_G_CTRL, &c) < 0) return false;
  *out = c.value;
  return true;
}

void CameraState::camera_map_bufs(MultiCameraState *s) {
  int exported_count = 0;
  for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
    memset(&v4l_buf, 0, sizeof(v4l_buf));
    v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l_buf.memory = V4L2_MEMORY_MMAP;
    v4l_buf.index = i;
    v4l_buf.length = 1; //FMT_NUM_PLANES
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
  rk_zerocopy_requested = (getenv("CAMERAD_RK_ZEROCOPY") != nullptr);
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

  LOG("-- Setting camera ctrls");
  char device[32];

  // ctrl is at subdev 2,7,12
  snprintf(device, sizeof(device), "/dev/v4l-subdev%d", camera_num * 5 + 2);
  ctrl_fd = open(device, O_RDWR);
  assert(ctrl_fd >= 0);

  // set vflip = 1 to all cameras
  ctrl.id = V4L2_CID_HFLIP;
  ctrl.value = 0;
  assert(ioctl(ctrl_fd, VIDIOC_S_CTRL, &ctrl) >= 0);
  // set vflip = 1 to all cameras
  ctrl.id = V4L2_CID_VFLIP;
  ctrl.value = 1;
  assert(ioctl(ctrl_fd, VIDIOC_S_CTRL, &ctrl) >= 0);

  video_fd = open_v4l_by_name_and_index("rkisp_mainpath", camera_num);
  assert(video_fd >= 0);
}

void CameraState::stream_start() {
  if (!enabled) return;
  // start v4l2 buffer queue
  LOG("-- Start Queueing V4L2 buffers");
  for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
    memset(&v4l_buf, 0, sizeof(v4l_buf));
    memset(planes, 0, sizeof(planes));
    v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l_buf.memory = V4L2_MEMORY_MMAP;
    v4l_buf.length = 1;
    v4l_buf.m.planes = planes;
    v4l_buf.index = i;
    if (ioctl(video_fd, VIDIOC_QBUF, &v4l_buf) < 0) {
      LOGE("camera %d: VIDIOC_QBUF failed during stream start idx=%d errno=%d '%s' (mode=%s)",
           camera_num, i, errno, strerror(errno), "mmap");
      enabled = false;
      return;
    }
  }

  // start streaming
  if (ioctl(video_fd, VIDIOC_STREAMON, &fmt.type) < 0) {
    LOGE("camera %d: VIDIOC_STREAMON failed errno=%d '%s'", camera_num, errno, strerror(errno));
    enabled = false;
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
  if (ioctl(video_fd, VIDIOC_DQBUF, &v4l_buf) < 0) {
    LOGE("camera %d: VIDIOC_DQBUF failed errno=%d '%s'", camera_num, errno, strerror(errno));
    return;
  }

  const int idx = v4l_buf.index;
  FrameMetadata &md = buf.camera_bufs_metadata[idx];

  // Defaults used when control reads are unavailable.
  md.integ_lines = 0;
  md.gain = 1.0f;
  if (md.sensor_temp_c == 0.0f) md.sensor_temp_c = -999.0f;

  int exposure_val = 0, gain_val_raw = 0, analog_gain_val = 0;
  bool have_exposure = false, have_gain = false, have_analog = false;

  if (ext_ctrl_supported) {
    struct v4l2_ext_control ext_ctrls[3] = {};
    struct v4l2_ext_controls ext = {};
    ext.which = V4L2_CTRL_WHICH_CUR_VAL;
    ext.count = 3;
    ext.controls = ext_ctrls;
    ext_ctrls[0].id = V4L2_CID_EXPOSURE;
    ext_ctrls[1].id = V4L2_CID_GAIN;
    ext_ctrls[2].id = V4L2_CID_ANALOGUE_GAIN;

    if (ioctl(ctrl_fd, VIDIOC_G_EXT_CTRLS, &ext) == 0) {
      exposure_val = ext_ctrls[0].value;
      gain_val_raw = ext_ctrls[1].value;
      analog_gain_val = ext_ctrls[2].value;
      have_exposure = true;
      have_gain = gain_val_raw > 0;
      have_analog = analog_gain_val > 0;
    } else if (errno == EINVAL || errno == ENOTTY) {
      ext_ctrl_supported = false;
    }
  }

  if (!ext_ctrl_supported) {
    have_exposure = read_ctrl_fd(ctrl_fd, V4L2_CID_EXPOSURE, &exposure_val);
    have_gain = read_ctrl_fd(ctrl_fd, V4L2_CID_GAIN, &gain_val_raw) && gain_val_raw > 0;
    have_analog = read_ctrl_fd(ctrl_fd, V4L2_CID_ANALOGUE_GAIN, &analog_gain_val) && analog_gain_val > 0;
  }

  if (have_exposure) {
    md.integ_lines = exposure_val;
  }
  if (have_gain) {
    md.gain = static_cast<float>(gain_val_raw);
  } else if (have_analog) {
    md.gain = (analog_gain_val >= 65536) ? (analog_gain_val / 65536.0f) : static_cast<float>(analog_gain_val);
    if (md.gain < 0.01f) md.gain = 1.0f;
  } else {
    static bool gain_warned = false;
    if (!gain_warned) {
      gain_warned = true;
      LOGW("V4L2 gain readback not supported on this subdev (AE likely controlled by rkaiq); logging gain=1.0");
    }
  }

  // Temperature changes slowly, so avoid paying this ioctl every frame.
  if (v4l_buf.sequence % temp_poll_divider == 0) {
    int temp_raw = 0;
    if (read_ctrl_fd(ctrl_fd, V4L2_CID_X3C_SENSOR_TEMPERATURE, &temp_raw)) {
      md.sensor_temp_c = temp_raw / 100.0f;
    } else {
      md.sensor_temp_c = -999.0f;
    }
  }

  md.frame_id = v4l_buf.sequence;
  md.request_id = v4l_buf.sequence;
  cap_time = static_cast<uint64_t>(v4l_buf.timestamp.tv_sec * 1000000000 + v4l_buf.timestamp.tv_usec * 1000);
  md.timestamp_sof = cap_time;
  md.timestamp_eof = cap_time;

  buf.queue(idx);

  if (ioctl(video_fd, VIDIOC_QBUF, &v4l_buf) < 0) {
    LOGE("camera %d: VIDIOC_QBUF failed post-dequeue errno=%d '%s'", camera_num, errno, strerror(errno));
    return;
  }
}

void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {

  s->driver_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_DRIVER);
  s->road_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_ROAD);
  s->wide_road_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_WIDE_ROAD);

  s->pm = new PubMaster({"roadCameraState", "driverCameraState", "wideRoadCameraState", "thumbnail"});
}

void cameras_open(MultiCameraState *s) {
  LOG("-- Opening devices");
  s->wide_road_cam.camera_open(s, 0, !env_disable_wide_road);
  LOGD("wide road camera opened");
  s->road_cam.camera_open(s, 1, !env_disable_road);
  LOGD("road camera opened");
  s->driver_cam.camera_open(s, 2, !env_disable_driver);
  LOGD("driver camera opened");
 }

void CameraState::camera_close() {
  // stop devices
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

  // unique_fd does not auto-invalidate on manual close; clear fd_ to avoid double-close.
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

  // restart rkaiq 3A server
  system("sudo killall -q /usr/kommu/rkaiq_3A_server || true");
  usleep(2500000);  // blocks for 2.5 seconds
  system("sudo /usr/kommu/rkaiq_3A_server &");

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
  if (env_log_raw_frames && c == &s->road_cam && cnt % 100 == 5) {  // no overlap with qlog decimation
    framed.setImage(get_raw_frame_image(b));
  }
  LOGT(c->buf.cur_frame_data.frame_id, "%s: Image set", c == &s->road_cam ? "RoadCamera" : "WideRoadCamera");

  s->pm->send(c == &s->road_cam ? "roadCameraState" : "wideRoadCameraState", msg);
}


#define THRESHOLD 10000000
bool check_timestamp_sync(uint64_t *t1, int len1, uint64_t *t2, int len2) {
    int i = 0, j = 0;

    while (i < len1 && j < len2) {
        uint64_t diff = t1[i] > t2[j] ? t1[i] - t2[j] : t2[j] - t1[i];
        if (diff <= THRESHOLD) {
            return true;
        }

        // Move the pointer with smaller timestamp forward
        if (t1[i] < t2[j]) {
            i++;
        } else {
            j++;
        }
    }

    return false; // No match within threshold
}

#define SYNC_CHECK_LEN 5
#define SYNC_CHECK_COUNT 40
void cameras_run(MultiCameraState *s) {
  LOG("-- Starting threads");
  std::vector<std::thread> threads;
  if (s->driver_cam.enabled) threads.push_back(start_process_thread(s, &s->driver_cam, process_driver_camera));
  if (s->road_cam.enabled) threads.push_back(start_process_thread(s, &s->road_cam, process_road_camera));
  if (s->wide_road_cam.enabled) threads.push_back(start_process_thread(s, &s->wide_road_cam, process_road_camera));

  s->wide_road_cam.stream_start();
  s->road_cam.stream_start();
  s->driver_cam.stream_start();

  uint64_t road_cam_ts[SYNC_CHECK_LEN];
  uint64_t wide_cam_ts[SYNC_CHECK_LEN];
  int count = 0;

  // poll events
  LOG("-- Dequeueing Video events");
  while (!do_exit) {
    struct pollfd fds[3] = {
      { .fd = s->driver_cam.video_fd, .events = POLLPRI | POLLIN },
      { .fd = s->road_cam.video_fd, .events = POLLPRI | POLLIN },
      { .fd = s->wide_road_cam.video_fd, .events = POLLPRI | POLLIN }
    };

    int ret = poll(fds, std::size(fds), 1000);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      LOGE("poll failed (%d - %d)", ret, errno);
      break;
    }

    for (int i = 0; i < 3; i++) {
      if (fds[i].revents & (POLLPRI | POLLIN)) {
        // Dequeue buffers for the corresponding camera if the file descriptor is ready
        switch (i) {
          case 0:
            s->driver_cam.dequeue_buf();
            break;
          case 1:
            s->road_cam.dequeue_buf();
            count++;
            if (count <= (SYNC_CHECK_COUNT + SYNC_CHECK_LEN - 1) && count >= SYNC_CHECK_COUNT) {
              road_cam_ts[count % SYNC_CHECK_COUNT] = s->road_cam.cap_time;
            }
            break;
          case 2:
            s->wide_road_cam.dequeue_buf();
            if (count <= (SYNC_CHECK_COUNT + SYNC_CHECK_LEN - 1) && count >= SYNC_CHECK_COUNT) {
              wide_cam_ts[count % SYNC_CHECK_COUNT] = s->wide_road_cam.cap_time;
            }
            break;
        }
      }
    }
  }

  LOG("************** STOPPING **************");

  for (auto &t : threads) t.join();

  // Stop thumbnail worker before cameras_close() destroys PubMaster.
  stop_thumbnail_worker();
  cameras_close(s);
}

