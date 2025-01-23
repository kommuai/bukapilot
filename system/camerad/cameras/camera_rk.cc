#include "system/camerad/cameras/camera_rk.h"

#include <poll.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "media/cam_defs.h"
#include "media/cam_isp.h"
#include "media/cam_isp_ife.h"
#include "media/cam_req_mgr.h"
#include "media/cam_sensor_cmn_header.h"
#include "media/cam_sync.h"
#include "common/swaglog.h"

extern ExitHandler do_exit;

void CameraState::camera_map_bufs(MultiCameraState *s) {
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
    buf.camera_bufs[i].addr = mmap(NULL, v4l_buf.m.planes[0].length,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED,
                                  video_fd, v4l_buf.m.planes[0].m.mem_offset);
    assert(buf.camera_bufs[i].addr != MAP_FAILED);
  }
}

void CameraState::camera_init(MultiCameraState *s, VisionIpcServer * v, cl_device_id device_id, cl_context ctx, VisionStreamType yuv_type) {
  if (!enabled) return;

  LOGD("camera init %d", camera_num);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  fmt.fmt.pix.width = 1920;
  fmt.fmt.pix.height = 1200;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  assert(ioctl(video_fd, VIDIOC_S_FMT, &fmt) >= 0);

  memset(&req, 0, sizeof(req));
  req.count = FRAME_BUF_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  req.memory = V4L2_MEMORY_MMAP;

  assert(ioctl(video_fd, VIDIOC_REQBUFS, &req) >= 0);

  buf.init(device_id, ctx, this, v, FRAME_BUF_COUNT, yuv_type);
  camera_map_bufs(s);
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
  // start v4l2 buffer queue
  LOG("-- Start Queueing V4L2 buffers");
  for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
    v4l_buf.index = i;
    assert(ioctl(video_fd, VIDIOC_QBUF, &v4l_buf) >= 0);
  }

  // start streaming
  assert(ioctl(video_fd, VIDIOC_STREAMON, &fmt.type) >= 0);
}

void CameraState::dequeue_buf() {
  // TODO: move to another class
  ctrl.id = V4L2_CID_EXPOSURE;
  assert(ioctl(ctrl_fd, VIDIOC_G_CTRL, &ctrl) >= 0);
  buf.camera_bufs_metadata[v4l_buf.index].integ_lines = ctrl.value;

  assert(ioctl(video_fd, VIDIOC_DQBUF, &v4l_buf) >= 0);
  // queue the index number of the v4l buffer that has just been populated
  buf.queue(v4l_buf.index);

  buf.camera_bufs_metadata[v4l_buf.index].frame_id = v4l_buf.sequence;
  buf.camera_bufs_metadata[v4l_buf.index].timestamp_sof = static_cast<uint64_t>(v4l_buf.timestamp.tv_sec * 1000000000 + v4l_buf.timestamp.tv_usec * 1000);
  buf.camera_bufs_metadata[v4l_buf.index].timestamp_eof = static_cast<uint64_t>(v4l_buf.timestamp.tv_sec * 1000000000 + v4l_buf.timestamp.tv_usec * 1000);
  // immediately queue after dequeing the buffer
  assert(ioctl(video_fd, VIDIOC_QBUF, &v4l_buf) >= 0);
}

void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {
  s->driver_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_DRIVER);
  s->road_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_ROAD);
  s->wide_road_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_WIDE_ROAD);

  s->pm = new PubMaster({"roadCameraState", "driverCameraState", "wideRoadCameraState", "thumbnail"});
}

void cameras_open(MultiCameraState *s) {
  LOG("-- Opening devices");
  s->wide_road_cam.camera_open(s, 2, !env_disable_wide_road);
  LOGD("wide road camera opened");
  s->road_cam.camera_open(s, 1, !env_disable_road);
  LOGD("road camera opened");
  s->driver_cam.camera_open(s, 0, !env_disable_driver);
  LOGD("driver camera opened");
 }

void CameraState::camera_close() {
  // stop devices
  LOG("-- Stop devices %d", camera_num);

  for(int i = 0; i < FRAME_BUF_COUNT; i++) {
    munmap(buf.camera_bufs[i].addr, buf.camera_bufs[i].mmap_len);
  }

  close(ctrl_fd);
  close(video_fd);

  LOGD("destroyed session %d", camera_num);
}

void cameras_close(MultiCameraState *s) {
  s->driver_cam.camera_close();
  s->road_cam.camera_close();
  s->wide_road_cam.camera_close();

  delete s->pm;
}

static void process_driver_camera(MultiCameraState *s, CameraState *c, int cnt) {
  MessageBuilder msg;
  auto framed = msg.initEvent().initDriverCameraState();
  framed.setFrameType(cereal::FrameData::FrameType::FRONT);
  fill_frame_data(framed, c->buf.cur_frame_data, c);

  s->pm->send("driverCameraState", msg);
}


void process_road_camera(MultiCameraState *s, CameraState *c, int cnt) {
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

void cameras_run(MultiCameraState *s) {
  LOG("-- Starting threads");
  std::vector<std::thread> threads;
  if (s->driver_cam.enabled) threads.push_back(start_process_thread(s, &s->driver_cam, process_driver_camera));
  if (s->road_cam.enabled) threads.push_back(start_process_thread(s, &s->road_cam, process_road_camera));
  if (s->wide_road_cam.enabled) threads.push_back(start_process_thread(s, &s->wide_road_cam, process_road_camera));

  s->driver_cam.stream_start();
  s->road_cam.stream_start();
  s->wide_road_cam.stream_start();

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
            break;
          case 2:
            s->wide_road_cam.dequeue_buf();
            break;
        }
      }
    }
  }

  LOG(" ************** STOPPING **************");

  for (auto &t : threads) t.join();

  cameras_close(s);
}

