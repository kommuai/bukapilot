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

// Special defined
#define V4L2_CID_X3C_SENSOR_TEMPERATURE (V4L2_CID_USER_BASE + 0x100)

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

  // get temperature sensor
  ctrl.id = V4L2_CID_X3C_SENSOR_TEMPERATURE;
  if (ioctl(ctrl_fd, VIDIOC_G_CTRL, &ctrl) >= 0) {
    // temperature is stored as value in centi-degrees (e.g. 3569 = 35.69°C)
    buf.camera_bufs_metadata[v4l_buf.index].sensor_temp_c = ctrl.value / 100.0f;
  } else {
    buf.camera_bufs_metadata[v4l_buf.index].sensor_temp_c = -999.0f;  // mark as invalid
  }

  assert(ioctl(video_fd, VIDIOC_DQBUF, &v4l_buf) >= 0);
  // queue the index number of the v4l buffer that has just been populated
  buf.queue(v4l_buf.index);

  buf.camera_bufs_metadata[v4l_buf.index].frame_id = v4l_buf.sequence;
  cap_time = static_cast<uint64_t>(v4l_buf.timestamp.tv_sec * 1000000000 + v4l_buf.timestamp.tv_usec * 1000); 
  buf.camera_bufs_metadata[v4l_buf.index].timestamp_sof = cap_time;
  buf.camera_bufs_metadata[v4l_buf.index].timestamp_eof = cap_time;
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

  // restart rkaiq 3A server
  system("sudo killall -q rkaiq_3A_server || true");
  usleep(500000);  // blocks for 0.5 seconds
  system("sudo rkaiq_3A_server &");

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
      // check sync
      bool synced = check_timestamp_sync(road_cam_ts, SYNC_CHECK_LEN, wide_cam_ts, SYNC_CHECK_LEN);
      if (!synced && count >= 50) {
        LOG("Not synced!");
        //do_exit = true;
      }
    }
  }

  LOG("************** STOPPING **************");

  for (auto &t : threads) t.join();

  cameras_close(s);
}

