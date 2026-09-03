#pragma once

#include <memory>

#include "system/camerad/camera/common.h"
#include "system/camerad/sensor/sensor.h"
#include "system/camerad/isp/camera_backend.h"

bool cameras_open(MultiCameraState *s);
void cameras_close(MultiCameraState *s);
void cameras_init(VisionIpcServer *v, MultiCameraState *s);
void cameras_run(MultiCameraState *s);

#define FRAME_BUF_COUNT 4

class CameraState {
public:
  std::unique_ptr<const SensorInfo> ci;
  bool enabled = false;
  std::unique_ptr<Ka2CameraBackend> ka2;

  unique_fd ctrl_fd;
  unique_fd video_fd;

  int camera_num = 0;
  bool rk_zerocopy_requested = true;
  bool rk_zerocopy_active = false;
  // Do not publish the partially-started sensor sequences. The three RK
  // streams are enabled sequentially, so the first few frame IDs are not
  // aligned even though the steady-state streams are locked to 20 FPS.
  uint8_t startup_discard_frames = 20;
  uint32_t first_published_sequence = 0;
  bool frame_id_initialized = false;

  struct v4l2_format fmt = {};
  struct v4l2_requestbuffers req = {};
  struct v4l2_buffer v4l_buf = {};
  struct v4l2_plane planes[1] = {};
  void camera_open(int camera_num, bool enabled);
  void camera_map_bufs();
  void camera_init(VisionIpcServer *v, VisionStreamType yuv_type);
  void dequeue_buf();
  void requeue_buf(int idx);
  void queue_all_buffers();
  void camera_close();

  CameraBuf buf;
  int frame_id_last = 0;

private:
};

typedef struct MultiCameraState {
  CameraState road_cam;
  CameraState wide_road_cam;
  CameraState driver_cam;

  PubMaster *pm;
} MultiCameraState;
