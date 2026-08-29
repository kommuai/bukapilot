#pragma once

#include <memory>

#include "system/camerad/cameras/camera_common.h"
#include "system/camerad/sensors/sensor.h"
#include "system/camerad/rk/ka2_camera_backend.h"

void cameras_open(MultiCameraState *s);
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
  bool rk_zerocopy_requested = false;
  bool rk_zerocopy_active = false;
  uint8_t startup_discard_frames = 3;

  struct v4l2_format fmt = {};
  struct v4l2_requestbuffers req = {};
  struct v4l2_buffer v4l_buf = {};
  struct v4l2_plane planes[1] = {};
  void camera_open(int camera_num, bool enabled);
  void camera_map_bufs();
  void camera_init(VisionIpcServer *v, VisionStreamType yuv_type);
  void dequeue_buf();
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
