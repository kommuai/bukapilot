#pragma once

#include <array>
#include <memory>
#include <mutex>

#include "system/camerad/camera/common.h"
#include "system/camerad/sensor/sensor.h"
#include "system/camerad/isp/camera_backend.h"

#define FRAME_BUF_COUNT 4

bool cameras_open(MultiCameraState *s);
void cameras_close(MultiCameraState *s);
void cameras_init(VisionIpcServer *v, MultiCameraState *s);
void cameras_run(MultiCameraState *s);

class CameraState {
public:
  std::unique_ptr<const SensorInfo> ci;
  bool enabled = false;
  std::unique_ptr<Ka2CameraBackend> ka2;

  unique_fd ctrl_fd;
  unique_fd video_fd;

  int camera_num = 0;
  bool rk_zerocopy_active = false;
  // Do not publish the partially-started sensor sequences. The three RK
  // streams are enabled sequentially, so the first few frame IDs are not
  // aligned even though the steady-state streams are locked to 20 FPS.
  uint8_t startup_discard_frames = 20;
  uint32_t first_published_sequence = 0;
  bool frame_id_initialized = false;

  struct PendingFrameLease {
    bool pending = false;
    bool processing_done = false;
    bool encoder_done = false;
    uint32_t frame_id = 0;
    uint64_t generation = 0;
  };
  std::mutex lease_mutex;
  std::mutex v4l2_ioctl_mutex;
  uint64_t active_lease_generation = 0;
  std::array<PendingFrameLease, FRAME_BUF_COUNT> pending_leases = {};

  struct v4l2_format fmt = {};
  struct v4l2_requestbuffers req = {};
  struct v4l2_buffer v4l_buf = {};
  struct v4l2_plane planes[1] = {};
  void camera_open(int camera_num);
  void camera_map_bufs();
  void camera_init(VisionIpcServer *v, VisionStreamType yuv_type);
  void dequeue_buf();
  void requeue_buf(int idx);
  void start_frame_lease(uint64_t generation);
  void acknowledge_frame(uint32_t frame_id, uint64_t generation);
  void renew_frame_lease(uint64_t old_generation, uint64_t new_generation);
  void stop_frame_lease(uint64_t generation);
  void fail_frame_lease(uint64_t generation);
  void finish_frame_processing(int idx, uint32_t frame_id, uint64_t generation);
  void cancel_unpublished_frame(int idx);
  void queue_all_buffers();
  void camera_close();

  CameraBuf buf;
  int frame_id_last = 0;

};

typedef struct MultiCameraState {
  CameraState road_cam;
  CameraState wide_road_cam;
  CameraState driver_cam;

  VisionIpcServer *vipc_server = nullptr;
  PubMaster *pm;
} MultiCameraState;
