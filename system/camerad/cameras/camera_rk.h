#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "system/camerad/cameras/camera_common.h"
#include "system/camerad/sensors/sensor.h"
#include "common/clutil.h"
#include "common/params.h"
#include "common/util.h"
#include "system/camerad/rk/rk_isp_userspace.h"

void cameras_open(MultiCameraState *s);
void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx);
void cameras_run(MultiCameraState *s);

#define FRAME_BUF_COUNT 4

// Stub for RK: no QCOM/spectra allocator; camerad uses mmap'd V4L2 buffers only.
class MemoryManager {
public:
  void init(int) {}
  ~MemoryManager() = default;
  template <class T>
  auto alloc(int len, uint32_t *handle) {
    (void)len;
    *handle = 0;
    return std::unique_ptr<T, std::function<void(void *)>>(nullptr, [](void *) {});
  }
private:
  void *alloc_buf(int, uint32_t *) { return nullptr; }
  void free(void *) {}
};

class CameraState {
public:
  MultiCameraState *multi_cam_state = nullptr;
  std::unique_ptr<const SensorInfo> ci;
  bool enabled = false;

  std::mutex exp_lock;
  std::mutex i2c_lock;

  // Comma-parity AE state (camera_qcom2.cc @ 555f48c5)
  int exposure_time = 5;
  bool dc_gain_enabled = true;
  int dc_gain_weight = 1;
  int gain_idx = 0;
  float analog_gain_frac = 1.0f;
  float cur_ev[3] = {};
  float best_ev_score = 1e6f;
  int new_exp_g = 0;
  int new_exp_t = 0;
  float measured_grey_fraction = 0.f;
  float target_grey_fraction = 0.3f;
  Rect ae_xywh = {};
  int i2c_bus = -1;   // /dev/i2c-N
  int i2c_addr = 0x36;

  unique_fd ctrl_fd;
  unique_fd csiphy_fd;
  unique_fd video_fd;

  int camera_num = 0;
  bool rk_zerocopy_requested = false;
  std::unique_ptr<RkIspUserspaceController> rk_isp;
  bool rk_zerocopy_active = false;
  bool ext_ctrl_supported = true;
  uint32_t temp_poll_divider = 8;
  float last_sensor_temp_c = -999.0f;  // last good V4L2 temp; avoid -999 flicker

  uint64_t cap_time = 0;

  struct v4l2_format fmt = {};
  struct v4l2_requestbuffers req = {};
  struct v4l2_buffer v4l_buf = {};
  struct v4l2_plane planes[1] = {};
  struct v4l2_control ctrl = {};

  float get_gain_factor() const;
  void apply_pwl_on();
  void update_exposure_score(float desired_ev, int exp_t, int exp_g_idx, float exp_gain);
  void set_camera_exposure(float grey_frac);

  void sensors_start();

  void camera_open(MultiCameraState *multi_cam_state, int camera_num, bool enabled);
  // CamHw prepare/start can reset sensor flips; call again after start.
  void apply_sensor_flips();
  void sensor_set_parameters();
  void camera_map_bufs(MultiCameraState *s);
  void camera_init(MultiCameraState *s, VisionIpcServer *v, cl_device_id device_id, cl_context ctx, VisionStreamType yuv_type);
  void dequeue_buf();
  void stream_start();
  void camera_close();

  int32_t session_handle = 0;
  int32_t sensor_dev_handle = 0;
  int32_t isp_dev_handle = 0;
  int32_t csiphy_dev_handle = 0;

  int32_t link_handle = 0;

  int buf0_handle = 0;
  int buf_handle[FRAME_BUF_COUNT] = {};
  int sync_objs[FRAME_BUF_COUNT] = {};
  int request_ids[FRAME_BUF_COUNT] = {};
  int request_id_last = 0;
  int frame_id_last = 0;
  int idx_offset = 0;
  bool skipped = false;

  CameraBuf buf;
  MemoryManager mm;

  void config_isp(int io_mem_handle, int fence, int request_id, int buf0_mem_handle, int buf0_offset);
  void enqueue_req_multi(int start, int n, bool dp);
  void enqueue_buffer(int i, bool dp);
  int clear_req_queue();

  int sensors_init();
  void sensors_poke(int request_id);
  void sensors_i2c(const struct i2c_random_wr_payload* dat, int len, int op_code, bool data_word);

private:
  Params params;
};

typedef struct MultiCameraState {
  int device_iommu;
  int cdm_iommu;

  CameraState road_cam;
  CameraState wide_road_cam;
  CameraState driver_cam;

  PubMaster *pm;
} MultiCameraState;
