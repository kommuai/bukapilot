#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "common/util.h"
#include "system/camerad/cameras/camera_common.h"
#include "system/camerad/rk/rk_isp_userspace.h"
#include "system/camerad/sensors/sensor.h"

struct MultiCameraState;
class CameraState;

class Ka2CameraBackend {
public:
  bool open(CameraState *cam);
  void close(CameraState *cam);
  void on_stream_start(CameraState *cam);
  void on_dequeue(CameraState *cam, FrameMetadata &md, int buf_idx);
  void apply_flips(CameraState *cam);
  void log_init_summary(const CameraState *cam) const;

  RkIspUserspaceController *isp() { return rk_isp_.get(); }
  const RkIspUserspaceController *isp() const { return rk_isp_.get(); }

  static void prepare_system(MultiCameraState *s);
  static void prepare_isp_all(MultiCameraState *s);
  static void synced_stream_and_start(MultiCameraState *s);

private:
  float get_gain_factor(const CameraState *cam) const;
  void update_exposure_score(CameraState *cam, float desired_ev, int exp_t, int exp_g_idx, float exp_gain);
  void apply_pwl_on(CameraState *cam);
  void sensors_i2c(CameraState *cam, const i2c_random_wr_payload *dat, int len);
  void set_exposure_rect(CameraState *cam);
  void set_camera_exposure(CameraState *cam, float grey_frac);
  void apply_fixed_exposure(CameraState *cam, int exp_t, int gidx, bool hcg);
  bool read_ctrl(const CameraState *cam, uint32_t id, int *out) const;
  bool write_ctrl(const CameraState *cam, uint32_t id, int val) const;
  void apply_sensor_exposure_hw(CameraState *cam, int exp_t, int gidx, bool dc_gain);
  std::string resolve_mainpath_dev(int camera_num) const;

  std::unique_ptr<RkIspUserspaceController> rk_isp_;
  std::mutex exp_lock_;
  std::mutex i2c_lock_;
  unique_fd i2c_fd_;
  int i2c_bus_ = -1;
  int i2c_addr_ = 0x36;
  int exposure_time_ = 5;
  bool dc_gain_enabled_ = false;
  int dc_gain_weight_ = 1;
  int gain_idx_ = 0;
  float analog_gain_frac_ = 1.0f;
  float cur_ev_[3] = {};
  float best_ev_score_ = 1e6f;
  int new_exp_g_ = 0;
  int new_exp_t_ = 0;
  float measured_grey_fraction_ = 0.f;
  float target_grey_fraction_ = 0.125f;
  float fl_pix_ = 0.f;
  Rect ae_xywh_ = {};
  bool ae_roi_ready_ = false;
  std::vector<i2c_random_wr_payload> last_exp_regs_;
  float last_sensor_temp_c_ = -999.0f;
};
