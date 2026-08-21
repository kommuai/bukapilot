#pragma once
// KA2 choice-2: userspace ISP control without rkaiq_3A.service.
// CamHw comes from in-process librkaiq; AE is owned by camerad (manual).

#include <cstdint>
#include <memory>
#include <string>

struct rk_aiq_sys_ctx_s;

struct RkIspCamConfig {
  int camera_num = 0;
  int sensor_ctrl_fd = -1;
  std::string mainpath_dev;  // e.g. /dev/video42
};

class RkIspUserspaceController {
public:
  RkIspUserspaceController() = default;
  ~RkIspUserspaceController() { shutdown(); }

  bool init(const RkIspCamConfig &cfg);
  bool prepare_and_start();  // after all cams inited (mul-cam)
  void enable_frame_ae(bool on);
  void on_frame_grey(float grey_frac);
  void shutdown();

  bool active() const { return active_; }
  int stats_fd() const { return -1; }   // aiq owns stats
  int params_fd() const { return -1; }

  static void stop_rkaiq();
  static void start_rkaiq();
  static void set_multi_cam_count(int n);
  // Install camerad-owned JSON under /tmp/camerad_calib (never /etc/iqfiles).
  static bool ensure_runtime_calib();

private:
  void seed_exposure_from_sensor();
  bool set_sensor_exposure(int exp_lines, int again);
  void ae_update(float grey_frac);

  RkIspCamConfig cfg_{};
  rk_aiq_sys_ctx_s *aiq_ = nullptr;
  bool active_ = false;
  bool frame_ae_ = false;
  bool started_ = false;

  float target_grey_ = 0.1f;
  float cur_ev_[3] = {100.f, 100.f, 100.f};
  int exposure_time_ = 100;
  int again_ = 16;
  int frame_i_ = 0;

  static int multi_cam_n_;
};
