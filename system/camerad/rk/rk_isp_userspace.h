#pragma once
// KA2 choice-2: userspace ISP control without rkaiq_3A.service.
// CamHw comes from in-process librkaiq; AE is owned by camerad (manual).

#include <cstdint>
#include <memory>
#include <string>
#include <ctime>
#include <sys/types.h>

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
  void apply_mwb(float r, float g, float b);
  void shutdown();

  bool active() const { return active_; }
  bool isp_started() const { return started_; }
  const std::string &mainpath_dev() const { return cfg_.mainpath_dev; }
  int stats_fd() const { return -1; }   // aiq owns stats
  int params_fd() const { return -1; }

  static void stop_rkaiq();
  static void start_rkaiq();
  static void set_multi_cam_count(int n);
  // Install camerad-owned JSON under /tmp/camerad_calib (never /etc/iqfiles).
  static bool ensure_runtime_calib();

private:
  RkIspCamConfig cfg_{};
  rk_aiq_sys_ctx_s *aiq_ = nullptr;
  bool active_ = false;
  bool started_ = false;

  float wb_last_r_ = -1.f, wb_last_g_ = -1.f, wb_last_b_ = -1.f;
  static int multi_cam_n_;
};
