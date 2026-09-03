#pragma once
// KA2 choice-2: userspace ISP control without rkaiq_3A.service.
// CamHw comes from in-process librkaiq; AE is owned by camerad (manual).

#include <cstdint>
#include <memory>
#include <string>
#include <sys/types.h>

struct rk_aiq_sys_ctx_s;

struct RkIspCamConfig {
  int camera_num = 0;
  std::string mainpath_dev;  // e.g. /dev/video42
};

class RkIspUserspaceController {
public:
  RkIspUserspaceController() = default;
  ~RkIspUserspaceController() { shutdown(); }

  bool init(const RkIspCamConfig &cfg);
  bool prepare();              // sysctl_prepare (all cams before stream on)
  bool start();                // sysctl_start (barrier-gated multi-cam)
  bool set_external_exposure(uint32_t frame_id, int integration_lines, float analog_gain,
                             uint32_t sensor_gain_code, bool high_conversion_gain);
  void shutdown();

  bool active() const { return active_; }
  bool isp_started() const { return started_; }

  static void set_multi_cam_count(int n);
  static bool calibration_available();

private:
  bool register_calibration(const char *sensor_entity);
  void clear_calibration_registration();
  bool subscribe_params_events();
  void close_params_fd();

  RkIspCamConfig cfg_{};
  rk_aiq_sys_ctx_s *aiq_ = nullptr;
  int params_fd_ = -1;
  bool active_ = false;
  bool prepared_ = false;
  bool started_ = false;
  std::string calib_sensor_entity_;
  static int multi_cam_n_;
};
