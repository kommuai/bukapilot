#pragma once
// KA2 choice-2: userspace ISP control without rkaiq_3A.service.
// CamHw comes from in-process librkaiq; AE is owned by camerad (manual).

#include <cstdint>
#include <array>
#include <memory>
#include <map>
#include <mutex>
#include <string>
#include <sys/types.h>

#include <uAPI2/rk_aiq_user_api2_custom_ae.h>

struct rk_aiq_sys_ctx_s;
using RkIspAeRunCallback = int32_t (*)(void *, const rk_aiq_customAe_stats_t *,
                                       rk_aiq_customeAe_results_t *);

struct RkIspCamConfig {
  int camera_num = 0;
  std::string mainpath_dev;  // e.g. /dev/video42
  void *ae_owner = nullptr;
  RkIspAeRunCallback ae_run = nullptr;
};

// Compact snapshot of the ISP AE statistics used by the existing AE shadow
// logging in camera_backend.cc.
struct RkIspAeShadowStats {
  uint32_t frame_id = 0;
  uint32_t histogram_median[4] = {};
  uint64_t histogram_total[4] = {};
  uint16_t raw_mean[4] = {};
};

class RkIspUserspaceController {
public:
  RkIspUserspaceController() = default;
  ~RkIspUserspaceController() { shutdown(); }

  bool init(const RkIspCamConfig &cfg);
  bool prepare();              // sysctl_prepare (all cams before stream on)
  bool start();                // sysctl_start (barrier-gated multi-cam)
  bool custom_ae_active() const { return custom_ae_enabled_; }
  bool get_ae_shadow_stats(RkIspAeShadowStats *stats);
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
  bool register_custom_ae();
  void unregister_custom_ae();
  static int32_t custom_ae_init(void *ctx);
  static int32_t custom_ae_run(void *ctx, const rk_aiq_customAe_stats_t *stats,
                               rk_aiq_customeAe_results_t *result);
  static int32_t custom_ae_exit(void *ctx);
  bool attach_direct_stats();
  void close_direct_stats();

  RkIspCamConfig cfg_{};
  rk_aiq_sys_ctx_s *aiq_ = nullptr;
  bool active_ = false;
  bool prepared_ = false;
  bool started_ = false;
  bool custom_ae_registered_ = false;
  bool custom_ae_enabled_ = false;
  int direct_stats_fd_ = -1;
  std::array<void *, 4> direct_stats_maps_{};
  std::array<size_t, 4> direct_stats_map_sizes_{};
  std::string calib_sensor_entity_;
  static int multi_cam_n_;
  static std::mutex custom_ae_mutex_;
  static std::map<rk_aiq_sys_ctx_s *, RkIspUserspaceController *> custom_ae_controllers_;
};
