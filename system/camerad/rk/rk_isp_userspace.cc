#include "system/camerad/rk/rk_isp_userspace.h"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <common/rkisp2-config.h>
#include <uAPI2/rk_aiq_user_api2_sysctl.h>
#include <uAPI2/rk_aiq_user_api2_imgproc.h>
#include <uAPI2/rk_aiq_user_api2_agamma.h>
#include <uAPI2/rk_aiq_user_api2_accm.h>
#include <uAPI2/rk_aiq_user_api2_awb.h>
#include <algos/accm/rk_aiq_types_accm_algo.h>
#include "system/camerad/rk/ka2_isp_calibration.h"
#include "system/camerad/rk/rk_tone_tables.h"

#include "common/swaglog.h"

namespace {

std::string resolve_v4l_dev_by_name_index(const char *name, int index) {
  for (int i = 0; i < 64; i++) {
    char syspath[64], devname[128];
    snprintf(syspath, sizeof(syspath), "/sys/class/video4linux/video%d/name", i);
    FILE *f = fopen(syspath, "r");
    if (!f) continue;
    if (!fgets(devname, sizeof(devname), f)) {
      fclose(f);
      continue;
    }
    fclose(f);
    devname[strcspn(devname, "\n")] = 0;
    if (strcmp(devname, name) != 0) continue;
    if (index-- != 0) continue;
    char path[32];
    snprintf(path, sizeof(path), "/dev/video%d", i);
    return path;
  }
  return {};
}

struct RkCamSync {
  int expected = 1;
};

RkCamSync g_sync;

}  // namespace

static void apply_ccm(const rk_aiq_sys_ctx_t *aiq, int camera_num);

int RkIspUserspaceController::multi_cam_n_ = 1;

void RkIspUserspaceController::set_multi_cam_count(int n) {
  multi_cam_n_ = std::max(1, n);
  g_sync.expected = multi_cam_n_;
}

bool RkIspUserspaceController::calibration_available() {
  // Profiles are compiled into this binary; register_calibration validates the
  // selected profile at the actual camera boundary.
  return true;
}

bool RkIspUserspaceController::register_calibration(const char *sensor_entity) {
  const rk_aiq_ka2_calib_view_t *profile = ka2_calibration::profile(cfg_.camera_num);
  if (!profile) {
    LOGE("RkIspUserspace cam%d: invalid calibration index", cfg_.camera_num);
    return false;
  }

  const XCamReturn rc = rk_aiq_uapi2_sysctl_preInit_ka2_calib(sensor_entity, profile);
  if (rc != XCAM_RETURN_NO_ERROR) {
    LOGE("RkIspUserspace cam%d: typed calibration registration failed (%d)",
         cfg_.camera_num, static_cast<int>(rc));
    return false;
  }
  calib_sensor_entity_ = sensor_entity;
  return true;
}

void RkIspUserspaceController::clear_calibration_registration() {
  if (!calib_sensor_entity_.empty()) {
    rk_aiq_uapi2_sysctl_preInit(calib_sensor_entity_.c_str(), RK_AIQ_WORKING_MODE_NORMAL, nullptr);
    calib_sensor_entity_.clear();
  }
}

bool RkIspUserspaceController::set_external_exposure(uint32_t frame_id, int integration_lines,
                                                      float analog_gain,
                                                      uint32_t sensor_gain_code,
                                                      bool high_conversion_gain) {
  if (!active_ || !aiq_ || integration_lines <= 0 || analog_gain <= 0.0f || sensor_gain_code == 0) return false;
  const XCamReturn rc = rk_aiq_uapi2_sysctl_setExternalExposureInfoV2(
      aiq_, frame_id, analog_gain, sensor_gain_code, static_cast<uint32_t>(integration_lines),
      high_conversion_gain ? 1 : 0);
  if (rc != XCAM_RETURN_NO_ERROR && frame_id % 120 == 0) {
    LOGW("RkIspUserspace cam%d: external exposure metadata failed (%d)", cfg_.camera_num, (int)rc);
  }
  return rc == XCAM_RETURN_NO_ERROR;
}


bool RkIspUserspaceController::subscribe_params_events() {
  if (params_fd_ < 0) return false;
  struct v4l2_event_subscription sub = {};
  sub.type = CIFISP_V4L2_EVENT_STREAM_START;
  if (ioctl(params_fd_, VIDIOC_SUBSCRIBE_EVENT, &sub) < 0) {
    LOGE("RkIspUserspace cam%d: subscribe STREAM_START failed errno=%d", cfg_.camera_num, errno);
    return false;
  }
  return true;
}

void RkIspUserspaceController::close_params_fd() {
  if (params_fd_ >= 0) {
    close(params_fd_);
    params_fd_ = -1;
  }
}

bool RkIspUserspaceController::init(const RkIspCamConfig &cfg) {
  cfg_ = cfg;
  if (cfg_.mainpath_dev.empty()) {
    LOGE("RkIspUserspace cam%d: mainpath_dev required", cfg_.camera_num);
    return false;
  }
  const char *sns = rk_aiq_uapi2_sysctl_getBindedSnsEntNmByVd(cfg_.mainpath_dev.c_str());
  if (!sns || !sns[0]) {
    LOGE("RkIspUserspace cam%d: no sns for %s", cfg_.camera_num, cfg_.mainpath_dev.c_str());
    return false;
  }
  if (!register_calibration(sns)) {
    return false;
  }
  aiq_ = rk_aiq_uapi2_sysctl_init(sns, nullptr, nullptr, nullptr);
  if (!aiq_) {
    LOGE("RkIspUserspace cam%d: sysctl_init failed with typed calibration", cfg_.camera_num);
    clear_calibration_registration();
    return false;
  }
  rk_aiq_uapi2_sysctl_setListenStrmStatus(aiq_, false);
  if (multi_cam_n_ > 1) {
    rk_aiq_uapi2_sysctl_setMulCamConc(aiq_, true);
  }
  // Seed CCM before sysctl_prepare generates the initial ISP parameters.
  apply_ccm(aiq_, cfg.camera_num);

  const std::string params_dev = resolve_v4l_dev_by_name_index("rkisp-input-params", cfg_.camera_num);
  if (params_dev.empty()) {
    LOGE("RkIspUserspace cam%d: rkisp-input-params not found", cfg_.camera_num);
    shutdown();
    return false;
  }
  params_fd_ = open(params_dev.c_str(), O_RDWR | O_CLOEXEC);
  if (params_fd_ < 0) {
    LOGE("RkIspUserspace cam%d: open %s failed errno=%d", cfg_.camera_num, params_dev.c_str(), errno);
    shutdown();
    return false;
  }
  if (!subscribe_params_events()) {
    shutdown();
    return false;
  }

  active_ = true;
  return true;
}

static void disable_isp_awb(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  if (rk_aiq_uapi2_sysctl_setModuleCtl(aiq, RK_MODULE_AWB_GAIN, false) != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: failed to disable RK_MODULE_AWB_GAIN", camera_num);
  }
  if (rk_aiq_uapi2_sysctl_setModuleCtl(aiq, RK_MODULE_AWB, false) != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: failed to disable RK_MODULE_AWB", camera_num);
  }

  rk_aiq_uapiV2_wbV21_attrib_t awb = {};
  awb.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  awb.byPass = true;
  awb.mode = RK_AIQ_WB_MODE_MANUAL;
  awb.stManual.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  awb.stManual.mode = RK_AIQ_MWB_MODE_WBGAIN;
  awb.stManual.para.gain = {1.0f, 1.0f, 1.0f, 1.0f};
  if (rk_aiq_user_api2_awbV21_SetAllAttrib(aiq, awb) != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: awbV21 bypass failed", camera_num);
  }

  bool awb_gain_enabled = true;
  if (rk_aiq_uapi2_sysctl_getModuleCtl(aiq, RK_MODULE_AWB_GAIN, &awb_gain_enabled) != XCAM_RETURN_NO_ERROR || awb_gain_enabled) {
    LOGW("RkIspUserspace cam%d: RKISP AWB gain remains enabled", camera_num);
  }
  bool awb_stats_enabled = true;
  if (rk_aiq_uapi2_sysctl_getModuleCtl(aiq, RK_MODULE_AWB, &awb_stats_enabled) != XCAM_RETURN_NO_ERROR || awb_stats_enabled) {
    LOGW("RkIspUserspace cam%d: RKISP AWB stats remain enabled", camera_num);
  }
}

static bool disable_isp_nr_and_blc(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  static constexpr rk_aiq_module_id_t kDisabledModules[] = {
    RK_MODULE_NR,
    RK_MODULE_TNR,
    RK_MODULE_RAWNR,
    RK_MODULE_BLS,
  };
  static constexpr const char *kModuleNames[] = {"NR", "TNR", "RAWNR", "BLS"};

  bool ok = true;
  bool module_off[sizeof(kDisabledModules) / sizeof(kDisabledModules[0])] = {};
  for (size_t i = 0; i < sizeof(kDisabledModules) / sizeof(kDisabledModules[0]); i++) {
    if (rk_aiq_uapi2_sysctl_setModuleCtl(aiq, kDisabledModules[i], false) != XCAM_RETURN_NO_ERROR) {
      LOGE("RkIspUserspace cam%d: failed to disable RK_MODULE_%s", camera_num, kModuleNames[i]);
      ok = false;
    }
  }

  bool module_enabled = true;
  for (size_t i = 0; i < sizeof(kDisabledModules) / sizeof(kDisabledModules[0]); i++) {
    module_enabled = true;
    const int rc = rk_aiq_uapi2_sysctl_getModuleCtl(aiq, kDisabledModules[i], &module_enabled);
    if (rc != XCAM_RETURN_NO_ERROR || module_enabled) {
      LOGE("RkIspUserspace cam%d: RK_MODULE_%s readback is %s (rc=%d)",
           camera_num, kModuleNames[i], module_enabled ? "enabled" : "unavailable", (int)rc);
      ok = false;
    } else {
      module_off[i] = true;
    }
  }

  LOG("RkIspUserspace cam%d: RKAIQ shutdown NR=%s TNR=%s RAWNR=%s BLS=%s",
       camera_num,
       module_off[0] ? "off" : "check-failed", module_off[1] ? "off" : "check-failed",
       module_off[2] ? "off" : "check-failed", module_off[3] ? "off" : "check-failed");
  return ok;
}

static void apply_ccm(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  rk_aiq_ccm_attrib_t ccm = {};
  ccm.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  ccm.byPass = false;
  ccm.mode = RK_AIQ_CCM_MODE_MANUAL;
  auto s12 = [](uint32_t v) -> float {
    int x = static_cast<int>(v & 0xfff);
    if (x >= 0x800) x -= 0x1000;
    return static_cast<float>(x) / 128.0f;
  };
  for (int i = 0; i < 9; i++) ccm.stManual.Matrix.ccMatrix[i] = s12(rk_tone::kOx03c10CcmQ[i]);
  for (int i = 0; i < 17; i++) ccm.stManual.YAlp.y_alpha_curve[i] = 1024.f;
  ccm.stManual.YAlp.low_bound_pos_bit = 8.f;
  if (rk_aiq_user_api2_accm_SetAttrib(aiq, &ccm) != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: CCM SetAttrib failed", camera_num);
  }
}

static void apply_comma_gamma(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  rk_aiq_gamma_v11_attr_t gam = {};
  gam.sync.done = false;
  gam.mode = RK_AIQ_GAMMA_MODE_MANUAL;
  gam.stManual.Gamma_en = true;
  gam.stManual.Gamma_out_offset = 0;
  for (int i = 0; i < rk_tone::kGammaKnots; i++) {
    gam.stManual.Gamma_curve[i] = rk_tone::kOx03c10GammaLinearV11[i];
  }
  if (rk_aiq_user_api2_agamma_v11_SetAttrib(aiq, &gam) != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: agamma SetAttrib failed", camera_num);
  }
}

bool RkIspUserspaceController::prepare() {
  if (!active_ || !aiq_ || prepared_) return active_ && prepared_;
  XCamReturn r = rk_aiq_uapi2_sysctl_prepare(aiq_, 1920, 1200, RK_AIQ_WORKING_MODE_NORMAL);
  if (r != XCAM_RETURN_NO_ERROR) {
    LOGE("RkIspUserspace cam%d: prepare failed %d", cfg_.camera_num, (int)r);
    return false;
  }

  // RKAIQ AE is disabled in the calibration. camerad is the sole AE owner and
  // writes the sensor exposure/gain directly over I2C.
  if (!disable_isp_nr_and_blc(aiq_, cfg_.camera_num)) {
    LOGE("RkIspUserspace cam%d: refusing to continue with RKAIQ NR/BLC state unverified", cfg_.camera_num);
    return false;
  }

  apply_comma_gamma(aiq_, cfg_.camera_num);
  prepared_ = true;
  return true;
}

bool RkIspUserspaceController::start() {
  if (!active_ || !aiq_ || !prepared_ || started_) return active_ && started_;

  XCamReturn r = rk_aiq_uapi2_sysctl_start(aiq_);
  if (r != XCAM_RETURN_NO_ERROR) {
    LOGE("RkIspUserspace cam%d: start failed %d", cfg_.camera_num, (int)r);
    return false;
  }
  started_ = true;

  disable_isp_awb(aiq_, cfg_.camera_num);

  return true;
}

void RkIspUserspaceController::shutdown() {
  if (aiq_) {
    if (started_) {
      rk_aiq_uapi2_sysctl_stop(aiq_, false);
      started_ = false;
    }
    rk_aiq_uapi2_sysctl_deinit(aiq_);
    aiq_ = nullptr;
  }
  close_params_fd();
  clear_calibration_registration();
  active_ = false;
  prepared_ = false;
}
