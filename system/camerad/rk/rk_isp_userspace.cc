#include "system/camerad/rk/rk_isp_userspace.h"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <common/rkisp2-config.h>
#include <uAPI2/rk_aiq_user_api2_sysctl.h>
#include <uAPI2/rk_aiq_user_api2_imgproc.h>
#include <uAPI2/rk_aiq_user_api2_agamma.h>
#include <uAPI2/rk_aiq_user_api2_accm.h>
#include <uAPI2/rk_aiq_user_api2_awb.h>
#include <algos/accm/rk_aiq_types_accm_algo.h>
#include "system/camerad/rk/rk_tone_tables.h"

#include "common/swaglog.h"

namespace {

constexpr const char *kCalibRuntimeDir = "/tmp/camerad_calib";
constexpr const char *kCalibEmbedDir = "/data/openpilot/system/camerad/rk/calib_embed";

const char *kCalibNames[] = {
  "ox03c10_D2V10K_9419.json",
  "ox03c10_D2V11K_9420.json",
  "ox03c10_D2V12K_9421.json",
};

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

bool link_or_copy_calib(const std::string &src, const std::string &dst) {
  struct stat ss {};
  if (stat(src.c_str(), &ss) != 0) return false;

  struct stat ds {};
  if (lstat(dst.c_str(), &ds) == 0) {
    if (S_ISLNK(ds.st_mode)) {
      char target[512];
      const ssize_t n = readlink(dst.c_str(), target, sizeof(target) - 1);
      if (n > 0) {
        target[n] = '\0';
        if (src == target) return true;
      }
    } else if (S_ISREG(ds.st_mode) && ds.st_size == ss.st_size && ds.st_mtime >= ss.st_mtime) {
      return true;
    }
    unlink(dst.c_str());
  }

  if (symlink(src.c_str(), dst.c_str()) == 0) return true;

  FILE *in = fopen(src.c_str(), "rb");
  if (!in) return false;
  FILE *out = fopen(dst.c_str(), "wb");
  if (!out) {
    fclose(in);
    return false;
  }
  char buf[65536];
  size_t nread = 0;
  while ((nread = fread(buf, 1, sizeof(buf), in)) > 0) {
    if (fwrite(buf, 1, nread, out) != nread) {
      fclose(in);
      fclose(out);
      unlink(dst.c_str());
      return false;
    }
  }
  const bool ok = !ferror(in) && fflush(out) == 0;
  fclose(in);
  fclose(out);
  if (!ok) unlink(dst.c_str());
  return ok;
}

}  // namespace

static void apply_ccm(const rk_aiq_sys_ctx_t *aiq, int camera_num);

int RkIspUserspaceController::multi_cam_n_ = 1;

void RkIspUserspaceController::set_multi_cam_count(int n) {
  multi_cam_n_ = std::max(1, n);
  g_sync.expected = multi_cam_n_;
}

bool RkIspUserspaceController::ensure_runtime_calib() {
  mkdir(kCalibRuntimeDir, 0755);
  int ok = 0;
  for (const char *name : kCalibNames) {
    const std::string src = std::string(kCalibEmbedDir) + "/" + name;
    const std::string dst = std::string(kCalibRuntimeDir) + "/" + name;
    if (!link_or_copy_calib(src, dst)) {
      LOGE("RkIspUserspace: calib install failed %s -> %s", src.c_str(), dst.c_str());
      continue;
    }
    ok++;
  }
  if (ok < 3) {
    LOGE("RkIspUserspace: need 3 calib json under %s (from %s); got %d", kCalibRuntimeDir, kCalibEmbedDir, ok);
    return false;
  }
  return true;
}

void RkIspUserspaceController::stop_rkaiq() {
  std::system("sudo systemctl stop rkaiq_3A.service 2>/dev/null; "
              "sudo killall -9 rkaiq_3A_server 2>/dev/null; "
              "sudo killall -9 /usr/kommu/rkaiq_3A_server 2>/dev/null; "
              "sudo chmod 1777 /tmp 2>/dev/null; "
              "sudo rm -f /tmp/aiq*.lock /tmp/UNIX.domain* 2>/dev/null; "
              "touch /tmp/aiq0.lock /tmp/aiq1.lock /tmp/aiq2.lock 2>/dev/null; "
              "chmod 666 /tmp/aiq0.lock /tmp/aiq1.lock /tmp/aiq2.lock 2>/dev/null; "
              "sleep 1");
  if (std::system("pidof rkaiq_3A_server >/dev/null") == 0) {
    LOGE("RkIspUserspace: rkaiq_3A_server still alive after stop");
  }
  ensure_runtime_calib();
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
  if (!ensure_runtime_calib()) {
    return false;
  }
  const char *sns = rk_aiq_uapi2_sysctl_getBindedSnsEntNmByVd(cfg_.mainpath_dev.c_str());
  if (!sns || !sns[0]) {
    LOGE("RkIspUserspace cam%d: no sns for %s", cfg_.camera_num, cfg_.mainpath_dev.c_str());
    return false;
  }
  aiq_ = rk_aiq_uapi2_sysctl_init(sns, kCalibRuntimeDir, nullptr, nullptr);
  if (!aiq_) {
    LOGE("RkIspUserspace cam%d: sysctl_init failed (dir=%s)", cfg_.camera_num, kCalibRuntimeDir);
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
    return false;
  }
  params_fd_ = open(params_dev.c_str(), O_RDWR | O_CLOEXEC);
  if (params_fd_ < 0) {
    LOGE("RkIspUserspace cam%d: open %s failed errno=%d", cfg_.camera_num, params_dev.c_str(), errno);
    return false;
  }
  if (!subscribe_params_events()) {
    close_params_fd();
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

  rk_aiq_uapi2_setExpMode(aiq_, OP_MANUAL);
  // Keep rkaiq available for ISP statistics and parameter generation, but do
  // not let its SensorHw path compete with the direct-I2C exposure owner.
  if (rk_aiq_uapi2_setAeLock(aiq_, true) != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: AE lock failed; direct-I2C ownership is unsafe", cfg_.camera_num);
  }
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
  active_ = false;
  prepared_ = false;
}
