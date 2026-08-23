#include "system/camerad/rk/rk_isp_userspace.h"

#include <atomic>
#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <dlfcn.h>

#include <uAPI2/rk_aiq_user_api2_sysctl.h>
#include <uAPI2/rk_aiq_user_api2_imgproc.h>
#include <uAPI2/rk_aiq_user_api2_agamma.h>
#include <uAPI2/rk_aiq_user_api2_ablc.h>
#include <uAPI2/rk_aiq_user_api2_accm.h>
#include <uAPI2/rk_aiq_user_api2_awb.h>
#include <uAPI2/rk_aiq_user_api2_acp.h>
#include <uAPI2/rk_aiq_user_api2_adebayer.h>
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

static constexpr float kKa2WbR = 1.55f;
static constexpr float kKa2WbG = 1.00f;
static constexpr float kKa2WbB = 1.88f;

}  // namespace

int RkIspUserspaceController::multi_cam_n_ = 1;

void RkIspUserspaceController::set_multi_cam_count(int n) {
  multi_cam_n_ = std::max(1, n);
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
  LOGD("RkIspUserspace: stopping rkaiq_3A daemon");
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
  } else {
  }
  ensure_runtime_calib();
}

void RkIspUserspaceController::start_rkaiq() {
  LOGD("RkIspUserspace: starting rkaiq_3A");
  if (std::system("sudo systemctl start rkaiq_3A.service") != 0) {
    std::system("sudo /usr/kommu/rkaiq_3A_server >/dev/null 2>&1 &");
  }
  usleep(2000000);
}

void RkIspUserspaceController::apply_mwb(float r, float g, float b) {
  if (!aiq_) return;
  wb_last_r_ = r;
  wb_last_g_ = g;
  wb_last_b_ = b;
  rk_aiq_uapiV2_wbV21_attrib_t awb = {};
  awb.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  awb.byPass = false;
  awb.mode = RK_AIQ_WB_MODE_MANUAL;
  awb.stManual.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  awb.stManual.mode = RK_AIQ_MWB_MODE_WBGAIN;
  awb.stManual.para.gain = {r, g, g, b};
  rk_aiq_user_api2_awbV21_SetAllAttrib(aiq_, awb);
  float pre[4] = {r, g, g, b};
  rk_aiq_user_api2_awb_setAwbPreWbgain(aiq_, pre);
  rk_aiq_uapi2_setMWBGain(aiq_, &awb.stManual.para.gain);
  rk_aiq_uapi2_setWBMode(aiq_, OP_MANUAL);
}

void RkIspUserspaceController::ensure_sdg_hook() {
  static std::atomic<bool> install_done{false};
  static void *hook_handle = nullptr;
  if (install_done.load(std::memory_order_acquire)) return;

  dlopen("librkaiq.so", RTLD_NOW | RTLD_NOLOAD);

  char exe[4096];
  const ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
  if (n <= 0) {
    LOGW("RkIspUserspace: SDG hook path resolve failed");
    return;
  }
  exe[n] = '\0';
  std::string hook_path(exe);
  const auto slash = hook_path.rfind('/');
  if (slash == std::string::npos) return;
  hook_path = hook_path.substr(0, slash) + "/rk/librk_sdg_hook.so";

  using InstallFn = int (*)();
  InstallFn install = nullptr;

  for (int attempt = 0; attempt < 8; attempt++) {
    if (hook_handle) {
      dlclose(hook_handle);
      hook_handle = nullptr;
    }
    hook_handle = dlopen(hook_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!hook_handle) {
      LOGW("RkIspUserspace: SDG hook dlopen(%s) failed: %s", hook_path.c_str(), dlerror());
      return;
    }
    install = reinterpret_cast<InstallFn>(dlsym(hook_handle, "install_librkaiq_sdg_hook"));
    if (!install) {
      LOGW("RkIspUserspace: SDG hook install symbol missing");
      return;
    }
    const int patched = install();
    if (patched > 0) {
      install_done.store(true, std::memory_order_release);
      LOG("RkIspUserspace: SDG hook ok (%d base(s))", patched);
      return;
    }
  }
  LOGW("RkIspUserspace: SDG hook patch failed (no librkaiq base patched)");
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
  if (multi_cam_n_ > 1) {
    rk_aiq_uapi2_sysctl_setMulCamConc(aiq_, true);
  }
  active_ = true;
  return true;
}

static void apply_daylight_wb_ccm(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  const float wr = kKa2WbR, wg = kKa2WbG, wb = kKa2WbB;

  rk_aiq_uapi2_sysctl_setModuleCtl(aiq, RK_MODULE_AWB_GAIN, true);

  rk_aiq_uapiV2_wbV21_attrib_t awb = {};
  awb.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  awb.byPass = false;
  awb.mode = RK_AIQ_WB_MODE_MANUAL;
  awb.stManual.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  awb.stManual.mode = RK_AIQ_MWB_MODE_WBGAIN;
  awb.stManual.para.gain = {wr, wg, wg, wb};
  if (rk_aiq_user_api2_awbV21_SetAllAttrib(aiq, awb) != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: awbV21_SetAllAttrib failed", camera_num);
  }
  rk_aiq_uapiV2_wb_opMode_t op = {};
  op.mode = RK_AIQ_WB_MODE_MANUAL;
  rk_aiq_user_api2_awb_SetWpModeAttrib(aiq, op);
  rk_aiq_wb_mwb_attrib_t mwb = awb.stManual;
  rk_aiq_user_api2_awb_SetMwbAttrib(aiq, mwb);
  rk_aiq_wb_gain_t g = mwb.para.gain;
  rk_aiq_uapi2_setMWBGain(aiq, &g);
  rk_aiq_uapi2_setWBMode(aiq, OP_MANUAL);
  float pre[4] = {wr, wg, wg, wb};
  rk_aiq_user_api2_awb_setAwbPreWbgain(aiq, pre);

  auto s12 = [](uint32_t v) -> float {
    int x = static_cast<int>(v & 0xfff);
    if (x >= 0x800) x -= 0x1000;
    return static_cast<float>(x) / 128.0f;
  };
  rk_aiq_ccm_attrib_t ccm = {};
  ccm.byPass = false;
  ccm.mode = RK_AIQ_CCM_MODE_MANUAL;
  for (int i = 0; i < 9; i++) ccm.stManual.Matrix.ccMatrix[i] = s12(rk_tone::kOx03c10CcmQ[i]);
  for (int i = 0; i < 17; i++) ccm.stManual.YAlp.y_alpha_curve[i] = 1024.f;
  ccm.stManual.YAlp.low_bound_pos_bit = 8.f;
  if (rk_aiq_user_api2_accm_SetAttrib(aiq, &ccm) != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: CCM SetAttrib failed", camera_num);
  }
}

static void apply_road_chroma_guard(const rk_aiq_sys_ctx_t *aiq) {
  // Road calib disables NR/LSC/dehaze; keep chroma via get-modify-set (slice 3 lesson).
  acp_attrib_t acp = {};
  acp.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  rk_aiq_user_api2_acp_GetAttrib(aiq, &acp);
  acp.brightness = 128;
  acp.contrast = 128;
  acp.saturation = 128;
  acp.hue = 128;
  rk_aiq_user_api2_acp_SetAttrib(aiq, &acp);

  adebayer_attrib_t db = {};
  db.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  rk_aiq_user_api2_adebayer_GetAttrib(aiq, &db);
  db.mode = RK_AIQ_DEBAYER_MODE_MANUAL;
  db.stManual.cnr_strength = 0;
  rk_aiq_user_api2_adebayer_SetAttrib(aiq, db);
}

static void apply_comma_gamma(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  // PWL decompand is owned by the SDG librkaiq hook; set ox03c10 gamma only.
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

bool RkIspUserspaceController::prepare_and_start() {
  if (!active_ || !aiq_ || started_) return active_ && started_;
  ensure_sdg_hook();
  XCamReturn r = rk_aiq_uapi2_sysctl_prepare(aiq_, 1920, 1200, RK_AIQ_WORKING_MODE_NORMAL);
  if (r != XCAM_RETURN_NO_ERROR) {
    LOGE("RkIspUserspace cam%d: prepare failed %d", cfg_.camera_num, (int)r);
    return false;
  }

  rk_aiq_uapi2_setExpMode(aiq_, OP_MANUAL);
  rk_aiq_blc_attrib_t blc = {};
  blc.eMode = ABLC_OP_MODE_MANUAL;
  blc.stBlc0Manual.enable = true;
  blc.stBlc0Manual.blc_r = 0;
  blc.stBlc0Manual.blc_gr = 0;
  blc.stBlc0Manual.blc_gb = 0;
  blc.stBlc0Manual.blc_b = 0;
  blc.stBlc1Manual.enable = false;
  if (rk_aiq_user_api2_ablc_SetAttrib(aiq_, &blc) != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: ablc_SetAttrib failed", cfg_.camera_num);
  }

  apply_comma_gamma(aiq_, cfg_.camera_num);

  r = rk_aiq_uapi2_sysctl_start(aiq_);
  if (r != XCAM_RETURN_NO_ERROR) {
    LOGE("RkIspUserspace cam%d: start failed %d", cfg_.camera_num, (int)r);
    return false;
  }
  started_ = true;

  apply_daylight_wb_ccm(aiq_, cfg_.camera_num);
  if (cfg_.camera_num == 1) apply_road_chroma_guard(aiq_);

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
  active_ = false;
}
