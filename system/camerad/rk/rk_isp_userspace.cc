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
#include <time.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <common/rkisp2-config.h>
#include <uAPI2/rk_aiq_user_api2_sysctl.h>
#include <uAPI2/rk_aiq_user_api2_imgproc.h>
#include <uAPI2/rk_aiq_user_api2_agamma.h>
#include <uAPI2/rk_aiq_user_api2_adegamma.h>
#include <uAPI2/rk_aiq_user_api2_acnr_v31.h>
#include <uAPI2/rk_aiq_user_api2_ablc.h>
#include <uAPI2/rk_aiq_user_api2_accm.h>
#include <uAPI2/rk_aiq_user_api2_awb.h>
#include <uAPI2/rk_aiq_user_api2_acp.h>
#include <uAPI2/rk_aiq_user_api2_adebayer.h>
#include <uAPI2/rk_aiq_user_api2_alsc.h>
#include <algos/accm/rk_aiq_types_accm_algo.h>
#include "system/camerad/rk/rk_tone_tables.h"
#include "system/camerad/rk/rk_agent_debug.h"

#include "common/swaglog.h"

// Not exported in device rkaiq headers; present in librkaiq.so (running camerad uses it).
extern "C" XCamReturn rk_aiq_uapi2_sysctl_setExternalExposureInfo(const rk_aiq_sys_ctx_t *ctx,
                                                                  uint32_t frame_id,
                                                                  rk_aiq_frame_info_t *info);

namespace {

constexpr const char *kCalibRuntimeDir = "/tmp/camerad_calib";
constexpr const char *kCalibEmbedDir = "/data/openpilot/system/camerad/rk/calib_embed";

const char *kCalibNames[] = {
  "ox03c10_D2V10K_9419.json",
  "ox03c10_D2V11K_9420.json",
  "ox03c10_D2V12K_9421.json",
};

// Calibrated ColorChecker WB from HEAD. Unity WB is available only as an
// explicit diagnostic override through Ka2UnityWb=1.
static constexpr float kKa2WbR = 1.90f;
static constexpr float kKa2WbG = 1.00f;
static constexpr float kKa2WbB = 1.68f;

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

bool RkIspUserspaceController::set_external_exposure(uint32_t frame_id, int integration_lines,
                                                      float analog_gain,
                                                      bool high_conversion_gain) {
  if (!active_ || !aiq_ || integration_lines <= 0 || analog_gain <= 0.0f) return false;
  (void)high_conversion_gain;  // rk_aiq_frame_info_t has no HCG field.

  rk_aiq_frame_info_t info = {};
  info.frame_id = frame_id;
  info.normal_exp = static_cast<float>(integration_lines);
  info.normal_gain = analog_gain;
  info.normal_exp_reg = static_cast<uint32_t>(integration_lines);
  // OX03C10 uses 0x100 at 1x analog gain.  This field is the sensor code,
  // not the ISP gain or a 16x fixed-point representation.
  info.normal_gain_reg = static_cast<uint32_t>(std::lround(analog_gain * 256.0f));
  info.hdr_exp_l = info.hdr_exp_m = info.hdr_exp_s = info.normal_exp;
  info.hdr_gain_l = info.hdr_gain_m = info.hdr_gain_s = info.normal_gain;
  info.hdr_exp_l_reg = info.hdr_exp_m_reg = info.hdr_exp_s_reg = info.normal_exp_reg;
  info.hdr_gain_l_reg = info.hdr_gain_m_reg = info.hdr_gain_s_reg = info.normal_gain_reg;

  const XCamReturn rc = rk_aiq_uapi2_sysctl_setExternalExposureInfo(aiq_, frame_id, &info);
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

static void apply_daylight_wb_ccm(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  const char *unity_env = getenv("Ka2UnityWb");
  const bool unity_wb = (unity_env && unity_env[0] == '1' && unity_env[1] == '\0');
  const float wr = unity_wb ? 1.00f : kKa2WbR;
  const float wg = unity_wb ? 1.00f : kKa2WbG;
  const float wb = unity_wb ? 1.00f : kKa2WbB;

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
  char wbuf[80];
  snprintf(wbuf, sizeof(wbuf), "{\"cam\":%d,\"unity\":%s,\"r\":%.2f,\"b\":%.2f}",
           camera_num, unity_wb ? "true" : "false", wr, wb);
  rk_agent_debug_log("H7", "rk_isp_userspace.cc:apply_daylight_wb_ccm", "wb gains", wbuf);
}

static void apply_chroma_nr(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  const char *cnr_env = getenv("Ka2CnrPercent");
  if (!cnr_env || cnr_env[0] == '\0') return;
  const float pct = std::clamp(static_cast<float>(atof(cnr_env)), 0.f, 100.f);
  rk_aiq_cnr_strength_v31_t str = {};
  str.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  str.strength_enable = true;
  str.percent = pct;
  const XCamReturn rs = rk_aiq_user_api2_acnrV31_SetStrength(aiq, &str);
  char buf[80];
  snprintf(buf, sizeof(buf), "{\"cam\":%d,\"percent\":%.1f,\"set_rc\":%d}", camera_num, pct, (int)rs);
  rk_agent_debug_log("H8", "rk_isp_userspace.cc:apply_chroma_nr", "acnrV31 strength", buf);
}

static void log_debayer_manual(const rk_aiq_sys_ctx_t *aiq, int camera_num, const char *stage,
                               const adebayer_attrib_t &db, int set_rc) {
  const auto &m = db.stManual;
  char buf[1024];
  snprintf(buf, sizeof(buf),
           "{\"cam\":%d,\"stage\":\"%s\",\"enable\":%u,\"mode\":%d,\"set_rc\":%d,"
           "\"filter1\":[%d,%d,%d,%d,%d],\"filter2\":[%d,%d,%d,%d,%d],"
           "\"gain_offset\":%u,\"sharp_strength\":%u,\"hf_offset\":%u,\"offset\":%u,"
           "\"clip_en\":%u,\"filter_g_en\":%u,\"filter_c_en\":%u,"
           "\"thed0\":%u,\"thed1\":%u,\"dist_scale\":%u,\"cnr_strength\":%u,\"shift_num\":%u}",
           camera_num, stage, db.enable, (int)db.mode, set_rc,
           m.filter1[0], m.filter1[1], m.filter1[2], m.filter1[3], m.filter1[4],
           m.filter2[0], m.filter2[1], m.filter2[2], m.filter2[3], m.filter2[4],
           m.gain_offset, m.sharp_strength, m.hf_offset, m.offset,
           m.clip_en, m.filter_g_en, m.filter_c_en,
           m.thed0, m.thed1, m.dist_scale, m.cnr_strength, m.shift_num);
  rk_agent_debug_log("H9", "rk_isp_userspace.cc:log_debayer_manual", "debayer manual attrs", buf);
}

static bool env_u8(const char *name, uint8_t *out, int lo, int hi) {
  const char *e = getenv(name);
  if (!e || e[0] == '\0') return false;
  *out = static_cast<uint8_t>(std::clamp(atoi(e), lo, hi));
  return true;
}

static void apply_debayer_tune(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  const char *enable_override = getenv("Ka2RoadDebayerOverride");
  if (camera_num != 1 || !enable_override || strcmp(enable_override, "1") != 0) return;


  const char *cnr_env = getenv("Ka2DebayerCnr");
  int cnr = 0;
  if (cnr_env && cnr_env[0] != '\0') {
    cnr = std::clamp(atoi(cnr_env), 0, 9);
  }

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
  log_debayer_manual(aiq, camera_num, "pre_override", db, 0);
  db.mode = RK_AIQ_DEBAYER_MODE_MANUAL;
  db.stManual.cnr_strength = static_cast<uint8_t>(cnr);
  uint8_t v = 0;
  if (env_u8("Ka2DebayerThed0", &v, 0, 15)) db.stManual.thed0 = v;
  if (env_u8("Ka2DebayerThed1", &v, 0, 15)) db.stManual.thed1 = v;
  if (env_u8("Ka2DebayerDistScale", &v, 0, 15)) db.stManual.dist_scale = v;
  const XCamReturn rs = rk_aiq_user_api2_adebayer_SetAttrib(aiq, db);
  log_debayer_manual(aiq, camera_num, "post_override", db, (int)rs);
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"cam\":%d,\"cnr\":%d,\"thed0\":%u,\"thed1\":%u,\"dist_scale\":%u,\"set_rc\":%d}",
           camera_num, cnr, db.stManual.thed0, db.stManual.thed1, db.stManual.dist_scale, (int)rs);
  rk_agent_debug_log("H6", "rk_isp_userspace.cc:apply_debayer_tune", "debayer cnr", buf);
}


static void apply_lsc_off(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  rk_aiq_lsc_attrib_t attr = {};
  rk_aiq_user_api2_alsc_GetAttrib(aiq, &attr);
  attr.byPass = true;
  attr.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  const XCamReturn rs = rk_aiq_user_api2_alsc_SetAttrib(aiq, attr);
  rk_aiq_lsc_querry_info_t qi = {};
  rk_aiq_user_api2_alsc_QueryLscInfo(aiq, &qi);
  LOGW("RkIspUserspace cam%d: LSC OFF byPass set=%d en=%d", camera_num, (int)rs, qi.lsc_en ? 1 : 0);
}


static int gamma_knot_index_for_x(int x) {
  for (int i = 0; i < rk_tone::kGammaKnots; i++) {
    if (rk_tone::kGammaX[i] == x) return i;
  }
  return -1;
}

static void log_gamma_state(const rk_aiq_sys_ctx_t *aiq, int camera_num, const char *stage,
                            bool use_composed, const uint16_t *sent_curve) {
  rk_aiq_gamma_v11_attr_t got = {};
  const int gr = rk_aiq_user_api2_agamma_v11_GetAttrib(aiq, &got);
  const int i128 = gamma_knot_index_for_x(128);
  const int i512 = gamma_knot_index_for_x(512);
  const int i1024 = gamma_knot_index_for_x(1024);
  const int i2048 = gamma_knot_index_for_x(2048);
  char buf[640];
  snprintf(buf, sizeof(buf),
           "{\"cam\":%d,\"stage\":\"%s\",\"use_composed\":%s,\"set_rc\":%d,\"get_rc\":%d,"
           "\"sent_y128\":%u,\"sent_y512\":%u,\"sent_y1024\":%u,\"sent_y2048\":%u,"
           "\"got_y128\":%u,\"got_y512\":%u,\"got_y1024\":%u,\"got_y2048\":%u,"
           "\"linear_y128\":%u,\"linear_y512\":%u,\"composed_y128\":%u,\"composed_y1024\":%u}",
           camera_num, stage, use_composed ? "true" : "false", 0, gr,
           i128 >= 0 ? sent_curve[i128] : 0u, i512 >= 0 ? sent_curve[i512] : 0u,
           i1024 >= 0 ? sent_curve[i1024] : 0u, i2048 >= 0 ? sent_curve[i2048] : 0u,
           (gr == 0 && i128 >= 0) ? got.stManual.Gamma_curve[i128] : 0u,
           (gr == 0 && i512 >= 0) ? got.stManual.Gamma_curve[i512] : 0u,
           (gr == 0 && i1024 >= 0) ? got.stManual.Gamma_curve[i1024] : 0u,
           (gr == 0 && i2048 >= 0) ? got.stManual.Gamma_curve[i2048] : 0u,
           i128 >= 0 ? rk_tone::kOx03c10GammaLinearV11[i128] : 0u,
           i512 >= 0 ? rk_tone::kOx03c10GammaLinearV11[i512] : 0u,
           i128 >= 0 ? rk_tone::kOx03c10GammaCommaV11[i128] : 0u,
           i1024 >= 0 ? rk_tone::kOx03c10GammaCommaV11[i1024] : 0u);
  rk_agent_debug_log("H2", "rk_isp_userspace.cc:log_gamma_state", "agamma curve state", buf);
}

static void apply_comma_degamma(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  rk_aiq_degamma_attrib_t dg = {};
  dg.mode = RK_AIQ_DEGAMMA_MODE_MANUAL;
  dg.stManual.en = true;
  for (int i = 0; i < rk_tone::kDegammaKnots; i++) {
    dg.stManual.X_axis[i] = rk_tone::kDegammaX[i];
    dg.stManual.curve_R[i] = rk_tone::kOx03c10DegammaY[i];
    dg.stManual.curve_G[i] = rk_tone::kOx03c10DegammaY[i];
    dg.stManual.curve_B[i] = rk_tone::kOx03c10DegammaY[i];
  }
  const XCamReturn set_rc = rk_aiq_user_api2_adegamma_SetAttrib(aiq, dg);
  char buf[128];
  snprintf(buf, sizeof(buf), "{\"cam\":%d,\"set_rc\":%d,\"y512\":%d}", camera_num, (int)set_rc,
           rk_tone::kOx03c10DegammaY[2]);
  rk_agent_debug_log("H5", "rk_isp_userspace.cc:apply_comma_degamma", "adegamma applied", buf);
}

static void apply_comma_gamma(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  // Comma Spectra: PWL12 linearization then gamma. Composed LUT via agamma alone crushes
  // midtones (y=0 for input <1024) — use split adegamma + gamma-only when Ka2CommaTone=1.
  const char *tone_env = getenv("Ka2CommaTone");
  const bool use_comma_tone = !(tone_env && tone_env[0] == '0' && tone_env[1] == '\0');
  if (use_comma_tone) {
    apply_comma_degamma(aiq, camera_num);
  }
  const uint16_t *curve = rk_tone::kOx03c10GammaLinearV11;

  rk_aiq_gamma_v11_attr_t gam = {};
  gam.sync.done = false;
  gam.mode = RK_AIQ_GAMMA_MODE_MANUAL;
  gam.stManual.Gamma_en = true;
  gam.stManual.Gamma_out_offset = 0;
  for (int i = 0; i < rk_tone::kGammaKnots; i++) {
    gam.stManual.Gamma_curve[i] = curve[i];
  }
  const XCamReturn set_rc = rk_aiq_user_api2_agamma_v11_SetAttrib(aiq, &gam);
  if (set_rc != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: agamma SetAttrib failed (%s)", camera_num,
         use_comma_tone ? "comma split" : "gamma-only");
  } else {
    LOGW("RkIspUserspace cam%d: agamma %s", camera_num,
         use_comma_tone ? "gamma-only (comma split path)" : "gamma-only");
  }
  char sel[128];
  snprintf(sel, sizeof(sel), "{\"cam\":%d,\"use_comma_tone\":%s,\"mode\":\"%s\",\"set_rc\":%d}",
           camera_num, use_comma_tone ? "true" : "false",
           use_comma_tone ? "split_degamma_gamma" : "gamma_only", (int)set_rc);
  rk_agent_debug_log("H1", "rk_isp_userspace.cc:apply_comma_gamma", "tone curve selected", sel);
  log_gamma_state(aiq, camera_num, "after_set", use_comma_tone, curve);
}

bool RkIspUserspaceController::prepare() {
  if (!active_ || !aiq_ || prepared_) return active_ && prepared_;
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
  prepared_ = true;
  return true;
}

bool RkIspUserspaceController::wait_isp_stream_start() {
  if (!active_ || params_fd_ < 0) return false;
  struct v4l2_event event = {};
  for (;;) {
    if (ioctl(params_fd_, VIDIOC_DQEVENT, &event) == 0 && event.type == CIFISP_V4L2_EVENT_STREAM_START) {
      struct timespec ts = {};
      clock_gettime(CLOCK_MONOTONIC, &ts);
      LOG("RkIspUserspace cam%d: ISP STREAM_START at %ld.%09ld", cfg_.camera_num, ts.tv_sec, ts.tv_nsec);
      return true;
    }
    if (errno != EAGAIN && errno != EINTR) {
      LOGE("RkIspUserspace cam%d: DQEVENT failed errno=%d", cfg_.camera_num, errno);
      return false;
    }
    usleep(1000);
  }
}

bool RkIspUserspaceController::start() {
  if (!active_ || !aiq_ || !prepared_ || started_) return active_ && started_;

  struct timespec ts = {};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  LOG("RkIspUserspace cam%d: sysctl_start at %ld.%09ld", cfg_.camera_num, ts.tv_sec, ts.tv_nsec);

  XCamReturn r = rk_aiq_uapi2_sysctl_start(aiq_);
  if (r != XCAM_RETURN_NO_ERROR) {
    LOGE("RkIspUserspace cam%d: start failed %d", cfg_.camera_num, (int)r);
    return false;
  }
  started_ = true;

  apply_daylight_wb_ccm(aiq_, cfg_.camera_num);
  apply_chroma_nr(aiq_, cfg_.camera_num);
  apply_debayer_tune(aiq_, cfg_.camera_num);
  apply_lsc_off(aiq_, cfg_.camera_num);
  {
    const char *tone_env = getenv("Ka2CommaTone");
    const bool use_comma_tone = !(tone_env && tone_env[0] == '0' && tone_env[1] == '\0');
    const uint16_t *curve = rk_tone::kOx03c10GammaLinearV11;
    log_gamma_state(aiq_, cfg_.camera_num, "after_start_wb", use_comma_tone, curve);
  }

  return true;
}

bool RkIspUserspaceController::prepare_and_start() {
  if (!prepare()) return false;
  return start();
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
