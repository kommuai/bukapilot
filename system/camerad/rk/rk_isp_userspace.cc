#include "system/camerad/rk/rk_isp_userspace.h"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <uAPI2/rk_aiq_user_api2_sysctl.h>
#include <uAPI2/rk_aiq_user_api2_imgproc.h>
#include <uAPI2/rk_aiq_user_api2_adehaze.h>
#include <uAPI2/rk_aiq_user_api2_agamma.h>
#include <uAPI2/rk_aiq_user_api2_adegamma.h>
#include <uAPI2/rk_aiq_user_api2_ablc.h>
#include <uAPI2/rk_aiq_user_api2_accm.h>
#include <uAPI2/rk_aiq_user_api2_awb.h>
#include <uAPI2/rk_aiq_user_api2_alsc.h>
#include <algos/accm/rk_aiq_types_accm_algo.h>
#include "system/camerad/rk/rk_tone_tables.h"

#include "common/swaglog.h"

namespace {

// Camerad-owned calib (Spectra-style). Never read /etc/iqfiles/ox03c10*.
constexpr const char *kCalibRuntimeDir = "/tmp/camerad_calib";
constexpr const char *kCalibEmbedDir = "/data/openpilot/system/camerad/rk/calib_embed";

const char *kCalibNames[] = {
  "ox03c10_D2V10K_9419.json",
  "ox03c10_D2V11K_9420.json",
  "ox03c10_D2V12K_9421.json",
};

bool copy_file(const std::string &src, const std::string &dst) {
  std::ifstream in(src, std::ios::binary);
  if (!in) return false;
  std::ofstream out(dst, std::ios::binary | std::ios::trunc);
  if (!out) return false;
  out << in.rdbuf();
  return static_cast<bool>(out);
}

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
    // Always refresh from embed so disable_algos / Enable flips take effect.
    if (!copy_file(src, dst)) {
      LOGE("RkIspUserspace: calib copy failed %s -> %s", src.c_str(), dst.c_str());
      continue;
    }
    ok++;
    LOGW("RkIspUserspace: installed runtime calib %s", dst.c_str());
  }
  // refresh sizes for log
  for (const char *name : kCalibNames) {
    struct stat st {};
    const std::string dst = std::string(kCalibRuntimeDir) + "/" + name;
    if (stat(dst.c_str(), &st) == 0) {
      LOGW("RkIspUserspace: calib ready %s size=%ld", name, (long)st.st_size);
    }
  }
  if (ok < 3) {
    LOGE("RkIspUserspace: need 3 calib json under %s (from %s); got %d", kCalibRuntimeDir, kCalibEmbedDir, ok);
    return false;
  }
  // Prove we are not using vendor /etc/iqfiles ox03c10*
  if (access("/etc/iqfiles/ox03c10_D2V11K_9420.json", F_OK) == 0) {
    LOGW("RkIspUserspace: WARNING /etc/iqfiles still has ox03c10 json (should be unused)");
  } else {
    LOGW("RkIspUserspace: /etc/iqfiles has no ox03c10* (good)");
  }
  return true;
}

void RkIspUserspaceController::stop_rkaiq() {
  LOGW("RkIspUserspace: stopping rkaiq_3A (exclusive in-process CamHw)");
  // librkaiq needs writable /tmp/aiq*.lock (segfaults on EACCES).
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
    LOGW("RkIspUserspace: rkaiq_3A_server stopped");
  }
  ensure_runtime_calib();
}

void RkIspUserspaceController::start_rkaiq() {
  LOGW("RkIspUserspace: starting rkaiq_3A");
  if (std::system("sudo systemctl start rkaiq_3A.service") != 0) {
    std::system("sudo /usr/kommu/rkaiq_3A_server >/dev/null 2>&1 &");
  }
  usleep(2000000);
}

void RkIspUserspaceController::enable_frame_ae(bool on) {
  frame_ae_ = on;
  if (on) seed_exposure_from_sensor();
}

void RkIspUserspaceController::on_frame_grey(float grey_frac) {
  if (frame_ae_ && active_) ae_update(grey_frac);
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
  LOGW("RkIspUserspace cam%d: sns=%s vd=%s calib_dir=%s", cfg_.camera_num, sns, cfg_.mainpath_dev.c_str(),
       kCalibRuntimeDir);

  aiq_ = rk_aiq_uapi2_sysctl_init(sns, kCalibRuntimeDir, nullptr, nullptr);
  if (!aiq_) {
    LOGE("RkIspUserspace cam%d: sysctl_init failed (dir=%s)", cfg_.camera_num, kCalibRuntimeDir);
    return false;
  }
  if (multi_cam_n_ > 1) {
    rk_aiq_uapi2_sysctl_setMulCamConc(aiq_, true);
  }
  active_ = true;
  seed_exposure_from_sensor();
  return true;
}


static void apply_daylight_wb_ccm(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  static const float kDayWb[3][4] = {
    {1.45f, 1.0f, 1.0f, 1.35f},  // wide outdoor mild
    {1.45f, 1.0f, 1.0f, 1.35f},  // road outdoor mild
    {1.35f, 1.0f, 1.0f, 1.30f},  // driver
  };
  const int ci = std::clamp(camera_num, 0, 2);

  rk_aiq_uapiV2_wb_opMode_t op = {};
  op.mode = RK_AIQ_WB_MODE_MANUAL;
  XCamReturn r1 = rk_aiq_user_api2_awb_SetWpModeAttrib(aiq, op);

  rk_aiq_wb_mwb_attrib_t mwb = {};
  mwb.mode = RK_AIQ_MWB_MODE_WBGAIN;
  mwb.para.gain.rgain = kDayWb[ci][0];
  mwb.para.gain.grgain = kDayWb[ci][1];
  mwb.para.gain.gbgain = kDayWb[ci][2];
  mwb.para.gain.bgain = kDayWb[ci][3];
  XCamReturn r2 = rk_aiq_user_api2_awb_SetMwbAttrib(aiq, mwb);

  rk_aiq_wb_querry_info_t qi = {};
  rk_aiq_user_api2_awb_QueryWBInfo(aiq, &qi);
  LOGW("RkIspUserspace cam%d: WB man set=%d/%d query R=%.3f Gr=%.3f Gb=%.3f B=%.3f",
       camera_num, (int)r1, (int)r2, qi.gain.rgain, qi.gain.grgain, qi.gain.gbgain, qi.gain.bgain);

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
  XCamReturn r3 = rk_aiq_user_api2_accm_SetAttrib(aiq, &ccm);
  LOGW("RkIspUserspace cam%d: CCM SetAttrib=%d m00=%.3f", camera_num, (int)r3, ccm.stManual.Matrix.ccMatrix[0]);
}


static void apply_dehaze(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
  // rkisp ADHAZ (was DISABLE_ADHAZ). Prefer v11 MANUAL with mild outdoor clarity.
  adehaze_sw_v11_t attr;
  memset(&attr, 0, sizeof(attr));
  XCamReturn rg = rk_aiq_user_api2_adehaze_v11_getSwAttrib(aiq, &attr);

  attr.sync.sync_mode = RK_AIQ_UAPI_MODE_DEFAULT;
  attr.mode = DEHAZE_API_MANUAL;
  attr.stManual.Enable = true;
  attr.stManual.cfg_alpha = 1.0f;

  auto &dz = attr.stManual.dehaze_setting;
  dz.en = true;
  dz.air_lc_en = true;
  dz.stab_fnum = 8.f;
  dz.sigma = 6.f;
  dz.wt_sigma = 8.f;
  dz.air_sigma = 160.f;
  dz.tmax_sigma = 0.01f;
  dz.pre_wet = 8.f;
  // DehazeData scalars (calib EnvLv tables collapse to these daytime defaults)
  dz.DehazeData.dc_min_th = 64.f;
  dz.DehazeData.dc_max_th = 192.f;
  dz.DehazeData.yhist_th = 249.f;
  dz.DehazeData.yblk_th = 0.002f;
  dz.DehazeData.dark_th = 250.f;
  dz.DehazeData.bright_min = 180.f;
  dz.DehazeData.bright_max = 240.f;
  dz.DehazeData.wt_max = 0.9f;
  dz.DehazeData.air_min = 200.f;
  dz.DehazeData.air_max = 250.f;
  dz.DehazeData.tmax_base = 125.f;
  dz.DehazeData.tmax_off = 0.1f;
  dz.DehazeData.tmax_max = 0.8f;
  dz.DehazeData.cfg_wt = 0.85f;
  dz.DehazeData.cfg_air = 170.f;
  dz.DehazeData.cfg_tmax = 0.12f;
  dz.DehazeData.dc_weitcur = 0.f;
  dz.DehazeData.bf_weight = 0.5f;
  dz.DehazeData.range_sigma = 0.14f;
  dz.DehazeData.space_sigma_pre = 0.14f;
  dz.DehazeData.space_sigma_cur = 0.14f;

  auto &enh = attr.stManual.enhance_setting;
  enh.en = true;
  enh.EnhanceData.enhance_value = 1.0f;
  enh.EnhanceData.enhance_chroma = 1.0f;
  static const float kEnhCurve[17] = {
    0,64,128,192,256,320,384,448,512,576,640,704,768,832,896,960,1023};
  for (int i = 0; i < 17; ++i) enh.enhance_curve[i] = kEnhCurve[i];

  // hist off initially — dehaze+enhance only (less AE interaction)
  attr.stManual.hist_setting.en = false;
  attr.stManual.hist_setting.hist_para_en = false;

  attr.Info.updateMDehazeStrth = true;
  attr.Info.MDehazeStrth = 70;
  attr.Info.updateMEnhanceStrth = true;
  attr.Info.MEnhanceStrth = 40;

  XCamReturn rs = rk_aiq_user_api2_adehaze_v11_setSwAttrib(aiq, &attr);

  // imgproc strength APIs (active in manual)
  XCamReturn r0 = rk_aiq_uapi2_setDehazeModuleEnable(aiq, true);
  XCamReturn r1 = rk_aiq_uapi2_setDehazeEnable(aiq, true);
  XCamReturn r2 = rk_aiq_uapi2_setEnhanceEnable(aiq, true);
  XCamReturn r3 = rk_aiq_uapi2_setMDehazeStrth(aiq, 70);
  XCamReturn r4 = rk_aiq_uapi2_setMEnhanceStrth(aiq, 40);

  adehaze_sw_v11_t got;
  memset(&got, 0, sizeof(got));
  got.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
  rk_aiq_user_api2_adehaze_v11_getSwAttrib(aiq, &got);
  unsigned int str = 0;
  rk_aiq_uapi2_getMDehazeStrth(aiq, &str);
  LOGW("RkIspUserspace cam%d: dehaze get0=%d set=%d img=%d/%d/%d str=%d/%d "
       "mode=%d manEn=%d dz=%d enh=%d autoEn=%d MStr=%u/%u",
       camera_num, (int)rg, (int)rs, (int)r0, (int)r1, (int)r2, (int)r3, (int)r4,
       (int)got.mode, (int)got.stManual.Enable, (int)got.stManual.dehaze_setting.en,
       (int)got.stManual.enhance_setting.en,
       (int)got.stAuto.DehazeTuningPara.Enable, got.Info.MDehazeStrth, str);
}

bool RkIspUserspaceController::prepare_and_start() {
  if (!active_ || !aiq_ || started_) return active_ && started_;
  XCamReturn r = rk_aiq_uapi2_sysctl_prepare(aiq_, 1920, 1200, RK_AIQ_WORKING_MODE_NORMAL);
  if (r != XCAM_RETURN_NO_ERROR) {
    LOGE("RkIspUserspace cam%d: prepare failed %d", cfg_.camera_num, (int)r);
    return false;
  }
  // Daylight WB + ox03c10 CCM (rkisp needs channel WB; Spectra CCM is gray-preserving).
  rk_aiq_uapi2_setExpMode(aiq_, OP_MANUAL);
  // BLC: ox03c10 black_level = 0
  {
    rk_aiq_blc_attrib_t blc = {};
    blc.eMode = ABLC_OP_MODE_MANUAL;
    blc.stBlc0Manual.enable = true;
    blc.stBlc0Manual.blc_r = 0;
    blc.stBlc0Manual.blc_gr = 0;
    blc.stBlc0Manual.blc_gb = 0;
    blc.stBlc0Manual.blc_b = 0;
    blc.stBlc1Manual.enable = false;
    XCamReturn br = rk_aiq_user_api2_ablc_SetAttrib(aiq_, &blc);
    if (br != XCAM_RETURN_NO_ERROR) {
      LOGW("RkIspUserspace cam%d: ablc_SetAttrib failed %d", cfg_.camera_num, (int)br);
    }
  }

  apply_daylight_wb_ccm(aiq_, cfg_.camera_num);

  // Adegamma (SDG): ox03c10 PWL inverse — requires librkaiq rebuilt with DEGAMMA_V1.
  {
    rk_aiq_degamma_attr_t dg = {};
    dg.mode = RK_AIQ_DEGAMMA_MODE_MANUAL;
    dg.Scene_mode = DEGAMMA_OUT_NORMAL;
    dg.stManual.en = true;
    for (int i = 0; i < rk_tone::kDegammaKnots; i++) {
      dg.stManual.X_axis[i] = rk_tone::kDegammaX[i];
      dg.stManual.curve_R[i] = rk_tone::kOx03c10DegammaY[i];
      dg.stManual.curve_G[i] = rk_tone::kOx03c10DegammaY[i];
      dg.stManual.curve_B[i] = rk_tone::kOx03c10DegammaY[i];
    }
    XCamReturn dr = rk_aiq_user_api2_adegamma_SetAttrib(aiq_, dg);
    if (dr != XCAM_RETURN_NO_ERROR) {
      LOGW("RkIspUserspace cam%d: adegamma_SetAttrib failed %d", cfg_.camera_num, (int)dr);
    }
  }
  {
    rk_aiq_gamma_v11_attr_t gam = {};
    gam.sync.done = false;
    gam.mode = RK_AIQ_GAMMA_MODE_MANUAL;
    gam.stManual.Gamma_en = true;
    gam.stManual.Gamma_out_offset = 0;
    for (int i = 0; i < rk_tone::kGammaKnots; i++) {
      gam.stManual.Gamma_curve[i] = rk_tone::kOx03c10GammaLinearV11[i];
    }
    XCamReturn gr = rk_aiq_user_api2_agamma_v11_SetAttrib(aiq_, &gam);
    if (gr != XCAM_RETURN_NO_ERROR) {
      LOGW("RkIspUserspace cam%d: agamma_v11_SetAttrib failed %d", cfg_.camera_num, (int)gr);
    }
  }
  r = rk_aiq_uapi2_sysctl_start(aiq_);
  if (r != XCAM_RETURN_NO_ERROR) {
    LOGE("RkIspUserspace cam%d: start failed %d", cfg_.camera_num, (int)r);
    return false;
  }
  started_ = true;
  apply_daylight_wb_ccm(aiq_, cfg_.camera_num);

  // Re-apply tone after start (some ISP modules ignore pre-start manual attrs).
  {
    rk_aiq_degamma_attr_t dg = {};
    dg.mode = RK_AIQ_DEGAMMA_MODE_MANUAL;
    dg.Scene_mode = DEGAMMA_OUT_NORMAL;
    dg.stManual.en = true;
    for (int i = 0; i < rk_tone::kDegammaKnots; i++) {
      dg.stManual.X_axis[i] = rk_tone::kDegammaX[i];
      dg.stManual.curve_R[i] = rk_tone::kOx03c10DegammaY[i];
      dg.stManual.curve_G[i] = rk_tone::kOx03c10DegammaY[i];
      dg.stManual.curve_B[i] = rk_tone::kOx03c10DegammaY[i];
    }
    XCamReturn dr = rk_aiq_user_api2_adegamma_SetAttrib(aiq_, dg);
    rk_aiq_degamma_attr_t got_dg = {};
    rk_aiq_user_api2_adegamma_GetAttrib(aiq_, &got_dg);
    LOGW("RkIspUserspace cam%d: adegamma set=%d mode=%d en=%d y8=%d y16=%d",
         cfg_.camera_num, (int)dr, (int)got_dg.mode, got_dg.stManual.en ? 1 : 0,
         got_dg.stManual.curve_G[8], got_dg.stManual.curve_G[16]);
  }
  {
    rk_aiq_gamma_v11_attr_t gam = {};
    gam.sync.done = false;
    gam.mode = RK_AIQ_GAMMA_MODE_MANUAL;
    gam.stManual.Gamma_en = true;
    gam.stManual.Gamma_out_offset = 0;
    for (int i = 0; i < rk_tone::kGammaKnots; i++) {
      gam.stManual.Gamma_curve[i] = rk_tone::kOx03c10GammaLinearV11[i];
    }
    XCamReturn gr = rk_aiq_user_api2_agamma_v11_SetAttrib(aiq_, &gam);
    rk_aiq_gamma_v11_attr_t got = {};
    rk_aiq_user_api2_agamma_v11_GetAttrib(aiq_, &got);
    LOGW("RkIspUserspace cam%d: agamma set=%d mode=%d en=%d c0=%d c24=%d c48=%d",
         cfg_.camera_num, (int)gr, (int)got.mode, got.stManual.Gamma_en ? 1 : 0,
         (int)got.stManual.Gamma_curve[0], (int)got.stManual.Gamma_curve[24],
         (int)got.stManual.Gamma_curve[48]);
  }
  apply_dehaze(aiq_, cfg_.camera_num);

  LOGW("RkIspUserspace cam%d: CamHw start ok (BLC0 + Spectra lin + LSC(json road) + daylight WB + ox03c10 CCM/gamma + dehaze, calib=%s)",
       cfg_.camera_num, kCalibRuntimeDir);
  {
    rk_aiq_lsc_querry_info_t lq = {};
    if (rk_aiq_user_api2_alsc_QueryLscInfo(aiq_, &lq) == XCAM_RETURN_NO_ERROR) {
      LOGW("RkIspUserspace cam%d: LSC query en=%d centerR=%u", cfg_.camera_num,
           (int)lq.lsc_en, (unsigned)lq.r_data_tbl[144]);
    }
  }

  return true;
}

void RkIspUserspaceController::seed_exposure_from_sensor() {
  if (cfg_.sensor_ctrl_fd < 0) return;
  struct v4l2_control c = {};
  c.id = V4L2_CID_EXPOSURE;
  if (ioctl(cfg_.sensor_ctrl_fd, VIDIOC_G_CTRL, &c) == 0 && c.value > 0) {
    exposure_time_ = std::clamp(c.value, 2, 2217);
  }
  c = {};
  c.id = V4L2_CID_ANALOGUE_GAIN;
  if (ioctl(cfg_.sensor_ctrl_fd, VIDIOC_G_CTRL, &c) == 0 && c.value > 0) {
    again_ = std::clamp(c.value, 16, 248);
  }
  float again_lin = again_ / 16.0f;
  float ev = exposure_time_ * again_lin;
  cur_ev_[0] = cur_ev_[1] = cur_ev_[2] = ev;
}

bool RkIspUserspaceController::set_sensor_exposure(int exp_lines, int again) {
  if (cfg_.sensor_ctrl_fd < 0) return false;
  exp_lines = std::clamp(exp_lines, 2, 2217);
  again = std::clamp(again, 16, 248);
  struct v4l2_control c = {};
  c.id = V4L2_CID_EXPOSURE;
  c.value = exp_lines;
  if (ioctl(cfg_.sensor_ctrl_fd, VIDIOC_S_CTRL, &c) < 0) return false;
  c.id = V4L2_CID_ANALOGUE_GAIN;
  c.value = again;
  if (ioctl(cfg_.sensor_ctrl_fd, VIDIOC_S_CTRL, &c) < 0) return false;
  exposure_time_ = exp_lines;
  again_ = again;
  return true;
}

void RkIspUserspaceController::ae_update(float grey_frac) {
  if (!(grey_frac > 0.f) || !std::isfinite(grey_frac)) return;
  const float target_grey_factor = 0.01f;
  const float dt = 0.05f;
  const float ts_grey = 10.0f;
  const float ts_ev = 0.05f;
  const float k_grey = (dt / ts_grey) / (1.0f + dt / ts_grey);
  const float k_ev = (dt / ts_ev) / (1.0f + dt / ts_ev);

  float again_lin = again_ / 16.0f;
  float cur_ev = exposure_time_ * again_lin;
  cur_ev_[frame_i_ % 3] = cur_ev;
  float cur_ev_lag = cur_ev_[(frame_i_ + 2) % 3];

  float new_target = std::clamp(0.4f - 0.3f * log2f(1.0f + target_grey_factor * cur_ev_lag) / log2f(6000.0f),
                                0.1f, 0.4f);
  target_grey_ = (1.0f - k_grey) * target_grey_ + k_grey * new_target;

  float desired_ev = std::clamp(cur_ev_lag * target_grey_ / grey_frac, 2.0f, 2217.0f * (248.0f / 16.0f));
  float k = (1.0f - k_ev) / 3.0f;
  desired_ev = k * cur_ev_[0] + k * cur_ev_[1] + k * cur_ev_[2] + k_ev * desired_ev;

  int again = 16;
  int exp = static_cast<int>(std::lround(desired_ev / (again / 16.0f)));
  if (exp > 2217) {
    float need = desired_ev / 2217.0f;
    again = std::clamp(static_cast<int>(std::lround(need * 16.0f)), 16, 248);
    exp = static_cast<int>(std::lround(desired_ev / (again / 16.0f)));
  }
  exp = std::clamp(exp, 2, 2217);
  if (set_sensor_exposure(exp, again) && (frame_i_ % 30 == 0)) {
    LOGD("RkIspUserspace cam%d AE grey=%.4f target=%.3f exp=%d again=%d", cfg_.camera_num, grey_frac,
         target_grey_, exp, again);
  }
  frame_i_++;
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
  frame_ae_ = false;
}
