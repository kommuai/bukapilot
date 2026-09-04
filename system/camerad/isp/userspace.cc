#include "system/camerad/isp/userspace.h"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <common/rkisp3-config.h>
#include <uAPI2/rk_aiq_user_api2_sysctl.h>
#include <uAPI2/rk_aiq_user_api2_imgproc.h>
#include <uAPI2/rk_aiq_user_api2_agamma.h>
#include <uAPI2/rk_aiq_user_api2_accm.h>
#include <uAPI2/rk_aiq_user_api2_awb.h>
#include <algos/accm/rk_aiq_types_accm_algo.h>
#include "system/camerad/isp/calibration.h"
#include "system/camerad/isp/tone_tables.h"

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

int duplicate_open_fd(const char *path) {
  DIR *dir = opendir("/proc/self/fd");
  if (!dir) return -1;

  int duplicate = -1;
  struct dirent *entry = nullptr;
  while ((entry = readdir(dir)) != nullptr) {
    char *end = nullptr;
    const long fd = strtol(entry->d_name, &end, 10);
    if (!end || *end != '\0' || fd < 0) continue;

    char link[256];
    const ssize_t length = readlink((std::string("/proc/self/fd/") + entry->d_name).c_str(),
                                    link, sizeof(link) - 1);
    if (length <= 0) continue;
    link[length] = '\0';
    if (strcmp(link, path) != 0) continue;

    duplicate = dup(static_cast<int>(fd));
    break;
  }
  closedir(dir);
  return duplicate;
}

struct RkCamSync {
  int expected = 1;
};

RkCamSync g_sync;

}  // namespace

static void apply_ccm(const rk_aiq_sys_ctx_t *aiq, int camera_num);

int RkIspUserspaceController::multi_cam_n_ = 1;
std::mutex RkIspUserspaceController::custom_ae_mutex_;
std::map<rk_aiq_sys_ctx_s *, RkIspUserspaceController *> RkIspUserspaceController::custom_ae_controllers_;

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

int32_t RkIspUserspaceController::custom_ae_init(void *) {
  return 0;
}

int32_t RkIspUserspaceController::custom_ae_run(void *ctx,
                                                 const rk_aiq_customAe_stats_t *stats,
                                                 rk_aiq_customeAe_results_t *result) {
  auto *aiq = static_cast<rk_aiq_sys_ctx_s *>(ctx);
  std::lock_guard lk(custom_ae_mutex_);
  const auto it = custom_ae_controllers_.find(aiq);
  if (it == custom_ae_controllers_.end() || !it->second->cfg_.ae_run) return -1;
  return it->second->cfg_.ae_run(it->second->cfg_.ae_owner, stats, result);
}

int32_t RkIspUserspaceController::custom_ae_exit(void *) {
  return 0;
}

bool RkIspUserspaceController::register_custom_ae() {
  if (!aiq_ || !cfg_.ae_run || custom_ae_registered_) return custom_ae_registered_;

  rk_aiq_customeAe_cbs_t cbs = {};
  cbs.pfn_ae_init = &RkIspUserspaceController::custom_ae_init;
  cbs.pfn_ae_run = &RkIspUserspaceController::custom_ae_run;
  cbs.pfn_ae_exit = &RkIspUserspaceController::custom_ae_exit;
  const XCamReturn rc = rk_aiq_uapi2_customAE_register(aiq_, &cbs);
  if (rc != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: custom AE registration failed (%d)", cfg_.camera_num, (int)rc);
    return false;
  }

  {
    std::lock_guard lk(custom_ae_mutex_);
    custom_ae_controllers_[aiq_] = this;
  }
  custom_ae_registered_ = true;
  return true;
}

void RkIspUserspaceController::unregister_custom_ae() {
  if (!aiq_) return;
  if (custom_ae_enabled_) {
    (void)rk_aiq_uapi2_customAE_enable(aiq_, false);
    custom_ae_enabled_ = false;
  }
  if (custom_ae_registered_) {
    (void)rk_aiq_uapi2_customAE_unRegister(aiq_);
    std::lock_guard lk(custom_ae_mutex_);
    custom_ae_controllers_.erase(aiq_);
    custom_ae_registered_ = false;
  }
}

bool RkIspUserspaceController::attach_direct_stats() {
  const std::string stats_dev = resolve_v4l_dev_by_name_index("rkisp-statistics", cfg_.camera_num);
  if (stats_dev.empty()) {
    LOGW("RkIspUserspace cam%d: rkisp-statistics node not found", cfg_.camera_num);
    return false;
  }

  // CamHw already owns this stream. Duplicate its file description instead of
  // opening/streaming the node a second time; the latter can wedge rkisp_v6.
  direct_stats_fd_ = duplicate_open_fd(stats_dev.c_str());
  if (direct_stats_fd_ < 0) {
    LOGW("RkIspUserspace cam%d: existing stats fd for %s not found", cfg_.camera_num, stats_dev.c_str());
    return false;
  }

  for (size_t i = 0; i < direct_stats_maps_.size(); i++) {
    struct v4l2_buffer query = {};
    query.type = V4L2_BUF_TYPE_META_CAPTURE;
    query.memory = V4L2_MEMORY_MMAP;
    query.index = i;
    if (ioctl(direct_stats_fd_, VIDIOC_QUERYBUF, &query) < 0 || query.length == 0) {
      LOGW("RkIspUserspace cam%d: stats QUERYBUF %zu failed errno=%d", cfg_.camera_num, i, errno);
      close_direct_stats();
      return false;
    }
    void *mapping = mmap(nullptr, query.length, PROT_READ, MAP_SHARED, direct_stats_fd_, query.m.offset);
    if (mapping == MAP_FAILED) {
      LOGW("RkIspUserspace cam%d: stats mmap %zu failed errno=%d", cfg_.camera_num, i, errno);
      close_direct_stats();
      return false;
    }
    direct_stats_maps_[i] = mapping;
    direct_stats_map_sizes_[i] = query.length;
  }

  LOG("RkIspUserspace cam%d: direct ISP statistics attached to %s fd=%d buffers=%zu",
      cfg_.camera_num, stats_dev.c_str(), direct_stats_fd_, direct_stats_maps_.size());
  return true;
}

void RkIspUserspaceController::close_direct_stats() {
  for (size_t i = 0; i < direct_stats_maps_.size(); i++) {
    if (direct_stats_maps_[i]) {
      munmap(direct_stats_maps_[i], direct_stats_map_sizes_[i]);
      direct_stats_maps_[i] = nullptr;
      direct_stats_map_sizes_[i] = 0;
    }
  }
  if (direct_stats_fd_ >= 0) {
    close(direct_stats_fd_);
    direct_stats_fd_ = -1;
  }
}

bool RkIspUserspaceController::get_ae_shadow_stats(RkIspAeShadowStats *stats) {
  if (!stats || direct_stats_fd_ < 0) return false;

  struct pollfd pfd = {direct_stats_fd_, POLLIN, 0};
  if (poll(&pfd, 1, 0) <= 0 || !(pfd.revents & POLLIN)) return false;

  struct v4l2_buffer buffer = {};
  buffer.type = V4L2_BUF_TYPE_META_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;
  if (ioctl(direct_stats_fd_, VIDIOC_DQBUF, &buffer) < 0) return false;
  if (buffer.index >= direct_stats_maps_.size() || !direct_stats_maps_[buffer.index]) {
    (void)ioctl(direct_stats_fd_, VIDIOC_QBUF, &buffer);
    return false;
  }

  const auto *raw = static_cast<const rkisp3x_isp_stat_buffer *>(direct_stats_maps_[buffer.index]);
  RkIspAeShadowStats snapshot = {};
  snapshot.frame_id = raw->frame_id;

  const isp2x_rawaebig_stat *raw_big[3] = {
    &raw->params.rawae1, &raw->params.rawae2, &raw->params.rawae3,
  };
  const isp2x_rawhistbig_stat *hist_big[3] = {
    &raw->params.rawhist1, &raw->params.rawhist2, &raw->params.rawhist3,
  };

  for (int channel = 0; channel < 4; channel++) {
    uint64_t sum_luma = 0;
    const uint32_t *hist = nullptr;
    if (channel == 0) {
      for (int i = 0; i < ISP2X_RAWAELITE_MEAN_NUM; i++) {
        const auto &value = raw->params.rawae0.data[i];
        sum_luma += (77 * value.channelr_xy + 150 * value.channelg_xy + 29 * value.channelb_xy) / 256;
      }
      hist = raw->params.rawhist0.hist_bin;
      sum_luma /= ISP2X_RAWAELITE_MEAN_NUM;
    } else {
      const auto &raw_value = *raw_big[channel - 1];
      for (int i = 0; i < ISP2X_RAWAEBIG_MEAN_NUM; i++) {
        const auto &value = raw_value.data[i];
        sum_luma += (77 * value.channelr_xy + 150 * value.channelg_xy + 29 * value.channelb_xy) / 256;
      }
      hist = hist_big[channel - 1]->hist_bin;
      sum_luma /= ISP2X_RAWAEBIG_MEAN_NUM;
    }
    snapshot.raw_mean[channel] = static_cast<uint16_t>(std::min<uint64_t>(sum_luma, UINT16_MAX));

    uint64_t total = 0;
    for (int i = 0; i < ISP2X_HIST_BIN_N_MAX; i++) total += hist[i];
    snapshot.histogram_total[channel] = total;
    const uint64_t midpoint = (total + 1) / 2;
    uint64_t cumulative = 0;
    for (int i = 0; i < ISP2X_HIST_BIN_N_MAX; i++) {
      cumulative += hist[i];
      if (cumulative >= midpoint) {
        snapshot.histogram_median[channel] = i;
        break;
      }
    }
  }

  if (ioctl(direct_stats_fd_, VIDIOC_QBUF, &buffer) < 0) return false;
  *stats = snapshot;
  return true;
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
  if (cfg_.ae_run && !register_custom_ae()) {
    LOGW("RkIspUserspace cam%d: using manual AE fallback", cfg_.camera_num);
  }
  // Seed CCM before sysctl_prepare generates the initial ISP parameters.
  apply_ccm(aiq_, cfg.camera_num);

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

static void apply_gamma(const rk_aiq_sys_ctx_t *aiq, int camera_num) {
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

  if (custom_ae_registered_) {
    const XCamReturn rc = rk_aiq_uapi2_customAE_enable(aiq_, true);
    if (rc == XCAM_RETURN_NO_ERROR) {
      custom_ae_enabled_ = true;
      LOG("RkIspUserspace cam%d: RKAIQ custom AE enabled", cfg_.camera_num);
    } else {
      LOGW("RkIspUserspace cam%d: custom AE enable failed (%d), using manual fallback",
           cfg_.camera_num, (int)rc);
    }
  }

  if (!custom_ae_enabled_ && rk_aiq_uapi2_sysctl_setModuleCtl(aiq_, RK_MODULE_AE, false) != XCAM_RETURN_NO_ERROR) {
    LOGW("RkIspUserspace cam%d: failed to disable stock AE fallback", cfg_.camera_num);
  }

  if (!disable_isp_nr_and_blc(aiq_, cfg_.camera_num)) {
    LOGE("RkIspUserspace cam%d: refusing to continue with RKAIQ NR/BLC state unverified", cfg_.camera_num);
    return false;
  }

  apply_gamma(aiq_, cfg_.camera_num);
  prepared_ = true;
  return true;
}

bool RkIspUserspaceController::start() {
  if (!active_ || !aiq_ || !prepared_ || started_) return active_ && started_;

  // Disable AWB before the analyzer threads can consume their first stats.
  // The KA2 profile intentionally has no AWB calibration because sensor WB is
  // fixed; doing this after start races the first group-analysis pass.
  disable_isp_awb(aiq_, cfg_.camera_num);

  XCamReturn r = rk_aiq_uapi2_sysctl_start(aiq_);
  if (r != XCAM_RETURN_NO_ERROR) {
    LOGE("RkIspUserspace cam%d: start failed %d", cfg_.camera_num, (int)r);
    return false;
  }
  started_ = true;

  if (!custom_ae_enabled_) (void)attach_direct_stats();

  return true;
}

void RkIspUserspaceController::shutdown() {
  close_direct_stats();
  if (aiq_) {
    if (started_) {
      rk_aiq_uapi2_sysctl_stop(aiq_, false);
      started_ = false;
    }
    unregister_custom_ae();
    rk_aiq_uapi2_sysctl_deinit(aiq_);
    aiq_ = nullptr;
  }
  clear_calibration_registration();
  active_ = false;
  prepared_ = false;
}
