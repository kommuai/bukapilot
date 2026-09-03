#include "system/camerad/rk/ka2_camera_backend.h"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "common/swaglog.h"
#include "system/camerad/cameras/camera_rk.h"
#include "system/camerad/rk/rk_pwl_regs.h"
#include "third_party/linux/include/v4l2-controls.h"

namespace {

constexpr bool kEnableWideRoad = true;
constexpr bool kEnableRoad = true;
constexpr bool kEnableDriver = true;

constexpr uint16_t kGrpHoldReg = 0x3208;
constexpr int kI2cChunk = 8;
constexpr int kI2cRetries = 5;
constexpr uint32_t kTemperaturePollPeriod = 8;

#define V4L2_CID_X3C_SENSOR_TEMPERATURE (V4L2_CID_USER_BASE + 0x100)

bool i2c_retryable_errno(int err) {
  return err == ETIMEDOUT || err == EREMOTEIO || err == EBUSY || err == EAGAIN;
}

bool i2c_write_msgs(int fd, struct i2c_msg *msgs, int nmsgs) {
  struct i2c_rdwr_ioctl_data data = {.msgs = msgs, .nmsgs = (__u32)nmsgs};
  for (int attempt = 0; attempt < kI2cRetries; attempt++) {
    if (ioctl(fd, I2C_RDWR, &data) >= 0) return true;
    const int err = errno;
    if (!i2c_retryable_errno(err) || attempt + 1 == kI2cRetries) {
      errno = err;
      return false;
    }
    usleep(2000 * (attempt + 1));
  }
  return false;
}

bool i2c_write_reg8(int fd, uint16_t addr, uint16_t reg, uint8_t val) {
  uint8_t buf[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), val};
  struct i2c_msg msg = {};
  msg.addr = addr;
  msg.flags = 0;
  msg.len = 3;
  msg.buf = buf;
  return i2c_write_msgs(fd, &msg, 1);
}

}  // namespace


bool Ka2CameraBackend::write_ctrl(const CameraState *cam, uint32_t id, int val) const {
  struct v4l2_control c = {};
  c.id = id;
  c.value = val;
  return ioctl(cam->ctrl_fd, VIDIOC_S_CTRL, &c) >= 0;
}

bool Ka2CameraBackend::read_ctrl(const CameraState *cam, uint32_t id, int *out) const {
  struct v4l2_control c = {};
  c.id = id;
  if (ioctl(cam->ctrl_fd, VIDIOC_G_CTRL, &c) < 0) return false;
  *out = c.value;
  return true;
}

std::string Ka2CameraBackend::resolve_mainpath_dev(int camera_num) const {
  int n = 0;
  for (int i = 0; i < 64; i++) {
    char syspath[64], name[128];
    snprintf(syspath, sizeof(syspath), "/sys/class/video4linux/video%d/name", i);
    FILE *f = fopen(syspath, "r");
    if (!f) continue;
    if (!fgets(name, sizeof(name), f)) {
      fclose(f);
      continue;
    }
    fclose(f);
    name[strcspn(name, "\n")] = 0;
    if (strcmp(name, "rkisp_mainpath") != 0) continue;
    if (n++ != camera_num) continue;
    char path[32];
    snprintf(path, sizeof(path), "/dev/video%d", i);
    return path;
  }
  return {};
}

bool Ka2CameraBackend::open(CameraState *cam) {
  char device[32];
  snprintf(device, sizeof(device), "/dev/v4l-subdev%d", cam->camera_num * 5 + 2);
  cam->ctrl_fd = ::open(device, O_RDWR);
  if (cam->ctrl_fd < 0) return false;

  // Configure orientation before rkaiq reads the kernel's Bayer format.
  apply_flips(cam);

  int temp_raw = 0;
  if (read_ctrl(cam, V4L2_CID_X3C_SENSOR_TEMPERATURE, &temp_raw)) {
    last_sensor_temp_c_ = temp_raw / 100.0f;
  }

  cam->ci = std::make_unique<OX03C10>();
  static const int kI2cBusByCam[3] = {1, 3, 6};
  i2c_bus_ = kI2cBusByCam[std::clamp(cam->camera_num, 0, 2)];
  // KA2 places every OX03C10 at 0x36 on its own I2C bus.
  i2c_addr_ = 0x36;
  // Keep the post-open write as well: STREAMON may reset sensor controls on
  // some kernel revisions, while rkaiq must see the orientation during init.
  apply_flips(cam);
  // A reopened sensor has lost its register state; force the first AE update
  // to submit the complete command even if the requested values are equal.
  frame_length_vts_ = 0;
  exposure_reg_count_ = 0;
  last_exp_reg_count_ = 0;
  dc_gain_weight_ = cam->ci->dc_gain_min_weight;
  gain_idx_ = cam->ci->analog_gain_rec_idx;
  exposure_time_ = 5;
  dc_gain_enabled_ = false;
  analog_gain_frac_ = cam->ci->sensor_analog_gains[gain_idx_];
  {
    float g0 = get_gain_factor(cam) * analog_gain_frac_ * exposure_time_;
    cur_ev_[0] = cur_ev_[1] = cur_ev_[2] = g0;
  }
  set_exposure_rect(cam);

  cam->video_fd = open_v4l_by_name_and_index("rkisp_mainpath", cam->camera_num);
  if (cam->video_fd < 0) return false;

  rk_isp_ = std::make_unique<RkIspUserspaceController>();
  RkIspCamConfig cfg;
  cfg.camera_num = cam->camera_num;
  cfg.mainpath_dev = resolve_mainpath_dev(cam->camera_num);
  if (!rk_isp_->init(cfg)) {
    LOGE("camera %d: RkIspUserspace CamHw init failed", cam->camera_num);
    rk_isp_->shutdown();
    rk_isp_.reset();
    return false;
  }
  if (!seed_external_exposure(cam)) {
    LOGE("camera %d: failed to seed initial external exposure", cam->camera_num);
    rk_isp_->shutdown();
    rk_isp_.reset();
    return false;
  }
  return true;
}

void Ka2CameraBackend::close(CameraState *cam) {
  if (rk_isp_) {
    rk_isp_->shutdown();
    rk_isp_.reset();
  }
  if (i2c_fd_.fd_ >= 0) {
    ::close(i2c_fd_.fd_);
    i2c_fd_.fd_ = -1;
  }
}

void Ka2CameraBackend::on_stream_start(CameraState *cam) {
  apply_pwl_on(cam);
}

float Ka2CameraBackend::get_gain_factor(const CameraState *cam) const {
  // LCG is unity.  Only HCG contributes the sensor's DCG ratio.
  if (!cam->ci || !dc_gain_enabled_) return 1.0f;
  return (1.0f + dc_gain_weight_ * (cam->ci->dc_gain_factor - 1.0f) / std::max(1, cam->ci->dc_gain_max_weight));
}

bool Ka2CameraBackend::set_frame_length_vts(CameraState *cam, int exposure_lines) {
  if (!cam || !cam->ci || i2c_bus_ < 0) return false;
  const int vts = std::clamp(
      std::max(ox03c10_limits::kMinVts, exposure_lines + ox03c10_limits::kHdr4Margin),
      ox03c10_limits::kMinVts, ox03c10_limits::kMaxVts);
  if (vts == frame_length_vts_) return true;

  // 0x380e/0x380f are the sensor's total frame length (VTS), not VBLANK.
  frame_length_vts_ = vts;
  return true;
}

void Ka2CameraBackend::apply_pwl_on(CameraState *cam) {
  std::vector<i2c_random_wr_payload> wr;
  wr.reserve(rk_pwl::kOx03c10PwlOnLen);
  for (size_t i = 0; i < rk_pwl::kOx03c10PwlOnLen; i++) {
    wr.push_back({rk_pwl::kOx03c10PwlOn[i].addr, rk_pwl::kOx03c10PwlOn[i].data});
  }
  sensors_i2c(cam, wr.data(), (int)wr.size());
}

bool Ka2CameraBackend::sensors_i2c(CameraState *cam, const i2c_random_wr_payload *dat, int len) {
  if (i2c_bus_ < 0 || !dat || len <= 0) return false;
  std::lock_guard lk(i2c_lock_);
  if (i2c_fd_.fd_ < 0) {
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", i2c_bus_);
    int fd = ::open(path, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
      if (cam->frame_id_last % 120 == 0) {
        LOGE("camera %d: open %s failed errno=%d", cam->camera_num, path, errno);
      }
      return false;
    }
    i2c_fd_.fd_ = fd;
  }
  const bool ae_regs = len <= 64 && std::any_of(dat, dat + len, [](const i2c_random_wr_payload &r) {
    return (r.reg_addr >= 0x3500 && r.reg_addr < 0x3600) || r.reg_addr == 0x380e || r.reg_addr == 0x380f;
  });
  if (ae_regs && !i2c_write_reg8(i2c_fd_.fd_, (uint16_t)i2c_addr_, kGrpHoldReg, 0x00)) {
    const int err = errno;
    if (cam->frame_id_last % 120 == 0) {
      LOGE("camera %d: i2c group hold start failed errno=%d", cam->camera_num, err);
    }
    ::close(i2c_fd_.fd_);
    i2c_fd_.fd_ = -1;
    errno = err;
    return false;
  }

  // Wide and driver require delayed launch; road accepts quick launch.
  const uint8_t group_launch = cam->camera_num == 1 ? 0xE0 : 0xA0;
  const auto release_group_hold = [&] {
    return i2c_write_reg8(i2c_fd_.fd_, (uint16_t)i2c_addr_, kGrpHoldReg, 0x10) &&
           i2c_write_reg8(i2c_fd_.fd_, (uint16_t)i2c_addr_, kGrpHoldReg, group_launch);
  };

  uint8_t bufs[kI2cChunk][3];
  struct i2c_msg msgs[kI2cChunk];
  for (int off = 0; off < len; off += kI2cChunk) {
    const int chunk = std::min(kI2cChunk, len - off);
    for (int i = 0; i < chunk; i++) {
      bufs[i][0] = (uint8_t)((dat[off + i].reg_addr >> 8) & 0xff);
      bufs[i][1] = (uint8_t)(dat[off + i].reg_addr & 0xff);
      bufs[i][2] = (uint8_t)(dat[off + i].reg_data & 0xff);
      msgs[i] = {};
      msgs[i].addr = (uint16_t)i2c_addr_;
      msgs[i].flags = 0;
      msgs[i].len = 3;
      msgs[i].buf = bufs[i];
    }
    if (!i2c_write_msgs(i2c_fd_.fd_, msgs, chunk)) {
      const int err = errno;
      if (cam->frame_id_last % 120 == 0) LOGE("camera %d: i2c batch wr failed errno=%d", cam->camera_num, err);
      if (ae_regs) (void)release_group_hold();
      ::close(i2c_fd_.fd_);
      i2c_fd_.fd_ = -1;
      errno = err;
      return false;
    }
  }
  if (ae_regs) {
    if (!release_group_hold()) {
      const int err = errno;
      if (cam->frame_id_last % 120 == 0) {
        LOGE("camera %d: i2c group hold launch failed errno=%d", cam->camera_num, err);
      }
      ::close(i2c_fd_.fd_);
      i2c_fd_.fd_ = -1;
      errno = err;
      return false;
    }
  }
  return true;
}

void Ka2CameraBackend::set_exposure_rect(CameraState *cam) {
  static const struct { Rect r; float fl_ref; float focal_mm; } kAeTargets[3] = {
    {{96, 400, 1734, 524}, 567.0f, 1.71f},
    {{96, 160, 1734, 986}, 2648.0f, 8.0f},
    {{96, 242, 1736, 906}, 567.0f, 1.71f},
  };
  const int idx = std::clamp(cam->camera_num, 0, 2);
  const Rect xywh_ref = kAeTargets[idx].r;
  const float fl_ref = kAeTargets[idx].fl_ref;
  if (fl_pix_ <= 0.f && cam->ci) {
    fl_pix_ = kAeTargets[idx].focal_mm / cam->ci->pixel_size_mm / std::max(1, cam->ci->out_scale);
  }
  const int W = (cam->buf.rgb_width > 1) ? (int)cam->buf.rgb_width : 1920;
  const int H = (cam->buf.rgb_height > 1) ? (int)cam->buf.rgb_height : 1200;
  const int h_ref = 1208;
  ae_xywh_ = (Rect){
    std::max(0, W / 2 - (int)(fl_pix_ / fl_ref * xywh_ref.w / 2)),
    std::max(0, H / 2 - (int)(fl_pix_ / fl_ref * (h_ref / 2 - xywh_ref.y))),
    std::min((int)(fl_pix_ / fl_ref * xywh_ref.w), W / 2 + (int)(fl_pix_ / fl_ref * xywh_ref.w / 2)),
    std::min((int)(fl_pix_ / fl_ref * xywh_ref.h), H / 2 + (int)(fl_pix_ / fl_ref * (h_ref / 2 - xywh_ref.y))),
  };
}

bool Ka2CameraBackend::seed_external_exposure(CameraState *cam) {
  if (!cam || !cam->ci || !rk_isp_) return false;

  std::array<i2c_random_wr_payload, 10> regs{};
  const int count = cam->ci->getExposureRegisters(
      exposure_time_, gain_idx_, dc_gain_enabled_, regs.data(), regs.size());
  if (count <= 0) return false;

  uint32_t sensor_gain_code = 0;
  bool gain_code_high = false;
  bool gain_code_low = false;
  for (int i = 0; i < count; ++i) {
    if (regs[i].reg_addr == 0x3508) {
      sensor_gain_code = regs[i].reg_data << 8;
      gain_code_high = true;
    } else if (regs[i].reg_addr == 0x3509) {
      sensor_gain_code |= regs[i].reg_data;
      gain_code_low = true;
    }
  }
  if (!gain_code_high || !gain_code_low) return false;

  return rk_isp_->set_external_exposure(
      0, exposure_time_, analog_gain_frac_ * get_gain_factor(cam),
      sensor_gain_code, dc_gain_enabled_);
}

bool Ka2CameraBackend::commit_exposure(CameraState *cam, int exp_t, int gidx, bool hcg) {
  if (!cam || !cam->ci || !set_frame_length_vts(cam, exp_t)) return false;

  // Reuse the command buffers across AE updates. The register order remains
  // VTS first, followed by the sensor's documented exposure sequence.
  exposure_regs_[0] = {0x380e, (uint32_t)(frame_length_vts_ >> 8)};
  exposure_regs_[1] = {0x380f, (uint32_t)(frame_length_vts_ & 0xff)};
  const int sensor_count = cam->ci->getExposureRegisters(
      exp_t, gidx, hcg, exposure_regs_.data() + 2, (int)exposure_regs_.size() - 2);
  if (sensor_count <= 0) return false;
  exposure_reg_count_ = (size_t)sensor_count + 2;

  uint32_t sensor_gain_code = 0;
  bool gain_code_high = false;
  bool gain_code_low = false;
  for (size_t i = 2; i < exposure_reg_count_; ++i) {
    if (exposure_regs_[i].reg_addr == 0x3508) {
      sensor_gain_code = exposure_regs_[i].reg_data << 8;
      gain_code_high = true;
    } else if (exposure_regs_[i].reg_addr == 0x3509) {
      sensor_gain_code |= exposure_regs_[i].reg_data;
      gain_code_low = true;
    }
  }

  const bool changed = exposure_reg_count_ != last_exp_reg_count_ ||
      !std::equal(exposure_regs_.begin(), exposure_regs_.begin() + exposure_reg_count_, last_exp_regs_.begin(),
                  [](const i2c_random_wr_payload &a, const i2c_random_wr_payload &b) {
                    return a.reg_addr == b.reg_addr && a.reg_data == b.reg_data;
                  });
  if (changed && !sensors_i2c(cam, exposure_regs_.data(), (int)exposure_reg_count_)) return false;

  new_exp_g_ = gidx;
  new_exp_t_ = exp_t;
  gain_idx_ = gidx;
  exposure_time_ = exp_t;
  dc_gain_enabled_ = hcg;
  analog_gain_frac_ = cam->ci->sensor_analog_gains[gidx];
  if (changed) {
    std::copy_n(exposure_regs_.begin(), exposure_reg_count_, last_exp_regs_.begin());
    last_exp_reg_count_ = exposure_reg_count_;
  }

  const float gain = analog_gain_frac_ * get_gain_factor(cam);
  cur_ev_[cam->frame_id_last % 3] = exposure_time_ * gain;
  if (rk_isp_ && gain_code_high && gain_code_low) {
    rk_isp_->set_external_exposure(cam->frame_id_last + 1, exposure_time_, gain,
                                   sensor_gain_code, hcg);
  }
  return true;
}

void Ka2CameraBackend::set_camera_exposure(CameraState *cam, float grey_frac) {
  if (!cam->enabled || !cam->ci) return;
  std::lock_guard lk(exp_lock_);

  static const float target_grey_minimums[3] = {0.1f, 0.1f, 0.125f};
  const float tg_min = target_grey_minimums[std::clamp(cam->camera_num, 0, 2)];

  const float dt = 0.05f;
  const float ts_grey = 10.0f;
  const float ts_ev = 0.05f;
  const float k_grey = (dt / ts_grey) / (1.0f + dt / ts_grey);
  const float k_ev = (dt / ts_ev) / (1.0f + dt / ts_ev);

  const uint32_t previous_frame = cam->frame_id_last == 0 ? 0 : cam->frame_id_last - 1;
  const float cur_ev_scaled = cur_ev_[previous_frame % 3] * cam->ci->ev_scale;
  float new_target_grey = std::clamp(
      0.4f - 0.3f * log2f(1.0f + cam->ci->target_grey_factor * cur_ev_scaled) / log2f(6000.0f),
      tg_min, 0.4f);
  float target_grey = (1.0f - k_grey) * target_grey_fraction_ + k_grey * new_target_grey;

  grey_frac = std::max(grey_frac, 1e-4f);
  float desired_ev = std::clamp(cur_ev_scaled / cam->ci->ev_scale * target_grey / grey_frac, cam->ci->min_ev, cam->ci->max_ev);
  float k = (1.0f - k_ev) / 3.0f;
  desired_ev = (k * cur_ev_[0]) + (k * cur_ev_[1]) + (k * cur_ev_[2]) + (k_ev * desired_ev);

  best_ev_score_ = 1e6f;
  new_exp_g_ = gain_idx_;
  new_exp_t_ = exposure_time_;

  bool enable_dc_gain = dc_gain_enabled_;
  if (!enable_dc_gain && target_grey < cam->ci->dc_gain_on_grey) {
    enable_dc_gain = true;
    dc_gain_weight_ = cam->ci->dc_gain_min_weight;
  } else if (enable_dc_gain && target_grey > cam->ci->dc_gain_off_grey) {
    enable_dc_gain = false;
    dc_gain_weight_ = cam->ci->dc_gain_max_weight;
  }
  if (enable_dc_gain && dc_gain_weight_ < cam->ci->dc_gain_max_weight) dc_gain_weight_ += 1;
  if (!enable_dc_gain && dc_gain_weight_ > cam->ci->dc_gain_min_weight) dc_gain_weight_ -= 1;

  // Match comma's bounded gain ramp: only evaluate one gain step on either
  // side of the current index per physical frame. The sensor's hard maximum
  // remains kMaxAnalogGainIdx (15.0x analog gain).
  int selected_gain = std::clamp(gain_idx_, cam->ci->analog_gain_min_idx, cam->ci->analog_gain_max_idx);
  const float dc_gain = enable_dc_gain
      ? (1.0f + dc_gain_weight_ * (cam->ci->dc_gain_factor - 1.0f) /
         std::max(1, cam->ci->dc_gain_max_weight))
      : 1.0f;
  const int min_gain = std::max(selected_gain - 1, cam->ci->analog_gain_min_idx);
  const int max_gain = std::min(selected_gain + 1, cam->ci->analog_gain_max_idx);
  for (int candidate_gain = min_gain; candidate_gain <= max_gain; candidate_gain++) {
    const float candidate_total_gain = cam->ci->sensor_analog_gains[candidate_gain] * dc_gain;
    const int candidate_exposure = std::clamp(
        (int)std::lround(desired_ev / candidate_total_gain),
        cam->ci->exposure_time_min, cam->ci->exposure_time_max);
    // Preserve comma's preference for the recommended gain in bright scenes.
    if (candidate_gain < cam->ci->analog_gain_rec_idx && candidate_exposure > 20 &&
        candidate_gain < selected_gain) {
      continue;
    }
    const float score = cam->ci->getExposureScore(
        desired_ev, candidate_exposure, candidate_gain, candidate_total_gain, selected_gain);
    if (score < best_ev_score_) {
      best_ev_score_ = score;
      new_exp_g_ = candidate_gain;
      new_exp_t_ = candidate_exposure;
    }
  }

  if (!commit_exposure(cam, new_exp_t_, new_exp_g_, enable_dc_gain)) return;

  measured_grey_fraction_ = grey_frac;
  target_grey_fraction_ = target_grey;
}

void Ka2CameraBackend::on_dequeue(CameraState *cam, FrameMetadata &md, int buf_idx) {
  md.integ_lines = exposure_time_;
  md.gain = analog_gain_frac_ * get_gain_factor(cam);
  md.high_conversion_gain = dc_gain_enabled_;
  md.sensor_temp_c = last_sensor_temp_c_;

  if (md.frame_id % kTemperaturePollPeriod == 0) {
    int temp_raw = 0;
    if (read_ctrl(cam, V4L2_CID_X3C_SENSOR_TEMPERATURE, &temp_raw)) {
      last_sensor_temp_c_ = temp_raw / 100.0f;
      md.sensor_temp_c = last_sensor_temp_c_;
    }
  }

  cam->frame_id_last = md.frame_id;

  const uint8_t *y = reinterpret_cast<const uint8_t *>(cam->buf.camera_bufs[buf_idx].addr);
  const int w = cam->buf.rgb_width;
  const int h = cam->buf.rgb_height;
  if (y && w > 0 && h > 0 && cam->ci) {
    if (!ae_roi_ready_ || ae_xywh_.w < 1000) {
      set_exposure_rect(cam);
      ae_roi_ready_ = true;
    }
    const int y_skip = (cam->camera_num == 2) ? 4 : 2;
    float grey = calculate_exposure_value(y, cam->buf.camera_bufs[buf_idx].stride, ae_xywh_, 2, y_skip);
    set_camera_exposure(cam, grey);
  }
  if (md.frame_id % 600 == 1) apply_pwl_on(cam);

  md.integ_lines = exposure_time_;
  md.gain = analog_gain_frac_ * get_gain_factor(cam);
  md.high_conversion_gain = dc_gain_enabled_;
  md.measured_grey_fraction = measured_grey_fraction_;
  md.target_grey_fraction = target_grey_fraction_;
}

void Ka2CameraBackend::apply_flips(CameraState *cam) {
  if (!cam || cam->ctrl_fd < 0) return;
  if (!write_ctrl(cam, V4L2_CID_HFLIP, 0)) {
    LOGE("camera %d: HFLIP=0 failed errno=%d '%s'", cam->camera_num, errno, strerror(errno));
  }
  if (!write_ctrl(cam, V4L2_CID_VFLIP, 1)) {
    LOGE("camera %d: VFLIP=1 failed errno=%d '%s'", cam->camera_num, errno, strerror(errno));
  }
}

bool Ka2CameraBackend::prepare_system(MultiCameraState *s) {
  if (!RkIspUserspaceController::calibration_available()) {
    LOGE("KA2: typed calibration profiles are unavailable; refusing camera startup");
    return false;
  }
  const int n = (kEnableWideRoad ? 1 : 0) + (kEnableRoad ? 1 : 0) + (kEnableDriver ? 1 : 0);
  RkIspUserspaceController::set_multi_cam_count(n);
  s->wide_road_cam.camera_open(0, kEnableWideRoad);
  s->road_cam.camera_open(1, kEnableRoad);
  s->driver_cam.camera_open(2, kEnableDriver);
  return (!kEnableWideRoad || s->wide_road_cam.enabled) &&
         (!kEnableRoad || s->road_cam.enabled) &&
         (!kEnableDriver || s->driver_cam.enabled);
}

void Ka2CameraBackend::prepare_isp_all(MultiCameraState *s) {
  CameraState *cams[3] = {&s->wide_road_cam, &s->road_cam, &s->driver_cam};
  for (int i = 0; i < 3; i++) {
    if (!cams[i]->enabled || !cams[i]->ka2 || !cams[i]->ka2->isp() || !cams[i]->ka2->isp()->active()) continue;
    if (!cams[i]->ka2->isp()->prepare()) {
      LOGE("camera %d: CamHw prepare failed", cams[i]->camera_num);
    }
  }
}

void Ka2CameraBackend::synced_stream_and_start(MultiCameraState *s) {
  CameraState *cams[3] = {&s->wide_road_cam, &s->road_cam, &s->driver_cam};

  for (int i = 0; i < 3; i++) {
    if (!cams[i]->enabled || !cams[i]->ka2 || !cams[i]->ka2->isp() || !cams[i]->ka2->isp()->active()) continue;
    if (!cams[i]->ka2->isp()->start()) {
      LOGE("camera %d: CamHw start failed", cams[i]->camera_num);
    }
  }

  for (int i = 0; i < 3; i++) {
    if (!cams[i]->enabled) continue;
    cams[i]->queue_all_buffers();
    if (ioctl(cams[i]->video_fd, VIDIOC_STREAMON, &cams[i]->fmt.type) < 0) {
      LOGE("camera %d: VIDIOC_STREAMON failed errno=%d '%s'", cams[i]->camera_num, errno, strerror(errno));
      cams[i]->enabled = false;
      continue;
    }
    if (cams[i]->ka2) {
      cams[i]->ka2->on_stream_start(cams[i]);
      cams[i]->ka2->apply_flips(cams[i]);
    }
  }
}
