#include "system/camerad/isp/camera_backend.h"

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
#include "system/camerad/camera/rk.h"
#include "system/camerad/isp/pwl_regs.h"
#include "third_party/linux/include/v4l2-controls.h"

namespace {

constexpr uint16_t kGrpHoldReg = 0x3208;
constexpr int kI2cChunk = 8;
constexpr int kI2cRetries = 5;
constexpr uint32_t kTemperaturePollPeriod = 8;
constexpr uint8_t kAeReversalConfirmFrames = 4;
constexpr uint8_t kAeReversalHoldFrames = 10;

int8_t ae_direction(int value) {
  return value > 0 ? 1 : value < 0 ? -1 : 0;
}

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

void update_max(std::atomic<uint64_t> &metric, uint64_t value) {
  uint64_t previous = metric.load(std::memory_order_relaxed);
  while (previous < value &&
         !metric.compare_exchange_weak(previous, value, std::memory_order_relaxed)) {
  }
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
  camera_ = cam;
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
  last_exposure_direction_ = 0;
  last_gain_direction_ = 0;
  pending_exposure_direction_ = 0;
  pending_gain_direction_ = 0;
  pending_reversal_frames_ = 0;
  reversal_hold_frames_ = 0;
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
  cfg.ae_window.h_offs = static_cast<uint16_t>(std::clamp(ae_xywh_.x, 0, 65535));
  cfg.ae_window.v_offs = static_cast<uint16_t>(std::clamp(ae_xywh_.y, 0, 65535));
  cfg.ae_window.h_size = static_cast<uint16_t>(std::clamp(ae_xywh_.w, 1, 65535));
  cfg.ae_window.v_size = static_cast<uint16_t>(std::clamp(ae_xywh_.h, 1, 65535));
  cfg.ae_owner = this;
  cfg.ae_run = [](void *owner, const rk_aiq_customAe_stats_t *stats,
                  rk_aiq_customeAe_results_t *result) {
    return static_cast<Ka2CameraBackend *>(owner)->run_rkaiq_ae(stats, result);
  };
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
  stop_ae_worker();
  if (rk_isp_) {
    rk_isp_->shutdown();
    rk_isp_.reset();
  }
  if (i2c_fd_.fd_ >= 0) {
    ::close(i2c_fd_.fd_);
    i2c_fd_.fd_ = -1;
  }
  camera_ = nullptr;
}

void Ka2CameraBackend::on_stream_start(CameraState *cam) {
  // One bounded writer serves both modes and provides a fallback until the
  // first RKAIQ histogram arrives.
  start_ae_worker();
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
  const uint64_t i2c_start_ns = monotonic_time_ns();
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
  const uint64_t i2c_duration_ns = monotonic_time_ns() - i2c_start_ns;
  ae_i2c_last_ns_.store(i2c_duration_ns, std::memory_order_relaxed);
  update_max(ae_i2c_max_ns_, i2c_duration_ns);
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

  // Do not rely on the kernel's cached controls after a restart.  In HDR4,
  // VTS is a sensor-internal timing register and can remain at an old value
  // even when the V4L2 VBLANK control reports its nominal value.  Commit the
  // complete 20 FPS packet first; commit_exposure also publishes the matching
  // metadata to RKAIQ.
  last_exp_reg_count_ = 0;
  return commit_exposure(cam, exposure_time_, gain_idx_, dc_gain_enabled_);
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
  if (changed && !sensors_i2c(cam, exposure_regs_.data(), (int)exposure_reg_count_)) {
    return false;
  }

  new_exp_g_ = gidx;
  new_exp_t_ = exp_t;
  gain_idx_ = gidx;
  exposure_time_ = exp_t;
  dc_gain_enabled_ = hcg;
  analog_gain_frac_ = cam->ci->sensor_analog_gains[gidx];
  published_exposure_time_.store(exposure_time_, std::memory_order_relaxed);
  published_gain_.store(analog_gain_frac_ * get_gain_factor(cam), std::memory_order_relaxed);
  published_hcg_.store(dc_gain_enabled_, std::memory_order_relaxed);
  if (changed) {
    std::copy_n(exposure_regs_.begin(), exposure_reg_count_, last_exp_regs_.begin());
    last_exp_reg_count_ = exposure_reg_count_;
  }

  const float gain = analog_gain_frac_ * get_gain_factor(cam);
  cur_ev_[cam->frame_id_last % 3] = exposure_time_ * gain;
  if (rk_isp_ && gain_code_high && gain_code_low) {
    const uint64_t isp_start_ns = monotonic_time_ns();
    rk_isp_->set_external_exposure(cam->frame_id_last + 1, exposure_time_, gain,
                                   sensor_gain_code, hcg);
    const uint64_t isp_duration_ns = monotonic_time_ns() - isp_start_ns;
    ae_isp_last_ns_.store(isp_duration_ns, std::memory_order_relaxed);
    update_max(ae_isp_max_ns_, isp_duration_ns);
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

  // Match reference's bounded gain ramp: only evaluate one gain step on either
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
    // Preserve reference's preference for the recommended gain in bright scenes.
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

  const int exposure_delta = std::abs(new_exp_t_ - exposure_time_);
  const bool significant_change =
      last_exp_reg_count_ == 0 || new_exp_g_ != gain_idx_ || enable_dc_gain != dc_gain_enabled_ ||
      exposure_delta > std::max(2, (int)std::lround(exposure_time_ * 0.02f));

  // Histogram quantization can make the optimizer cross the same boundary in
  // opposite directions on adjacent frames. Confirm reversals and hold them
  // briefly; this is constant-memory state and adds no image-processing work.
  const int8_t exposure_direction = ae_direction(new_exp_t_ - exposure_time_);
  const int8_t gain_direction = ae_direction(new_exp_g_ - gain_idx_);
  const bool exposure_reversal = exposure_direction != 0 &&
                                 last_exposure_direction_ != 0 &&
                                 exposure_direction != last_exposure_direction_;
  const bool gain_reversal = gain_direction != 0 &&
                             last_gain_direction_ != 0 &&
                             gain_direction != last_gain_direction_;
  const bool reversing = exposure_reversal || gain_reversal;
  const float current_ev = cur_ev_scaled / cam->ci->ev_scale;
  const bool severe_grey_error = grey_frac < target_grey * 0.5f ||
                                 grey_frac > target_grey * 1.5f;
  const bool at_low_exposure_limit =
      exposure_time_ <= cam->ci->exposure_time_min + 2 && desired_ev < current_ev;
  const bool at_high_exposure_limit =
      exposure_time_ >= cam->ci->exposure_time_max - 2 && desired_ev > current_ev;
  const bool reversal_emergency = severe_grey_error || at_low_exposure_limit ||
                                  at_high_exposure_limit;

  bool allow_change = significant_change;
  bool reversal_accepted = false;
  if (!significant_change) {
    pending_exposure_direction_ = 0;
    pending_gain_direction_ = 0;
    pending_reversal_frames_ = 0;
    if (reversal_hold_frames_ > 0) --reversal_hold_frames_;
  } else if (!reversing) {
    pending_exposure_direction_ = 0;
    pending_gain_direction_ = 0;
    pending_reversal_frames_ = 0;
    if (reversal_hold_frames_ > 0) --reversal_hold_frames_;
  } else if (reversal_emergency) {
    pending_exposure_direction_ = 0;
    pending_gain_direction_ = 0;
    pending_reversal_frames_ = 0;
    reversal_accepted = true;
  } else if (reversal_hold_frames_ > 0) {
    --reversal_hold_frames_;
    pending_exposure_direction_ = 0;
    pending_gain_direction_ = 0;
    pending_reversal_frames_ = 0;
    allow_change = false;
  } else {
    if (pending_exposure_direction_ != exposure_direction ||
        pending_gain_direction_ != gain_direction) {
      pending_exposure_direction_ = exposure_direction;
      pending_gain_direction_ = gain_direction;
      pending_reversal_frames_ = 0;
    }
    if (pending_reversal_frames_ < kAeReversalConfirmFrames) {
      ++pending_reversal_frames_;
    }
    if (pending_reversal_frames_ < kAeReversalConfirmFrames) {
      allow_change = false;
    } else {
      reversal_accepted = true;
    }
  }

  if (!allow_change) {
    measured_grey_fraction_ = grey_frac;
    target_grey_fraction_ = target_grey;
    published_measured_grey_.store(measured_grey_fraction_, std::memory_order_relaxed);
    published_target_grey_.store(target_grey_fraction_, std::memory_order_relaxed);
    ae_exposure_skips_.fetch_add(1, std::memory_order_relaxed);
    if (reversing) ae_reversal_skips_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  if (!commit_exposure(cam, new_exp_t_, new_exp_g_, enable_dc_gain)) return;

  if (exposure_direction != 0) last_exposure_direction_ = exposure_direction;
  if (gain_direction != 0) last_gain_direction_ = gain_direction;
  if (reversal_accepted) {
    pending_exposure_direction_ = 0;
    pending_gain_direction_ = 0;
    pending_reversal_frames_ = 0;
    reversal_hold_frames_ = kAeReversalHoldFrames;
  }

  measured_grey_fraction_ = grey_frac;
  target_grey_fraction_ = target_grey;
  published_measured_grey_.store(measured_grey_fraction_, std::memory_order_relaxed);
  published_target_grey_.store(target_grey_fraction_, std::memory_order_relaxed);
}

int32_t Ka2CameraBackend::run_rkaiq_ae(const rk_aiq_customAe_stats_t *stats,
                                       rk_aiq_customeAe_results_t *result) {
  if (!result || !camera_ || !camera_->enabled || !camera_->ci) return 0;

  // Camerad owns the sensor packet. A valid empty result keeps SensorHw in
  // explicit-I2C mode without invoking the unsupported kernel register ioctl.
  result->exp_i2c_params = {};
  result->exp_i2c_params.bValid = true;
  result->frame_length_lines = ox03c10_limits::kActiveRows;
  if (stats) {
    result->linear_exp = stats->linear_exp;
    for (size_t i = 0; i < std::size(result->hdr_exp); i++) {
      result->hdr_exp[i] = stats->hdr_exp[i];
    }
  }
  result->is_longfrm_mode = false;

  // The translator has already converted the ISP30 histogram to 256 bins in
  // the same 8-bit luma domain used by the legacy policy. This camera is
  // prepared in normal (non-HDR) mode, so only rawae_stat[0] is valid. The
  // other entries are HDR short/medium/long paths and must not be combined
  // with the normal-path histogram.
  std::array<uint64_t, 256> histogram = {};
  uint64_t total = 0;
  if (stats) {
    const auto &channel = stats->rawae_stat[0];
    for (size_t i = 0; i < histogram.size(); i++) {
      histogram[i] = channel.rawhist_big.bins[i];
      total += histogram[i];
    }
  }
  if (total == 0) {
    return 0;
  }

  const uint64_t midpoint = (total + 1) / 2;
  uint64_t cumulative = 0;
  size_t median_bin = 0;
  for (; median_bin < histogram.size(); median_bin++) {
    cumulative += histogram[median_bin];
    if (cumulative >= midpoint) break;
  }

  // Use half the measured road NV12/raw transfer scale as the shared camera
  // benchmark. Keep the value in the AE grey domain before consumption.
  constexpr float kRkaiqGreyScale = 18.0f;
  const float raw_grey = (static_cast<float>(median_bin) + 0.5f) / 256.0f;
  const float scaled_grey = std::clamp(raw_grey * kRkaiqGreyScale, 0.0f, 1.0f);
  rkaiq_pending_grey_.store(scaled_grey, std::memory_order_relaxed);
  rkaiq_ae_callback_seq_.fetch_add(1, std::memory_order_release);
  return 0;
}

void Ka2CameraBackend::on_dequeue(CameraState *cam, FrameMetadata &md) {
  md.integ_lines = published_exposure_time_.load(std::memory_order_relaxed);
  md.gain = published_gain_.load(std::memory_order_relaxed);
  md.high_conversion_gain = published_hcg_.load(std::memory_order_relaxed);
  md.sensor_temp_c = published_sensor_temp_c_.load(std::memory_order_relaxed);

  cam->frame_id_last = md.frame_id;
  md.measured_grey_fraction = published_measured_grey_.load(std::memory_order_relaxed);
  md.target_grey_fraction = published_target_grey_.load(std::memory_order_relaxed);
}

void Ka2CameraBackend::start_ae_worker() {
  std::lock_guard lk(ae_mtx_);
  if (ae_running_) return;
  ae_running_ = true;
  ae_thread_ = std::thread(&Ka2CameraBackend::ae_worker_loop, this);
}

void Ka2CameraBackend::stop_ae_worker() {
  {
    std::lock_guard lk(ae_mtx_);
    if (!ae_running_ && !ae_thread_.joinable()) return;
    ae_running_ = false;
  }
  ae_cv_.notify_one();
  ae_done_cv_.notify_all();
  if (ae_thread_.joinable()) ae_thread_.join();

  std::lock_guard lk(ae_mtx_);
  ae_jobs_.clear();
  ae_pending_.fill(false);
  ae_done_cv_.notify_all();
}

void Ka2CameraBackend::enqueue_ae(CameraState *cam, int buf_idx, const FrameMetadata &md) {
  if (!cam || buf_idx < 0 || buf_idx >= 4) return;

  {
    std::lock_guard lk(ae_mtx_);
    if (!ae_running_ || ae_pending_[buf_idx]) return;

    // Keep the AE backlog bounded. Dropping an AE job does not release its
    // V4L2 buffer; the processing thread still owns and releases that buffer.
    if (ae_jobs_.size() >= kAeQueueDepth) {
      const int dropped_idx = ae_jobs_.front().buf_idx;
      ae_jobs_.pop_front();
      ae_pending_[dropped_idx] = false;
      ae_dropped_jobs_.fetch_add(1, std::memory_order_relaxed);
      ae_done_cv_.notify_all();
    }

    ae_pending_[buf_idx] = true;
    ae_jobs_.push_back({cam, buf_idx, md});
  }
  ae_cv_.notify_one();
}

void Ka2CameraBackend::wait_for_ae(int buf_idx) {
  if (buf_idx < 0 || buf_idx >= 4) return;
  std::unique_lock lk(ae_mtx_);
  ae_done_cv_.wait(lk, [this, buf_idx] {
    return !ae_running_ || !ae_pending_[buf_idx];
  });
}

void Ka2CameraBackend::ae_worker_loop() {
  util::set_thread_name("CameraAE");
  while (true) {
    AeJob job;
    {
      std::unique_lock lk(ae_mtx_);
      ae_cv_.wait(lk, [this] { return !ae_running_ || !ae_jobs_.empty(); });
      if (!ae_running_ && ae_jobs_.empty()) break;
      job = ae_jobs_.front();
      ae_jobs_.pop_front();
    }

    process_ae_job(job);

    {
      std::lock_guard lk(ae_mtx_);
      ae_pending_[job.buf_idx] = false;
      ++ae_completed_jobs_;
    }
    ae_done_cv_.notify_all();

    if (ae_completed_jobs_ % 120 == 0) {
      log_ae_metrics(job.cam, job.md.frame_id);
    }
  }
}

void Ka2CameraBackend::process_ae_job(const AeJob &job) {
  CameraState *cam = job.cam;
  const FrameMetadata &md = job.md;
  ae_frame_id_ = md.frame_id;
  ae_i2c_last_ns_.store(0, std::memory_order_relaxed);
  ae_isp_last_ns_.store(0, std::memory_order_relaxed);

  // Consume only the newest histogram decision. This keeps the physical
  // sensor writer bounded at the capture cadence and avoids stale updates.
  const uint32_t rkaiq_seq = rkaiq_ae_callback_seq_.load(std::memory_order_acquire);
  if (rk_isp_ && rk_isp_->custom_ae_active() && rkaiq_seq != 0) {
    if (rkaiq_seq != rkaiq_ae_consumed_seq_) {
      rkaiq_ae_consumed_seq_ = rkaiq_seq;
      set_camera_exposure(cam, rkaiq_pending_grey_.load(std::memory_order_relaxed));
    }
    return;
  }

  const uint64_t dequeue_to_ae_ns = monotonic_time_ns() - md.dequeue_monotonic_ns;
  ae_dequeue_to_ae_last_ns_.store(dequeue_to_ae_ns, std::memory_order_relaxed);
  update_max(ae_dequeue_to_ae_max_ns_, dequeue_to_ae_ns);

  if (md.frame_id % kTemperaturePollPeriod == 0) {
    int temp_raw = 0;
    if (read_ctrl(cam, V4L2_CID_X3C_SENSOR_TEMPERATURE, &temp_raw)) {
      last_sensor_temp_c_ = temp_raw / 100.0f;
      published_sensor_temp_c_.store(last_sensor_temp_c_, std::memory_order_relaxed);
    }
  }

  const uint8_t *y = reinterpret_cast<const uint8_t *>(cam->buf.camera_bufs[job.buf_idx].addr);
  const int w = cam->buf.rgb_width;
  const int h = cam->buf.rgb_height;
  if (y && w > 0 && h > 0 && cam->ci) {
    if (!ae_roi_ready_ || ae_xywh_.w < 1000) {
      set_exposure_rect(cam);
      ae_roi_ready_ = true;
    }

    RkIspAeShadowStats rkaiq_stats = {};
    const uint64_t rkaiq_start_ns = monotonic_time_ns();
    const bool rkaiq_stats_valid = rk_isp_ && rk_isp_->get_ae_shadow_stats(&rkaiq_stats);
    const uint64_t rkaiq_duration_ns = monotonic_time_ns() - rkaiq_start_ns;
    ae_rkaiq_last_ns_.store(rkaiq_duration_ns, std::memory_order_relaxed);
    update_max(ae_rkaiq_max_ns_, rkaiq_duration_ns);
    if (rkaiq_stats_valid) {
      ae_rkaiq_reads_.fetch_add(1, std::memory_order_relaxed);
    } else {
      ae_rkaiq_failures_.fetch_add(1, std::memory_order_relaxed);
    }

    const int y_skip = (cam->camera_num == 2) ? 4 : 2;
    const uint64_t luma_start_ns = monotonic_time_ns();
    const float grey = calculate_exposure_value(
        y, cam->buf.camera_bufs[job.buf_idx].stride, ae_xywh_, 2, y_skip);
    const uint64_t luma_duration_ns = monotonic_time_ns() - luma_start_ns;
    ae_luma_last_ns_.store(luma_duration_ns, std::memory_order_relaxed);
    update_max(ae_luma_max_ns_, luma_duration_ns);
    set_camera_exposure(cam, grey);

    // Keep this log sparse enough for driving, while retaining frame-aligned
    // samples across changing lighting. The CPU result still controls AE.
    if (rkaiq_stats_valid && md.frame_id % 10 == 0) {
      const int64_t frame_delta = static_cast<int64_t>(rkaiq_stats.frame_id) - static_cast<int64_t>(md.frame_id);
      LOG("camera %d RKAIQ_AE_SHADOW frame=%u stats_frame=%u delta=%lld read_us=%llu "
          "cpu_grey=%.6f hist_med=%u,%u,%u,%u hist_total=%llu,%llu,%llu,%llu "
          "raw_mean=%u,%u,%u,%u integ=%u gain=%.4f hcg=%d",
          cam->camera_num, md.frame_id, rkaiq_stats.frame_id, (long long)frame_delta,
          (unsigned long long)(rkaiq_duration_ns / 1000), grey,
          rkaiq_stats.histogram_median[0], rkaiq_stats.histogram_median[1],
          rkaiq_stats.histogram_median[2], rkaiq_stats.histogram_median[3],
          (unsigned long long)rkaiq_stats.histogram_total[0],
          (unsigned long long)rkaiq_stats.histogram_total[1],
          (unsigned long long)rkaiq_stats.histogram_total[2],
          (unsigned long long)rkaiq_stats.histogram_total[3],
          rkaiq_stats.raw_mean[0], rkaiq_stats.raw_mean[1],
          rkaiq_stats.raw_mean[2], rkaiq_stats.raw_mean[3],
          md.integ_lines, md.gain, md.high_conversion_gain ? 1 : 0);
    }
  }

  if (md.frame_id % 600 == 1) apply_pwl_on(cam);
}

void Ka2CameraBackend::log_ae_metrics(CameraState *cam, uint32_t frame_id) {
  size_t ae_queue_size = 0;
  {
    std::lock_guard lk(ae_mtx_);
    ae_queue_size = ae_jobs_.size();
  }
  LOG("camera %d AE: frame=%u queue=%zu/%zu capture_queue=%zu/3 capture_peak=%zu "
      "capture_dropped=%llu ae_dropped=%llu exposure_skips=%llu reversal_skips=%llu "
      "luma_us=%llu max=%llu i2c_us=%llu max=%llu isp_us=%llu max=%llu "
      "rkaiq_us=%llu max=%llu reads=%llu failures=%llu "
      "dequeue_to_ae_us=%llu max=%llu",
      cam->camera_num, frame_id, ae_queue_size, kAeQueueDepth,
      cam->buf.queue_size(), cam->buf.queue_peak(),
      (unsigned long long)cam->buf.dropped_frames(),
      (unsigned long long)ae_dropped_jobs_.load(std::memory_order_relaxed),
      (unsigned long long)ae_exposure_skips_.load(std::memory_order_relaxed),
      (unsigned long long)ae_reversal_skips_.load(std::memory_order_relaxed),
      (unsigned long long)(ae_luma_last_ns_.load(std::memory_order_relaxed) / 1000),
      (unsigned long long)(ae_luma_max_ns_.load(std::memory_order_relaxed) / 1000),
      (unsigned long long)(ae_i2c_last_ns_.load(std::memory_order_relaxed) / 1000),
      (unsigned long long)(ae_i2c_max_ns_.load(std::memory_order_relaxed) / 1000),
      (unsigned long long)(ae_isp_last_ns_.load(std::memory_order_relaxed) / 1000),
      (unsigned long long)(ae_isp_max_ns_.load(std::memory_order_relaxed) / 1000),
      (unsigned long long)(ae_rkaiq_last_ns_.load(std::memory_order_relaxed) / 1000),
      (unsigned long long)(ae_rkaiq_max_ns_.load(std::memory_order_relaxed) / 1000),
      (unsigned long long)ae_rkaiq_reads_.load(std::memory_order_relaxed),
      (unsigned long long)ae_rkaiq_failures_.load(std::memory_order_relaxed),
      (unsigned long long)(ae_dequeue_to_ae_last_ns_.load(std::memory_order_relaxed) / 1000),
      (unsigned long long)(ae_dequeue_to_ae_max_ns_.load(std::memory_order_relaxed) / 1000));
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
  RkIspUserspaceController::set_multi_cam_count(3);
  s->wide_road_cam.camera_open(0);
  s->road_cam.camera_open(1);
  s->driver_cam.camera_open(2);
  return s->wide_road_cam.enabled && s->road_cam.enabled && s->driver_cam.enabled;
}

void Ka2CameraBackend::prepare_isp_all(MultiCameraState *s) {
  CameraState *cams[3] = {&s->wide_road_cam, &s->road_cam, &s->driver_cam};
  for (int i = 0; i < 3; i++) {
    if (!cams[i]->enabled || !cams[i]->ka2 || !cams[i]->ka2->isp() || !cams[i]->ka2->isp()->active()) continue;
    if (!cams[i]->ka2->isp()->prepare()) {
      LOGE("camera %d: CamHw prepare failed", cams[i]->camera_num);
      continue;
    }
    // RKAIQ prepare applies one stock-AE initialization result before the
    // custom handler is enabled. Reassert the physical HDR4 VTS afterward,
    // while the sensor is still in standby, so that result cannot reduce the
    // sensor's 20 FPS timing.
    if (!cams[i]->ka2->seed_external_exposure(cams[i])) {
      LOGE("camera %d: failed to reassert initial external exposure", cams[i]->camera_num);
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
    if (cams[i]->enabled) cams[i]->queue_all_buffers();
  }

  for (int i = 0; i < 3; i++) {
    if (!cams[i]->enabled) continue;
    // Program PWL while the sensor is powered but still in standby.  Doing a
    // long register burst after STREAMON can block the capture thread and
    // prevent the first frame from reaching VisionIPC.
    if (cams[i]->ka2) cams[i]->ka2->apply_pwl_on(cams[i]);
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
