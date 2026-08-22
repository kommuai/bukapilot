#include "system/camerad/rk/rk_pwl_regs.h"
#include "system/camerad/cameras/camera_rk.h"

#include <poll.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <string>
#include <vector>

#include "media/cam_defs.h"
#include "media/cam_isp.h"
#include "media/cam_isp_ife.h"
#include "media/cam_req_mgr.h"
#include "media/cam_sensor_cmn_header.h"
#include "media/cam_sync.h"
#include "third_party/linux/include/v4l2-controls.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "system/camerad/rk/rk_isp_userspace.h"

// Special defined
#define V4L2_CID_X3C_SENSOR_TEMPERATURE (V4L2_CID_USER_BASE + 0x100)

extern ExitHandler do_exit;

static const bool env_disable_wide_road = (getenv("DISABLE_WIDE_ROAD") != nullptr);
static const bool env_disable_road = (getenv("DISABLE_ROAD") != nullptr);
static const bool env_disable_driver = (getenv("DISABLE_DRIVER") != nullptr);
static const bool env_log_raw_frames = (getenv("LOG_RAW_FRAMES") != nullptr);

static inline bool read_ctrl_fd(int fd, uint32_t id, int *out) {
  struct v4l2_control c = {};
  c.id = id;
  if (ioctl(fd, VIDIOC_G_CTRL, &c) < 0) return false;
  *out = c.value;
  return true;
}

void CameraState::camera_map_bufs(MultiCameraState *s) {
  int exported_count = 0;
  for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
    memset(&v4l_buf, 0, sizeof(v4l_buf));
    v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l_buf.memory = V4L2_MEMORY_MMAP;
    v4l_buf.index = i;
    v4l_buf.length = 1; //FMT_NUM_PLANES
    v4l_buf.m.planes = planes;

    assert(ioctl(video_fd, VIDIOC_QUERYBUF, &v4l_buf) >= 0);

    buf.camera_bufs[i].mmap_len = v4l_buf.m.planes[0].length;
    buf.camera_bufs[i].len = v4l_buf.m.planes[0].length;
    buf.camera_bufs[i].fd = -1;
    buf.camera_bufs[i].addr = mmap(NULL, v4l_buf.m.planes[0].length,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED,
                                  video_fd, v4l_buf.m.planes[0].m.mem_offset);
    assert(buf.camera_bufs[i].addr != MAP_FAILED);
    buf.camera_bufs[i].init_yuv(buf.rgb_width, buf.rgb_height, buf.rgb_width, (size_t)buf.rgb_width * buf.rgb_height);

    if (rk_zerocopy_requested) {
      struct v4l2_exportbuffer exp = {};
      exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      exp.index = i;
      exp.plane = 0;
      exp.flags = O_CLOEXEC | O_RDWR;
      if (ioctl(video_fd, VIDIOC_EXPBUF, &exp) == 0) {
        buf.camera_bufs[i].fd = exp.fd;
        buf.camera_bufs[i].frame_id_in_buf = false;
        exported_count++;
      } else {
        LOGW("camera %d: VIDIOC_EXPBUF failed idx=%d errno=%d '%s', disabling rk zerocopy",
             camera_num, i, errno, strerror(errno));
      }
    }
  }

  rk_zerocopy_active = rk_zerocopy_requested && (exported_count == FRAME_BUF_COUNT);
  if (!rk_zerocopy_active) {
    for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
      if (buf.camera_bufs[i].fd >= 0) {
        close(buf.camera_bufs[i].fd);
        buf.camera_bufs[i].fd = -1;
      }
      buf.camera_bufs[i].frame_id_in_buf = true;
    }
  } else {
    LOGD("camera %d: rk zerocopy enabled with %d exported buffers", camera_num, exported_count);
  }
}

void CameraState::camera_init(MultiCameraState *s, VisionIpcServer * v, cl_device_id device_id, cl_context ctx, VisionStreamType yuv_type) {
  if (!enabled) return;
  rk_zerocopy_requested = (getenv("CAMERAD_RK_ZEROCOPY") != nullptr);
  rk_zerocopy_active = false;

  LOGD("camera init %d", camera_num);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  fmt.fmt.pix.width = 1920;
  fmt.fmt.pix.height = 1200;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(video_fd, VIDIOC_S_FMT, &fmt) < 0) {
    int err = errno;
    int vfd = video_fd;
    LOGE("camera %d: VIDIOC_S_FMT failed on fd %d (errno=%d '%s'), disabling camera",
         camera_num, vfd, err, strerror(err));
    enabled = false;
    return;
  }

  memset(&req, 0, sizeof(req));
  req.count = FRAME_BUF_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  req.memory = V4L2_MEMORY_MMAP;
  assert(ioctl(video_fd, VIDIOC_REQBUFS, &req) >= 0);

  buf.init(device_id, ctx, this, v, FRAME_BUF_COUNT, yuv_type);
  camera_map_bufs(s);
  buf.setupVipcBuffers(rk_zerocopy_active);
}

void CameraState::camera_open(MultiCameraState *multi_cam_state_, int camera_num_, bool enabled_) {
  multi_cam_state = multi_cam_state_;
  camera_num = camera_num_;
  enabled = enabled_;
  if (!enabled) return;

  LOG("-- Setting camera ctrls");
  char device[32];

  // ctrl is at subdev 2,7,12
  snprintf(device, sizeof(device), "/dev/v4l-subdev%d", camera_num * 5 + 2);
  ctrl_fd = open(device, O_RDWR);
  assert(ctrl_fd >= 0);

  apply_sensor_flips();

  // Seed sensor temp once so early frames are not -999.
  {
    int temp_raw = 0;
    if (read_ctrl_fd(ctrl_fd, V4L2_CID_X3C_SENSOR_TEMPERATURE, &temp_raw)) {
      last_sensor_temp_c = temp_raw / 100.0f;
      LOGW("camera %d: sensor temp seed %.1fC", camera_num, last_sensor_temp_c);
    } else {
      LOGW("camera %d: sensor temp seed failed (will stay -999 until first good read)", camera_num);
    }
  }

  // Sensor + AE init (comma ox03c10 recipe)
  ci = std::make_unique<OX03C10>();
  // Map camera_num → i2c bus (subdev 2/7/12 → i2c-1/3/6)
  static const int kI2cBusByCam[3] = {1, 3, 6};
  i2c_bus = kI2cBusByCam[std::clamp(camera_num, 0, 2)];
  i2c_addr = 0x36;
  dc_gain_weight = ci->dc_gain_min_weight;
  gain_idx = ci->analog_gain_rec_idx;
  exposure_time = 5;
  dc_gain_enabled = false;  // comma: hysteresis until dark
  analog_gain_frac = ci->sensor_analog_gains[gain_idx];
  {
    float g0 = get_gain_factor() * analog_gain_frac * exposure_time;
    cur_ev[0] = cur_ev[1] = cur_ev[2] = g0;
  }
  // Center ROI ~50% of frame (comma uses focal-length scaled box; same intent)
  {
    const int W = 1920, H = 1200;
    ae_xywh = (Rect){W / 4, H / 4, W / 2, H / 2};
  }

  video_fd = open_v4l_by_name_and_index("rkisp_mainpath", camera_num);
  assert(video_fd >= 0);

  {
    rk_isp = std::make_unique<RkIspUserspaceController>();
    RkIspCamConfig cfg;
    cfg.camera_num = camera_num;
    cfg.sensor_ctrl_fd = ctrl_fd;
    // Resolve mainpath node path for librkaiq CamHw bind.
    {
      int n = 0;
      for (int i = 0; i < 64; i++) {
        char syspath[64], name[128];
        snprintf(syspath, sizeof(syspath), "/sys/class/video4linux/video%d/name", i);
        FILE *f = fopen(syspath, "r");
        if (!f) continue;
        if (!fgets(name, sizeof(name), f)) { fclose(f); continue; }
        fclose(f);
        name[strcspn(name, "\n")] = 0;
        if (strcmp(name, "rkisp_mainpath") != 0) continue;
        if (n++ != camera_num) continue;
        char path[32];
        snprintf(path, sizeof(path), "/dev/video%d", i);
        cfg.mainpath_dev = path;
        break;
      }
    }
    if (!rk_isp->init(cfg)) {
      LOGE("camera %d: RkIspUserspace CamHw init failed", camera_num);
      rk_isp->shutdown();
      rk_isp.reset();
    } else {
      rk_isp->enable_frame_ae(false);  // AE via sensors_i2c / set_camera_exposure
      LOGW("camera %d: choice-2 CamHw in-process + comma AE/I2C (no rkaiq_3A)", camera_num);
    }
  }
}

void CameraState::stream_start() {
  if (!enabled) return;
  // start v4l2 buffer queue
  LOG("-- Start Queueing V4L2 buffers");
  for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
    memset(&v4l_buf, 0, sizeof(v4l_buf));
    memset(planes, 0, sizeof(planes));
    v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l_buf.memory = V4L2_MEMORY_MMAP;
    v4l_buf.length = 1;
    v4l_buf.m.planes = planes;
    v4l_buf.index = i;
    if (ioctl(video_fd, VIDIOC_QBUF, &v4l_buf) < 0) {
      LOGE("camera %d: VIDIOC_QBUF failed during stream start idx=%d errno=%d '%s' (mode=%s)",
           camera_num, i, errno, strerror(errno), "mmap");
      enabled = false;
      return;
    }
  }

  // start streaming
  if (ioctl(video_fd, VIDIOC_STREAMON, &fmt.type) < 0) {
    LOGE("camera %d: VIDIOC_STREAMON failed errno=%d '%s'", camera_num, errno, strerror(errno));
    enabled = false;
    return;
  }
  // After stream on: kernel finish_mode may leave PWL inconsistent — force comma PWL-ON.
  apply_pwl_on();
}


float CameraState::get_gain_factor() const {
  if (!ci) return 1.0f;
  return (1.0f + dc_gain_weight * (ci->dc_gain_factor - 1.0f) / std::max(1, ci->dc_gain_max_weight));
}

void CameraState::update_exposure_score(float desired_ev, int exp_t, int exp_g_idx, float exp_gain) {
  if (!ci) return;
  float score = ci->getExposureScore(desired_ev, exp_t, exp_g_idx, exp_gain, gain_idx);
  if (score < best_ev_score) {
    new_exp_t = exp_t;
    new_exp_g = exp_g_idx;
    best_ev_score = score;
  }
}

void CameraState::apply_pwl_on() {
  // Kernel module init can leave PWL inconsistent across cams (seen: road 0x5000=0).
  // Force comma PWL-ON so short-exposure data is packed as Spectra expects.
  std::vector<i2c_random_wr_payload> wr;
  wr.reserve(rk_pwl::kOx03c10PwlOnLen);
  for (size_t i = 0; i < rk_pwl::kOx03c10PwlOnLen; i++) {
    wr.push_back({rk_pwl::kOx03c10PwlOn[i].addr, rk_pwl::kOx03c10PwlOn[i].data});
  }
  sensors_i2c(wr.data(), (int)wr.size(), 0, false);
  LOGW("camera %d: applied ox03c10 PWL-ON (%zu regs)", camera_num, wr.size());
}

void CameraState::sensors_i2c(const struct i2c_random_wr_payload *dat, int len, int op_code, bool data_word) {
  (void)op_code;
  (void)data_word;
  if (i2c_bus < 0 || !dat || len <= 0) return;
  std::lock_guard lk(i2c_lock);
  if (i2c_fd.fd_ < 0) {
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", i2c_bus);
    int fd = open(path, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
      if (frame_id_last % 120 == 0) {
        LOGE("camera %d: open %s failed errno=%d", camera_num, path, errno);
      }
      return;
    }
    i2c_fd.fd_ = fd;
  }
  // Batch all register writes into one I2C_RDWR (one open fd, one syscall).
  constexpr int kMax = 64;
  if (len > kMax) len = kMax;
  uint8_t bufs[kMax][3];
  struct i2c_msg msgs[kMax];
  for (int i = 0; i < len; i++) {
    bufs[i][0] = (uint8_t)((dat[i].reg_addr >> 8) & 0xff);
    bufs[i][1] = (uint8_t)(dat[i].reg_addr & 0xff);
    bufs[i][2] = (uint8_t)(dat[i].reg_data & 0xff);
    msgs[i] = {};
    msgs[i].addr = (uint16_t)i2c_addr;
    msgs[i].flags = 0;
    msgs[i].len = 3;
    msgs[i].buf = bufs[i];
  }
  struct i2c_rdwr_ioctl_data ioctl_data = {};
  ioctl_data.msgs = msgs;
  ioctl_data.nmsgs = (__u32)len;
  if (ioctl(i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
    if (frame_id_last % 120 == 0) {
      LOGE("camera %d: i2c batch wr n=%d failed errno=%d", camera_num, len, errno);
    }
    // fd may be wedged; reopen next time
    close(i2c_fd.fd_);
    i2c_fd.fd_ = -1;
  }
}

void CameraState::set_camera_exposure(float grey_frac) {
  if (!enabled || !ci) return;
  std::lock_guard lk(exp_lock);

  // camera_num: 0=wide, 1=road, 2=driver (matches comma target_grey_minimums order)
  static const float target_grey_minimums[3] = {0.1f, 0.1f, 0.125f};
  const float tg_min = target_grey_minimums[std::clamp(camera_num, 0, 2)];

  const float dt = 0.05f;
  const float ts_grey = 10.0f;
  const float ts_ev = 0.05f;
  const float k_grey = (dt / ts_grey) / (1.0f + dt / ts_grey);
  const float k_ev = (dt / ts_ev) / (1.0f + dt / ts_ev);

  const float cur_ev_ = cur_ev[(buf.cur_frame_data.frame_id - 1) % 3] * ci->ev_scale;
  float new_target_grey = std::clamp(
      0.4f - 0.3f * log2f(1.0f + ci->target_grey_factor * cur_ev_) / log2f(6000.0f),
      tg_min, 0.4f);
  float target_grey = (1.0f - k_grey) * target_grey_fraction + k_grey * new_target_grey;

  grey_frac = std::max(grey_frac, 1e-4f);
  float desired_ev = std::clamp(cur_ev_ / ci->ev_scale * target_grey / grey_frac, ci->min_ev, ci->max_ev);
  float k = (1.0f - k_ev) / 3.0f;
  desired_ev = (k * cur_ev[0]) + (k * cur_ev[1]) + (k * cur_ev[2]) + (k_ev * desired_ev);

  best_ev_score = 1e6f;
  new_exp_g = gain_idx;
  new_exp_t = exposure_time;

  bool enable_dc_gain = dc_gain_enabled;
  if (!enable_dc_gain && target_grey < ci->dc_gain_on_grey) {
    enable_dc_gain = true;
    dc_gain_weight = ci->dc_gain_min_weight;
  } else if (enable_dc_gain && target_grey > ci->dc_gain_off_grey) {
    enable_dc_gain = false;
    dc_gain_weight = ci->dc_gain_max_weight;
  }
  if (enable_dc_gain && dc_gain_weight < ci->dc_gain_max_weight) dc_gain_weight += 1;
  if (!enable_dc_gain && dc_gain_weight > ci->dc_gain_min_weight) dc_gain_weight -= 1;

  int min_g = std::max(gain_idx - 1, ci->analog_gain_min_idx);
  int max_g = std::min(gain_idx + 1, ci->analog_gain_max_idx);
  for (int g = min_g; g <= max_g; g++) {
    float gain = ci->sensor_analog_gains[g] * get_gain_factor();
    int t = std::clamp((int)std::lround(desired_ev / gain), ci->exposure_time_min, ci->exposure_time_max);
    if (g < ci->analog_gain_rec_idx && t > 20 && g < gain_idx) continue;
    update_exposure_score(desired_ev, t, g, gain);
  }

  measured_grey_fraction = grey_frac;
  target_grey_fraction = target_grey;
  analog_gain_frac = ci->sensor_analog_gains[new_exp_g];
  gain_idx = new_exp_g;
  exposure_time = new_exp_t;
  dc_gain_enabled = enable_dc_gain;

  float gain = analog_gain_frac * get_gain_factor();
  cur_ev[buf.cur_frame_data.frame_id % 3] = exposure_time * gain;

  auto exp_reg_array = ci->getExposureRegisters(exposure_time, new_exp_g, dc_gain_enabled);
  // When exposure is floored but frame still too bright, cut HCG digital gain
  // (0x350a/0x350b). rkisp NV12 runs hotter than Spectra IFE at min EV.
  static float dig_cut_margin = -1.f;
  if (dig_cut_margin < 0.f) {
    const char *e = getenv("KA2_AE_DIG_CUT_MARGIN");
    dig_cut_margin = (e && e[0]) ? strtof(e, nullptr) : 1.02f;  // was 1.15
    LOGW("camera %d: AE dig-cut margin=%.3f", camera_num, dig_cut_margin);
  }
  const bool at_exp_floor = exposure_time <= std::max(ci->exposure_time_min, 2);
  const bool too_bright = grey_frac > target_grey * dig_cut_margin;
  if (at_exp_floor && too_bright) {
    exp_reg_array.push_back({0x350a, 0x00});
    exp_reg_array.push_back({0x350b, 0x40});  // ~0.25x digital
    if (buf.cur_frame_data.frame_id % 120 == 0) {
      LOGW("camera %d AE: dig-gain 0.25x (grey=%.3f target=%.3f integ=%d)",
           camera_num, grey_frac, target_grey, exposure_time);
    }
  } else {
    exp_reg_array.push_back({0x350a, 0x01});
    exp_reg_array.push_back({0x350b, 0x00});  // 1.0x
  }
  // Skip bus traffic when HDR register set is unchanged (steady outdoor floor).
  const bool regs_changed = last_exp_regs.size() != exp_reg_array.size() ||
      !std::equal(exp_reg_array.begin(), exp_reg_array.end(), last_exp_regs.begin(),
                  [](const i2c_random_wr_payload &a, const i2c_random_wr_payload &b) {
                    return a.reg_addr == b.reg_addr && a.reg_data == b.reg_data;
                  });
  if (regs_changed) {
    sensors_i2c(exp_reg_array.data(), (int)exp_reg_array.size(), 0, ci->data_word);
    last_exp_regs = std::move(exp_reg_array);
  }

  // Do NOT VIDIOC_S_CTRL exposure/gain after I2C — kernel ox03c10 driver
  // rewrites HCG only and fights SPD/VS/LCG from getExposureRegisters.

  if (buf.cur_frame_data.frame_id % 120 == 0) {
    LOGD("camera %d AE: grey=%.4f target=%.3f exp=%d gidx=%d gain=%.3f dc=%d",
         camera_num, grey_frac, target_grey, exposure_time, gain_idx, gain,
         (int)dc_gain_enabled);
  }
}


void CameraState::dequeue_buf() {
  if (!enabled) return;

  memset(&v4l_buf, 0, sizeof(v4l_buf));
  memset(planes, 0, sizeof(planes));
  v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  v4l_buf.memory = V4L2_MEMORY_MMAP;
  v4l_buf.length = 1;
  v4l_buf.m.planes = planes;
  if (ioctl(video_fd, VIDIOC_DQBUF, &v4l_buf) < 0) {
    LOGE("camera %d: VIDIOC_DQBUF failed errno=%d '%s'", camera_num, errno, strerror(errno));
    return;
  }

  const int idx = v4l_buf.index;
  FrameMetadata &md = buf.camera_bufs_metadata[idx];

  // Exposure/gain come from our I2C AE path — skip per-frame V4L2 ctrl readback.
  md.integ_lines = exposure_time;
  md.gain = analog_gain_frac * get_gain_factor();
  md.high_conversion_gain = dc_gain_enabled;
  md.sensor_temp_c = last_sensor_temp_c;

  // Temperature changes slowly; poll occasionally.
  if (v4l_buf.sequence % temp_poll_divider == 0) {
    int temp_raw = 0;
    if (read_ctrl_fd(ctrl_fd, V4L2_CID_X3C_SENSOR_TEMPERATURE, &temp_raw)) {
      last_sensor_temp_c = temp_raw / 100.0f;
      md.sensor_temp_c = last_sensor_temp_c;
    }
  }

  md.frame_id = v4l_buf.sequence;
  md.request_id = v4l_buf.sequence;
  cap_time = static_cast<uint64_t>(v4l_buf.timestamp.tv_sec * 1000000000 + v4l_buf.timestamp.tv_usec * 1000);
  md.timestamp_sof = cap_time;
  md.timestamp_eof = cap_time;

  // AE: coarse mean grey every other frame → I2C only when regs change.
  {
    const uint8_t *y = reinterpret_cast<const uint8_t *>(buf.camera_bufs[idx].addr);
    const int w = buf.rgb_width;
    const int h = buf.rgb_height;
    buf.cur_frame_data = md;
    frame_id_last = md.frame_id;
    if (y && w > 0 && h > 0 && ci && (md.frame_id & 1) == 0) {
      // Comma-parity AE meter: median Y in ROI (calculate_exposure_value), not mean.
      buf.cur_yuv_buf = &buf.camera_bufs[idx];
      buf.out_img_width = (uint32_t)w;
      buf.out_img_height = (uint32_t)h;
      float grey = calculate_exposure_value(&buf, ae_xywh, 2, 2);
      // rkisp NV12 is typically brighter than Spectra IFE for same EV; scale meter
      // so AE drives toward comma-like brightness. Override: KA2_AE_GREY_SCALE.
      static float grey_scale = -1.f;
      if (grey_scale < 0.f) {
        const char *e = getenv("KA2_AE_GREY_SCALE");
        grey_scale = (e && e[0]) ? strtof(e, nullptr) : 1.0f;
        LOGW("camera %d: AE median+scale=%.3f", camera_num, grey_scale);
      }
      grey = std::min(grey * grey_scale, 0.95f);
      set_camera_exposure(grey);
    }
    // Rare PWL refresh (kernel can drift); batched on persistent i2c fd.
    if (rk_isp) rk_isp->maybe_reload_wb_file();
    if (md.frame_id % 600 == 1) apply_pwl_on();
    md.integ_lines = exposure_time;
    md.gain = analog_gain_frac * get_gain_factor();
    md.high_conversion_gain = dc_gain_enabled;
    md.measured_grey_fraction = measured_grey_fraction;
    md.target_grey_fraction = target_grey_fraction;
  }

  buf.queue(idx);

  if (ioctl(video_fd, VIDIOC_QBUF, &v4l_buf) < 0) {
    LOGE("camera %d: VIDIOC_QBUF failed post-dequeue errno=%d '%s'", camera_num, errno, strerror(errno));
    return;
  }
}

void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {

  s->driver_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_DRIVER);
  s->road_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_ROAD);
  s->wide_road_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_WIDE_ROAD);

  s->pm = new PubMaster({"roadCameraState", "driverCameraState", "wideRoadCameraState", "thumbnail"});
}

void cameras_open(MultiCameraState *s) {
  LOG("-- Opening devices");
  // Userspace CamHw only — no rkaiq_3A daemon.
  RkIspUserspaceController::stop_rkaiq();
  int n = 0;
  if (!env_disable_wide_road) n++;
  if (!env_disable_road) n++;
  if (!env_disable_driver) n++;
  RkIspUserspaceController::set_multi_cam_count(n);
  s->wide_road_cam.camera_open(s, 0, !env_disable_wide_road);
  LOGD("wide road camera opened");
  s->road_cam.camera_open(s, 1, !env_disable_road);
  LOGD("road camera opened");
  s->driver_cam.camera_open(s, 2, !env_disable_driver);
  LOGD("driver camera opened");

  {
    CameraState *cams[3] = {&s->wide_road_cam, &s->road_cam, &s->driver_cam};
    for (int i = 0; i < 3; i++) {
      if (!cams[i]->enabled || !cams[i]->rk_isp || !cams[i]->rk_isp->active()) continue;
      if (!cams[i]->rk_isp->prepare_and_start()) {
        LOGE("camera %d: CamHw prepare/start failed", cams[i]->camera_num);
      }
      // CamHw reprograms the sensor; flips from camera_open are lost without this.
      cams[i]->apply_sensor_flips();
    }
  }
}

void CameraState::apply_sensor_flips() {
  if (ctrl_fd < 0) return;
  ctrl.id = V4L2_CID_HFLIP;
  ctrl.value = 0;
  if (ioctl(ctrl_fd, VIDIOC_S_CTRL, &ctrl) < 0) {
    LOGE("camera %d: HFLIP=0 failed errno=%d '%s'", camera_num, errno, strerror(errno));
  }
  ctrl.id = V4L2_CID_VFLIP;
  ctrl.value = 1;
  if (ioctl(ctrl_fd, VIDIOC_S_CTRL, &ctrl) < 0) {
    LOGE("camera %d: VFLIP=1 failed errno=%d '%s'", camera_num, errno, strerror(errno));
  }
  int hflip = -1, vflip = -1;
  ctrl.id = V4L2_CID_HFLIP;
  if (ioctl(ctrl_fd, VIDIOC_G_CTRL, &ctrl) == 0) hflip = ctrl.value;
  ctrl.id = V4L2_CID_VFLIP;
  if (ioctl(ctrl_fd, VIDIOC_G_CTRL, &ctrl) == 0) vflip = ctrl.value;
  LOGW("camera %d: sensor flips H=%d V=%d (want 0/1)", camera_num, hflip, vflip);
}

void CameraState::camera_close() {
  // stop devices
  LOG("-- Stop devices %d", camera_num);

  if (video_fd.fd_ >= 0) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(video_fd, VIDIOC_STREAMOFF, &type) < 0) {
      LOGW("camera %d: VIDIOC_STREAMOFF failed errno=%d '%s'", camera_num, errno, strerror(errno));
    }
  }

  if (buf.camera_bufs) {
    for (int i = 0; i < FRAME_BUF_COUNT; i++) {
      if (buf.camera_bufs[i].fd >= 0) {
        close(buf.camera_bufs[i].fd);
        buf.camera_bufs[i].fd = -1;
      }
      if (buf.camera_bufs[i].addr != nullptr && buf.camera_bufs[i].addr != MAP_FAILED && buf.camera_bufs[i].mmap_len > 0) {
        munmap(buf.camera_bufs[i].addr, buf.camera_bufs[i].mmap_len);
        buf.camera_bufs[i].addr = nullptr;
        buf.camera_bufs[i].mmap_len = 0;
      }
    }
  }

  // Shut down ISP controller before closing sensor ctrl_fd (it holds that fd).
  if (rk_isp) {
    rk_isp->shutdown();
    rk_isp.reset();
  }

  if (i2c_fd.fd_ >= 0) {
    close(i2c_fd.fd_);
    i2c_fd.fd_ = -1;
  }

  // unique_fd does not auto-invalidate on manual close; clear fd_ to avoid double-close.
  if (ctrl_fd.fd_ >= 0) {
    close(ctrl_fd.fd_);
    ctrl_fd.fd_ = -1;
  }
  if (video_fd.fd_ >= 0) {
    close(video_fd.fd_);
    video_fd.fd_ = -1;
  }
  if (csiphy_fd.fd_ >= 0) {
    close(csiphy_fd.fd_);
    csiphy_fd.fd_ = -1;
  }

  LOGD("destroyed session %d", camera_num);
}

void cameras_close(MultiCameraState *s) {
  s->driver_cam.camera_close();
  s->road_cam.camera_close();
  s->wide_road_cam.camera_close();

  delete s->pm;

  LOGW("RkIspUserspace: not starting rkaiq on close (userspace CamHw is default)");

}

static void process_driver_camera(MultiCameraState *s, CameraState *c, uint32_t cnt) {
  MessageBuilder msg;
  auto framed = msg.initEvent().initDriverCameraState();
  fill_frame_data(framed, c->buf.cur_frame_data, c);

  s->pm->send("driverCameraState", msg);
}


static void process_road_camera(MultiCameraState *s, CameraState *c, uint32_t cnt) {
  const CameraBuf *b = &c->buf;

  MessageBuilder msg;
  auto framed = c == &s->road_cam ? msg.initEvent().initRoadCameraState() : msg.initEvent().initWideRoadCameraState();
  fill_frame_data(framed, b->cur_frame_data, c);
  if (env_log_raw_frames && c == &s->road_cam && cnt % 100 == 5) {  // no overlap with qlog decimation
    framed.setImage(get_raw_frame_image(b));
  }
  LOGT(c->buf.cur_frame_data.frame_id, "%s: Image set", c == &s->road_cam ? "RoadCamera" : "WideRoadCamera");

  s->pm->send(c == &s->road_cam ? "roadCameraState" : "wideRoadCameraState", msg);
}


#define THRESHOLD 10000000
bool check_timestamp_sync(uint64_t *t1, int len1, uint64_t *t2, int len2) {
    int i = 0, j = 0;

    while (i < len1 && j < len2) {
        uint64_t diff = t1[i] > t2[j] ? t1[i] - t2[j] : t2[j] - t1[i];
        if (diff <= THRESHOLD) {
            return true;
        }

        // Move the pointer with smaller timestamp forward
        if (t1[i] < t2[j]) {
            i++;
        } else {
            j++;
        }
    }

    return false; // No match within threshold
}

#define SYNC_CHECK_LEN 5
#define SYNC_CHECK_COUNT 40
void cameras_run(MultiCameraState *s) {
  LOG("-- Starting threads");
  std::vector<std::thread> threads;
  if (s->driver_cam.enabled) threads.push_back(start_process_thread(s, &s->driver_cam, process_driver_camera));
  if (s->road_cam.enabled) threads.push_back(start_process_thread(s, &s->road_cam, process_road_camera));
  if (s->wide_road_cam.enabled) threads.push_back(start_process_thread(s, &s->wide_road_cam, process_road_camera));

  s->wide_road_cam.stream_start();
  s->road_cam.stream_start();
  s->driver_cam.stream_start();

  uint64_t road_cam_ts[SYNC_CHECK_LEN];
  uint64_t wide_cam_ts[SYNC_CHECK_LEN];
  int count = 0;

  // poll events
  LOG("-- Dequeueing Video events");
  while (!do_exit) {
    std::vector<pollfd> fds;
    CameraState *cams[3] = {&s->driver_cam, &s->road_cam, &s->wide_road_cam};
    int video_idx[3];
    for (int i = 0; i < 3; i++) {
      video_idx[i] = -1;
      if (!cams[i]->enabled) continue;
      video_idx[i] = (int)fds.size();
      fds.push_back({.fd = cams[i]->video_fd, .events = POLLPRI | POLLIN, .revents = 0});
    }

    if (fds.empty()) {
      usleep(10000);
      continue;
    }
    int ret = poll(fds.data(), fds.size(), 1000);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      LOGE("poll failed (%d - %d)", ret, errno);
      break;
    }

    for (int i = 0; i < 3; i++) {
      if (video_idx[i] >= 0 && (fds[video_idx[i]].revents & (POLLPRI | POLLIN))) {
        cams[i]->dequeue_buf();
        if (cams[i] == &s->road_cam) {
          count++;
          if (count <= (SYNC_CHECK_COUNT + SYNC_CHECK_LEN - 1) && count >= SYNC_CHECK_COUNT) {
            road_cam_ts[count % SYNC_CHECK_COUNT] = s->road_cam.cap_time;
          }
        } else if (cams[i] == &s->wide_road_cam) {
          if (count <= (SYNC_CHECK_COUNT + SYNC_CHECK_LEN - 1) && count >= SYNC_CHECK_COUNT) {
            wide_cam_ts[count % SYNC_CHECK_COUNT] = s->wide_road_cam.cap_time;
          }
        }
      }
    }
  }

  LOG("************** STOPPING **************");

  for (auto &t : threads) t.join();

  // Stop thumbnail worker before cameras_close() destroys PubMaster.
  stop_thumbnail_worker();
  cameras_close(s);
}

