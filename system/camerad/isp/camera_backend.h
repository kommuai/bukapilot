#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "common/util.h"
#include "system/camerad/camera/common.h"
#include "system/camerad/isp/userspace.h"
#include "system/camerad/sensor/sensor.h"

struct MultiCameraState;
class CameraState;

class Ka2CameraBackend {
public:
  bool open(CameraState *cam);
  void close(CameraState *cam);
  void on_stream_start(CameraState *cam);
  void on_dequeue(CameraState *cam, FrameMetadata &md);
  void enqueue_ae(CameraState *cam, int buf_idx, const FrameMetadata &md);
  void wait_for_ae(int buf_idx);
  void apply_flips(CameraState *cam);

  RkIspUserspaceController *isp() { return rk_isp_.get(); }
  const RkIspUserspaceController *isp() const { return rk_isp_.get(); }

  static bool prepare_system(MultiCameraState *s);
  static void prepare_isp_all(MultiCameraState *s);
  static void synced_stream_and_start(MultiCameraState *s);

private:
  struct AeJob {
    CameraState *cam = nullptr;
    int buf_idx = -1;
    FrameMetadata md = {};
  };

  void start_ae_worker();
  void stop_ae_worker();
  void ae_worker_loop();
  void process_ae_job(const AeJob &job);
  int32_t run_rkaiq_ae(const rk_aiq_customAe_stats_t *stats,
                       rk_aiq_customeAe_results_t *result);
  void log_ae_metrics(CameraState *cam, uint32_t frame_id);
  float get_gain_factor(const CameraState *cam) const;
  void apply_pwl_on(CameraState *cam);
  bool sensors_i2c(CameraState *cam, const i2c_random_wr_payload *dat, int len);
  void set_exposure_rect(CameraState *cam);
  bool seed_external_exposure(CameraState *cam);
  void set_camera_exposure(CameraState *cam, float grey_frac);
  bool commit_exposure(CameraState *cam, int exp_t, int gidx, bool hcg);
  bool set_frame_length_vts(CameraState *cam, int exposure_lines);
  bool read_ctrl(const CameraState *cam, uint32_t id, int *out) const;
  bool write_ctrl(const CameraState *cam, uint32_t id, int val) const;
  std::string resolve_mainpath_dev(int camera_num) const;

  std::unique_ptr<RkIspUserspaceController> rk_isp_;
  CameraState *camera_ = nullptr;
  std::mutex ae_mtx_;
  std::condition_variable ae_cv_;
  std::condition_variable ae_done_cv_;
  std::deque<AeJob> ae_jobs_;
  std::thread ae_thread_;
  std::array<bool, 4> ae_pending_ = {};
  bool ae_running_ = false;
  static constexpr size_t kAeQueueDepth = 2;

  std::atomic<uint64_t> ae_dropped_jobs_{0};
  std::atomic<uint64_t> ae_exposure_skips_{0};
  std::atomic<uint64_t> ae_reversal_skips_{0};
  std::atomic<uint64_t> ae_luma_last_ns_{0};
  std::atomic<uint64_t> ae_luma_max_ns_{0};
  std::atomic<uint64_t> ae_i2c_last_ns_{0};
  std::atomic<uint64_t> ae_i2c_max_ns_{0};
  std::atomic<uint64_t> ae_isp_last_ns_{0};
  std::atomic<uint64_t> ae_isp_max_ns_{0};
  std::atomic<uint64_t> ae_rkaiq_last_ns_{0};
  std::atomic<uint64_t> ae_rkaiq_max_ns_{0};
  std::atomic<uint64_t> ae_rkaiq_reads_{0};
  std::atomic<uint64_t> ae_rkaiq_failures_{0};
  std::atomic<uint64_t> ae_dequeue_to_ae_last_ns_{0};
  std::atomic<uint64_t> ae_dequeue_to_ae_max_ns_{0};
  std::atomic<uint32_t> rkaiq_ae_callback_seq_{0};
  std::atomic<float> rkaiq_pending_grey_{0.0f};
  uint32_t ae_completed_jobs_ = 0;
  uint32_t ae_frame_id_ = 0;
  uint32_t rkaiq_ae_consumed_seq_ = 0;

  std::mutex exp_lock_;
  std::mutex i2c_lock_;
  unique_fd i2c_fd_;
  int i2c_bus_ = -1;
  int i2c_addr_ = 0x36;
  int exposure_time_ = 5;
  int frame_length_vts_ = 0;
  bool dc_gain_enabled_ = false;
  int dc_gain_weight_ = 1;
  int gain_idx_ = 0;
  float analog_gain_frac_ = 1.0f;
  float cur_ev_[3] = {};
  float best_ev_score_ = 1e6f;
  int new_exp_g_ = 0;
  int new_exp_t_ = 0;
  int8_t last_exposure_direction_ = 0;
  int8_t last_gain_direction_ = 0;
  int8_t pending_exposure_direction_ = 0;
  int8_t pending_gain_direction_ = 0;
  uint8_t pending_reversal_frames_ = 0;
  uint8_t reversal_hold_frames_ = 0;
  float measured_grey_fraction_ = 0.f;
  float target_grey_fraction_ = 0.125f;
  float fl_pix_ = 0.f;
  Rect ae_xywh_ = {};
  bool ae_roi_ready_ = false;
  std::array<i2c_random_wr_payload, 12> exposure_regs_{};
  std::array<i2c_random_wr_payload, 12> last_exp_regs_{};
  size_t exposure_reg_count_ = 0;
  size_t last_exp_reg_count_ = 0;
  float last_sensor_temp_c_ = -999.0f;

  // The capture thread only publishes these snapshots; all AE state remains
  // owned by the bounded worker.
  std::atomic<int> published_exposure_time_{5};
  std::atomic<float> published_gain_{1.0f};
  std::atomic<bool> published_hcg_{false};
  std::atomic<float> published_sensor_temp_c_{-999.0f};
  std::atomic<float> published_measured_grey_{0.0f};
  std::atomic<float> published_target_grey_{0.125f};
};
