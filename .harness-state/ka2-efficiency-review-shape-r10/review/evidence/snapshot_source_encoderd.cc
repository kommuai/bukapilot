#include <cassert>
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include "system/loggerd/loggerd.h"
#include "system/loggerd/encoder/jpeg_encoder.h"
#include "common/timing.h"

#if defined(__TICI__) || defined(QCOM2)
#include "system/loggerd/encoder/v4l_encoder.h"
#define Encoder V4LEncoder
#elif defined(RK3588)
#include "system/loggerd/encoder/mpp_encoder.h"
#define Encoder MppEncoder
#else
#include "system/loggerd/encoder/ffmpeg_encoder.h"
#define Encoder FfmpegEncoder
#endif

ExitHandler do_exit;
constexpr int ENCODER_VIPC_STALL_TIMEOUT_MS = 3000;
constexpr int ENCODER_OUTPUT_STALL_TIMEOUT_MS = 1000;
constexpr int ENCODER_REOPEN_FAILURE_THRESHOLD = 10;
constexpr int ENCODER_RECONNECT_BACKOFF_MS = 50;
constexpr int ENCODER_HEALTH_LOG_PERIOD_MS = 5000;
constexpr int ENCODER_FAILURE_LOG_PERIOD_MS = 1000;
constexpr int ENCODER_FAILURE_LOG_BATCH_SIZE = 30;
constexpr int ENCODER_SEGMENT_SYNC_PERIOD_MS = 100;
constexpr int ENCODER_SEGMENT_SYNC_PERIOD_DETERMINISTIC_MS = 100;
#if defined(RK3588)
constexpr int MPP_SEGMENT_DRAIN_TIMEOUT_MS = 900;
#endif
constexpr int ENCODER_OUTPUT_PACKET_STALL_TIMEOUT_MS = 1000;
constexpr int ENCODER_QROAD_PACKET_GRACE_MS = 3000;
constexpr int ENCODER_RECOVERY_RETRY_BASE_SECONDS = 1;
constexpr int ENCODER_RECOVERY_RETRY_MAX_SECONDS = 30;
#if defined(RK3588)
constexpr int MPP_DRAIN_POLL_SLICE_MS = 10;
#endif

enum class EncoderOutputState : uint8_t {
  Active = 0,
  Unavailable = 1,
  Recovering = 2,
};

struct EncoderOutputRecovery {
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  EncoderOutputState state = EncoderOutputState::Active;
  TimePoint retry_at{};
  TimePoint pending_since{};
  TimePoint no_progress_since{};
  TimePoint watchdog_grace_until{};
  TimePoint retry_progress_since{};
  uint64_t observed_packet_count = 0;
  uint64_t observed_submitted_count = 0;
  int consecutive_input_errors = 0;
  int retry_backoff_level = 0;
  bool awaiting_progress_reset = false;

  void set_watchdog_grace(TimePoint now, bool qroad) {
    watchdog_grace_until = qroad ? now + std::chrono::milliseconds(ENCODER_QROAD_PACKET_GRACE_MS) : TimePoint{};
  }

  void reset_after_reopen(bool is_open, bool qroad, TimePoint retry_now, TimePoint grace_now,
                          uint64_t packets, uint64_t submitted) {
    state = is_open ? EncoderOutputState::Active : EncoderOutputState::Unavailable;
    retry_at = is_open ? TimePoint{} : retry_now + std::chrono::seconds(ENCODER_RECOVERY_RETRY_BASE_SECONDS);
    pending_since = {};
    no_progress_since = {};
    set_watchdog_grace(grace_now, qroad);
    awaiting_progress_reset = false;
    retry_backoff_level = 0;
    consecutive_input_errors = 0;
    observed_packet_count = packets;
    observed_submitted_count = submitted;
  }

  int schedule_retry(TimePoint now) {
    state = EncoderOutputState::Unavailable;
    pending_since = {};
    const int delay_seconds = retry_backoff_level >= 5 ? ENCODER_RECOVERY_RETRY_MAX_SECONDS :
      std::min(ENCODER_RECOVERY_RETRY_MAX_SECONDS,
               ENCODER_RECOVERY_RETRY_BASE_SECONDS << retry_backoff_level);
    retry_at = now + std::chrono::seconds(delay_seconds);
    return delay_seconds;
  }

  bool begin_due_retry(TimePoint now) {
    if (state != EncoderOutputState::Unavailable || now < retry_at) return false;
    state = EncoderOutputState::Recovering;
    retry_backoff_level = std::min(retry_backoff_level + 1, 6);
    return true;
  }

  void retry_opened(TimePoint grace_now, bool qroad, uint64_t packets, uint64_t submitted) {
    state = EncoderOutputState::Active;
    retry_at = {};
    pending_since = {};
    no_progress_since = {};
    set_watchdog_grace(grace_now, qroad);
    awaiting_progress_reset = true;
    retry_progress_since = {};
    consecutive_input_errors = 0;
    observed_packet_count = packets;
    observed_submitted_count = submitted;
  }

  void planned_segment_opened(TimePoint grace_now, bool qroad, uint64_t packets, uint64_t submitted) {
    observed_packet_count = packets;
    observed_submitted_count = submitted;
    pending_since = {};
    no_progress_since = {};
    set_watchdog_grace(grace_now, qroad);
  }

  void rotation_committed(TimePoint now, bool qroad) {
    no_progress_since = {};
    pending_since = {};
    if (state == EncoderOutputState::Active && qroad) set_watchdog_grace(now, true);
  }

  void note_input_error() { consecutive_input_errors++; }
  void note_input_success() { consecutive_input_errors = 0; }
  bool input_error_limit_reached(int limit) const { return consecutive_input_errors >= limit; }
  bool active() const { return state == EncoderOutputState::Active; }
  bool should_reopen_for_segment() const { return active(); }

  bool observe_submitted(uint64_t submitted, TimePoint now) {
    if (submitted <= observed_submitted_count) return false;
    observed_submitted_count = submitted;
    if (no_progress_since == TimePoint{}) no_progress_since = now;
    return true;
  }

  bool observe_published(uint64_t published, TimePoint now) {
    if (published <= observed_packet_count) return false;
    observed_packet_count = published;
    no_progress_since = {};
    if (awaiting_progress_reset) {
      if (retry_progress_since == TimePoint{}) {
        retry_progress_since = now;
      } else if (now - retry_progress_since >= std::chrono::seconds(1)) {
        retry_backoff_level = 0;
        awaiting_progress_reset = false;
        retry_progress_since = {};
      }
    }
    return true;
  }

  void observe_pending(size_t pending_count, TimePoint now) {
    if (pending_count != 0 && no_progress_since == TimePoint{}) no_progress_since = now;
    if (pending_count == 0) {
      pending_since = {};
    } else if (pending_since == TimePoint{}) {
      pending_since = now;
    }
  }

  bool packet_watchdog_expired(TimePoint now, bool in_rotation_barrier) const {
    if (in_rotation_barrier || no_progress_since == TimePoint{}) return false;
    const TimePoint watchdog_origin = std::max(no_progress_since, watchdog_grace_until);
    return now >= watchdog_origin &&
           now - watchdog_origin >= std::chrono::milliseconds(ENCODER_OUTPUT_PACKET_STALL_TIMEOUT_MS);
  }

  bool should_quarantine_after_drain(size_t pending_count) const {
    return state == EncoderOutputState::Active && pending_count != 0;
  }
};

#if defined(RK3588)
template <typename NowFn>
static int schedule_output_retry_policy(EncoderOutputRecovery &health, MppEncoder &mpp,
                                        NowFn &&now, bool deterministic_mode,
                                        const char *thread_name, const char *publish_name, const char *reason,
                                        bool context_already_closed = false) {
  if (!deterministic_mode) return 0;
  if (!context_already_closed && (mpp.encoder_is_open() || mpp.pending_frame_count() > 0)) {
    mpp.set_flush_pending_on_close(false);
    mpp.encoder_close();
  }
  mpp.set_flush_pending_on_close(true);
  const int delay_seconds = health.schedule_retry(now());
  LOGE("encoder %s output %s unavailable (%s); retry in %d sec",
       thread_name, publish_name, reason, delay_seconds);
  return delay_seconds;
}
#endif

enum class SegmentPhase : uint8_t {
  Steady = 0,
  PreRotateBarrier = 1,
  RotateCommit = 2,
  RecoveryBarrier = 3,
};

const char *phase_name(SegmentPhase phase) {
  switch (phase) {
    case SegmentPhase::Steady: return "steady";
    case SegmentPhase::PreRotateBarrier: return "pre_rotate_barrier";
    case SegmentPhase::RotateCommit: return "rotate_commit";
    case SegmentPhase::RecoveryBarrier: return "recovery_barrier";
  }
  return "unknown";
}

std::vector<int> parse_affinity_cores(const char *env_val) {
  std::vector<int> cores;
  if (env_val == nullptr || env_val[0] == '\0') return cores;

  std::stringstream ss(env_val);
  std::string token;
  while (std::getline(ss, token, ',')) {
    try {
      if (!token.empty()) cores.push_back(std::stoi(token));
    } catch (const std::exception&) {
      // Ignore invalid affinity token and continue parsing.
    }
  }
  return cores;
}

bool parse_nonnegative_int_strict(const std::string &value, int *parsed) {
  if (value.empty() || parsed == nullptr) return false;
  int result = 0;
  const char *begin = value.data();
  const char *end = begin + value.size();
  const auto conversion = std::from_chars(begin, end, result);
  if (conversion.ec != std::errc{} || conversion.ptr != end || result < 0) return false;
  *parsed = result;
  return true;
}

#if defined(RK3588) && defined(LOGGERD_TEST_BUILD)
bool encoderd_test_runtime_enabled() {
  const char *test_env = getenv("LOGGERD_TEST");
  return test_env != nullptr && std::strcmp(test_env, "1") == 0;
}

void log_test_mpp_counter_snapshot(const MppEncoder &encoder, const char *stream_name, int cur_seg) {
  if (!encoderd_test_runtime_enabled()) return;
  LOGW("LOGGERD_TEST_MPP_COUNTERS stream=%s cur_seg=%d segment_transition_reopen_count=%llu "
       "failure_recovery_reopen_count=%llu failure_retry_count=%llu",
       stream_name, cur_seg,
       static_cast<unsigned long long>(encoder.segment_transition_reopen_count()),
       static_cast<unsigned long long>(encoder.failure_recovery_reopen_count()),
       static_cast<unsigned long long>(encoder.failure_retry_count()));
}
#endif

#if defined(RK3588)
template <typename NowFn>
static bool retry_output_policy(EncoderOutputRecovery &health, MppEncoder &mpp,
                                EncoderOutputRecovery::TimePoint due_now, NowFn &&now,
                                int current_segment, bool qroad,
                                const char *thread_name, const char *publish_name,
                                int &encode_failure_count, int &encode_failure_burst_count) {
  if (!health.begin_due_retry(due_now)) return false;

  mpp.note_failure_retry_started();
#if defined(LOGGERD_TEST_BUILD)
  log_test_mpp_counter_snapshot(mpp, publish_name, current_segment);
#endif
  mpp.encoder_open();
  mpp.set_segment_num(current_segment);
  mpp.set_flush_pending_on_close(true);
  if (!mpp.encoder_is_open()) {
    (void)schedule_output_retry_policy(health, mpp, now, true,
                                       thread_name, publish_name, "scheduled reopen failed", true);
    return true;
  }

  mpp.note_failure_recovery_reopen();
#if defined(LOGGERD_TEST_BUILD)
  log_test_mpp_counter_snapshot(mpp, publish_name, current_segment);
#endif
  const auto grace_now = qroad ? now() : EncoderOutputRecovery::TimePoint{};
  const uint64_t packets = mpp.packet_published_count();
  const uint64_t submitted = mpp.input_submitted_count();
  health.retry_opened(grace_now, qroad, packets, submitted);
  encode_failure_count = 0;
  encode_failure_burst_count = 0;
  LOGW("encoder %s output %s recovered at segment %d", thread_name, publish_name, current_segment);
  return true;
}
#endif

const char *stream_affinity_env(VisionStreamType stream_type) {
  switch (stream_type) {
    case VISION_STREAM_ROAD:
      return "ENCODERD_ROAD_AFFINITY";
    case VISION_STREAM_WIDE_ROAD:
      return "ENCODERD_WIDE_AFFINITY";
    case VISION_STREAM_DRIVER:
      return "ENCODERD_DRIVER_AFFINITY";
    default:
      return "ENCODERD_STREAM_AFFINITY";
  }
}

struct EncoderdState {
  int max_waiting = 0;

  // Sync logic for startup
  std::atomic<int> encoders_ready = 0;
  std::atomic<uint32_t> start_frame_id = 0;
  bool camera_ready[VISION_STREAM_WIDE_ROAD + 1] = {};
  bool camera_synced[VISION_STREAM_WIDE_ROAD + 1] = {};
  // Global segment target to force all camera threads to converge together.
  std::atomic<int> forced_segment_target{-1};
  std::atomic<bool> forced_segment_drop_pending{true};
  // Shared parsed loggerd authority (single-reader, multi-consumer).
  std::atomic<int> loggerd_segment_authority{-1};
  std::atomic<int> loggerd_segment_epoch{-1};
  // 0=started, 1=committed
  std::atomic<int> loggerd_rotate_state{1};
};

// Handle initial encoder syncing by waiting for all encoders to reach the same frame id
bool sync_encoders(EncoderdState *s, VisionStreamType cam_type, uint32_t frame_id) {
  if (s->camera_synced[cam_type]) return true;

  if (s->max_waiting > 1 && s->encoders_ready != s->max_waiting) {
    // add a small margin to the start frame id in case one of the encoders already dropped the next frame
    update_max_atomic(s->start_frame_id, frame_id + 2);
    if (std::exchange(s->camera_ready[cam_type], true) == false) {
      ++s->encoders_ready;
      LOGD("camera %d encoder ready", cam_type);
    }
    return false;
  } else {
    if (s->max_waiting == 1) update_max_atomic(s->start_frame_id, frame_id);
    bool synced = frame_id >= s->start_frame_id;
    s->camera_synced[cam_type] = synced;
    if (!synced) LOGD("camera %d waiting for frame %d, cur %d", cam_type, (int)s->start_frame_id, frame_id);
    return synced;
  }
}

void reset_sync_for_camera(EncoderdState *s, VisionStreamType cam_type) {
  if (s->camera_ready[cam_type]) {
    int prev = s->encoders_ready.fetch_sub(1);
    if (prev <= 0) {
      s->encoders_ready = 0;
    }
  }
  s->camera_ready[cam_type] = false;
  s->camera_synced[cam_type] = false;
}


void encoder_thread(EncoderdState *s, const LogCameraInfo &cam_info) {
  util::set_thread_name(cam_info.thread_name);
#if defined(LOGGERD_TEST_BUILD)
  const char *test_runtime_env = getenv("LOGGERD_TEST");
  const bool test_runtime_enabled = test_runtime_env != nullptr && std::strcmp(test_runtime_env, "1") == 0;
  bool test_all_output_reopen_tracking = false;
  uint64_t test_all_output_reopen_count = 0;
#endif
  const bool deterministic_ka2_enabled =
#if defined(RK3588)
    !(getenv("ENCODERD_DETERMINISTIC_DISABLE") != nullptr && atoi(getenv("ENCODERD_DETERMINISTIC_DISABLE")) != 0);
#else
    false;
#endif
  const char *deterministic_test_env = LOGGERD_TEST ? getenv("LOGGERD_TEST_DETERMINISTIC") : nullptr;
  const bool deterministic_mode = deterministic_ka2_enabled &&
    (!LOGGERD_TEST || (deterministic_test_env != nullptr && atoi(deterministic_test_env) == 1));
  const bool critical_stream = (cam_info.stream_type == VISION_STREAM_ROAD) ||
                               (cam_info.stream_type == VISION_STREAM_WIDE_ROAD);
  if (!Hardware::PC()) {
    std::vector<int> stream_affinity = parse_affinity_cores(getenv(stream_affinity_env(cam_info.stream_type)));
    if (!stream_affinity.empty()) {
      int ret = util::set_core_affinity(stream_affinity);
      if (ret != 0) {
        LOGW("failed to set affinity for stream %s", cam_info.thread_name);
      }
    }
#if defined(RK3588)
    if (deterministic_mode && getenv("ENCODERD_DET_STREAM_RT") != nullptr &&
        atoi(getenv("ENCODERD_DET_STREAM_RT")) != 0) {
      // Prioritize road/wide critical streams over thumbnails/background work.
      int stream_prio = (cam_info.stream_type == VISION_STREAM_DRIVER) ? 56 : 58;
      int prio_ret = util::set_realtime_priority(stream_prio);
      if (prio_ret != 0) {
        LOGW("failed to set RT priority %d for stream %s", stream_prio, cam_info.thread_name);
      }
    }
#endif
  }
  const bool authority_reader = (cam_info.stream_type == VISION_STREAM_ROAD) || (s->max_waiting == 1);

  std::vector<std::unique_ptr<Encoder>> encoders;
  std::vector<int> encode_failure_counts;
  std::vector<int> encode_failure_burst_counts;
  std::vector<double> encode_failure_last_log_tms;
  std::vector<EncoderOutputRecovery> output_recovery;
  VisionIpcClient vipc_client = VisionIpcClient("camerad", cam_info.stream_type, false);
  Params params;

  std::unique_ptr<JpegEncoder> jpeg_encoder;

  int cur_seg = 0;
  size_t input_width = 0;
  size_t input_height = 0;

  auto close_and_reset_encoders = [&](bool clear_instances) {
    for (auto &e : encoders) {
      e->encoder_close();
    }
    if (clear_instances) {
      encoders.clear();
      encode_failure_counts.clear();
      encode_failure_burst_counts.clear();
      encode_failure_last_log_tms.clear();
      output_recovery.clear();
      jpeg_encoder.reset();
      cur_seg = 0;
      input_width = 0;
      input_height = 0;
    }
    reset_sync_for_camera(s, cam_info.stream_type);
  };

  auto reopen_all_encoders = [&]() {
#if defined(LOGGERD_TEST_BUILD)
    if (test_runtime_enabled && test_all_output_reopen_tracking) ++test_all_output_reopen_count;
#endif
#if defined(RK3588)
    for (auto &e : encoders) {
      static_cast<MppEncoder *>(e.get())->set_flush_pending_on_close(false);
    }
#endif
    for (size_t i = 0; i < encoders.size(); ++i) {
      encoders[i]->encoder_close();
      encoders[i]->encoder_open();
      // Recovery reopen should not advance segment timeline.
      encoders[i]->set_segment_num(cur_seg);
      encode_failure_counts[i] = 0;
#if defined(RK3588)
      if (deterministic_mode && i < output_recovery.size()) {
        auto *mpp = static_cast<MppEncoder *>(encoders[i].get());
        mpp->set_flush_pending_on_close(true);
        const bool open = mpp->encoder_is_open();
        const bool qroad = cam_info.encoder_infos[i].publish_name == std::string("qRoadEncodeData");
        const auto retry_now = open ? EncoderOutputRecovery::TimePoint{} : std::chrono::steady_clock::now();
        const auto grace_now = qroad ? std::chrono::steady_clock::now() : EncoderOutputRecovery::TimePoint{};
        const uint64_t packets = mpp->packet_published_count();
        const uint64_t submitted = mpp->input_submitted_count();
        output_recovery[i].reset_after_reopen(
          open, qroad, retry_now, grace_now, packets, submitted);
      }
#endif
    }
#if defined(RK3588)
    for (auto &e : encoders) {
      static_cast<MppEncoder *>(e.get())->set_flush_pending_on_close(true);
    }
#endif
  };

#if defined(RK3588)
  auto schedule_output_retry = [&](size_t i, const char *reason, bool context_already_closed = false) {
    if (!deterministic_mode || i >= output_recovery.size()) return;
    auto *mpp = static_cast<MppEncoder *>(encoders[i].get());
    (void)schedule_output_retry_policy(output_recovery[i], *mpp,
                                       [] { return std::chrono::steady_clock::now(); },
                                       deterministic_mode, cam_info.thread_name,
                                       cam_info.encoder_infos[i].publish_name, reason, context_already_closed);
  };

  auto retry_unavailable_outputs = [&]() {
    if (!deterministic_mode) return;
    const auto now = std::chrono::steady_clock::now();
    for (size_t i = 0; i < encoders.size() && i < output_recovery.size(); ++i) {
      auto *mpp = static_cast<MppEncoder *>(encoders[i].get());
      const bool qroad = cam_info.encoder_infos[i].publish_name == std::string("qRoadEncodeData");
      (void)retry_output_policy(output_recovery[i], *mpp, now,
                                [] { return std::chrono::steady_clock::now(); },
                                cur_seg, qroad, cam_info.thread_name,
                                cam_info.encoder_infos[i].publish_name,
                                encode_failure_counts[i], encode_failure_burst_counts[i]);
    }
  };
#endif

  auto advance_segment_once = [&](bool drop_pending) {
#if defined(RK3588)
    if (deterministic_mode) {
      const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(MPP_SEGMENT_DRAIN_TIMEOUT_MS);
      bool deadline_expired = false;
      while (true) {
        bool any_pending = false;
        for (size_t i = 0; i < encoders.size(); ++i) {
          if (!output_recovery[i].active()) continue;
          auto *mpp = static_cast<MppEncoder *>(encoders[i].get());
          if (!output_recovery[i].should_quarantine_after_drain(mpp->pending_frame_count())) continue;
          any_pending = true;
          const auto now = std::chrono::steady_clock::now();
          if (now >= deadline) {
            deadline_expired = true;
            break;
          }
          const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
          const int slice_ms = std::max(1, std::min<int>(MPP_DRAIN_POLL_SLICE_MS, static_cast<int>(remaining_ms)));
          (void)mpp->flush_pending_packets(slice_ms, false);
        }
        if (!any_pending) break;
        if (std::chrono::steady_clock::now() >= deadline) {
          deadline_expired = true;
          break;
        }
        util::sleep_for(1);
      }

      std::vector<std::string> timed_out_outputs;
      if (deadline_expired) {
        for (size_t i = 0; i < encoders.size(); ++i) {
          if (!output_recovery[i].active()) continue;
          auto *mpp = static_cast<MppEncoder *>(encoders[i].get());
          if (!output_recovery[i].should_quarantine_after_drain(mpp->pending_frame_count())) continue;
          timed_out_outputs.emplace_back(cam_info.encoder_infos[i].publish_name);
          schedule_output_retry(i, "segment drain deadline");
        }
      }
      if (!timed_out_outputs.empty()) {
        std::string names;
        for (const auto &name : timed_out_outputs) {
          if (!names.empty()) names += ",";
          names += name;
        }
        LOGE("encoder %s segment %d shared %d ms drain deadline expired; quarantined pending outputs: %s",
             cam_info.thread_name, cur_seg, MPP_SEGMENT_DRAIN_TIMEOUT_MS, names.c_str());
      }

      ++cur_seg;
      for (size_t i = 0; i < encoders.size(); ++i) {
        auto *mpp = static_cast<MppEncoder *>(encoders[i].get());
        if (output_recovery[i].should_reopen_for_segment()) {
          mpp->set_flush_pending_on_close(true);
          mpp->encoder_close();
          mpp->encoder_open();
          mpp->set_segment_num(cur_seg);
          if (mpp->encoder_is_open()) {
            mpp->note_segment_transition_reopen();
#if defined(LOGGERD_TEST_BUILD)
            log_test_mpp_counter_snapshot(*mpp, cam_info.encoder_infos[i].publish_name, cur_seg);
#endif
            const bool qroad = cam_info.encoder_infos[i].publish_name == std::string("qRoadEncodeData");
            const uint64_t packets = mpp->packet_published_count();
            const uint64_t submitted = mpp->input_submitted_count();
            const auto grace_now = qroad ? std::chrono::steady_clock::now() : EncoderOutputRecovery::TimePoint{};
            output_recovery[i].planned_segment_opened(grace_now, qroad, packets, submitted);
          } else {
            schedule_output_retry(i, "planned segment reopen failed", true);
          }
        } else {
          mpp->set_segment_num(cur_seg);
        }
      }
      return true;
    }

    if (!drop_pending) {
      const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(MPP_SEGMENT_DRAIN_TIMEOUT_MS);
      for (auto &e : encoders) {
        const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now()).count();
        if (remaining_ms <= 0 ||
            !static_cast<MppEncoder *>(e.get())->flush_pending_packets(static_cast<int>(remaining_ms))) {
          LOGE("encoder %s failed to drain segment %d before the %d ms rotation deadline",
               cam_info.thread_name, cur_seg, MPP_SEGMENT_DRAIN_TIMEOUT_MS);
          for (auto &encoder : encoders) {
            static_cast<MppEncoder *>(encoder.get())->set_flush_pending_on_close(false);
          }
          return false;
        }
      }
    }
    for (auto &e : encoders) {
      static_cast<MppEncoder *>(e.get())->set_flush_pending_on_close(!drop_pending);
    }
#endif
    for (auto &e : encoders) {
      e->encoder_close();
      e->encoder_open();
    }
    ++cur_seg;
    for (auto &e : encoders) {
      e->set_segment_num(cur_seg);
    }
#if defined(RK3588)
    for (auto &e : encoders) {
      static_cast<MppEncoder *>(e.get())->set_flush_pending_on_close(true);
    }
#endif
    return true;
  };

  while (!do_exit) {
    if (!vipc_client.connect(false)) {
      util::sleep_for(5);
      continue;
    }

    reset_sync_for_camera(s, cam_info.stream_type);

    const VisionBuf &buf_info = vipc_client.buffers[0];
    LOGW("encoder %s init %zux%zu", cam_info.thread_name, buf_info.width, buf_info.height);
    if (buf_info.width == 0 || buf_info.height == 0) {
      LOGE("encoder %s got invalid buffer dimensions %zux%zu, reconnecting",
           cam_info.thread_name, buf_info.width, buf_info.height);
      util::sleep_for(ENCODER_RECONNECT_BACKOFF_MS);
      continue;
    }

    // If camerad stream format changes, recreate encoder objects.
    if (!encoders.empty() && (buf_info.width != input_width || buf_info.height != input_height)) {
      LOGW("encoder %s stream dimensions changed %zux%zu -> %zux%zu, recreating encoders",
           cam_info.thread_name, input_width, input_height, buf_info.width, buf_info.height);
      close_and_reset_encoders(true);
    }

    // init encoders once and keep segment counters across vipc reconnects
    if (encoders.empty()) {
      input_width = buf_info.width;
      input_height = buf_info.height;
      for (const auto &encoder_info : cam_info.encoder_infos) {
        auto &e = encoders.emplace_back(new Encoder(encoder_info, buf_info.width, buf_info.height));
        e->encoder_open();
        output_recovery.emplace_back();
#if defined(RK3588)
        if (deterministic_mode) {
          auto *mpp = static_cast<MppEncoder *>(e.get());
          mpp->set_flush_pending_on_close(true);
          const bool open = mpp->encoder_is_open();
          const bool qroad = encoder_info.publish_name == std::string("qRoadEncodeData");
          const auto retry_now = open ? EncoderOutputRecovery::TimePoint{} : std::chrono::steady_clock::now();
          const auto grace_now = qroad ? std::chrono::steady_clock::now() : EncoderOutputRecovery::TimePoint{};
          const uint64_t packets = mpp->packet_published_count();
          const uint64_t submitted = mpp->input_submitted_count();
          output_recovery.back().reset_after_reopen(open, qroad, retry_now, grace_now, packets, submitted);
        }
#endif
        encode_failure_counts.emplace_back(0);
        encode_failure_burst_counts.emplace_back(0);
        encode_failure_last_log_tms.emplace_back(0.0);
      }

      // Only one thumbnail can be generated per camera stream
      // Thumbnail publishing is owned by camerad; encoderd never publishes thumbnail.
    } else {
      // Do not reopen on transient vipc reconnect; preserve segment continuity.
      for (size_t i = 0; i < encode_failure_counts.size(); ++i) {
        encode_failure_counts[i] = 0;
        encode_failure_burst_counts[i] = 0;
      }
    }

    bool lagging = false;
    double last_frame_seen_tms = millis_since_boot();
    double last_encode_success_tms = last_frame_seen_tms;
    double last_health_log_tms = last_frame_seen_tms;
    double last_segment_sync_tms = last_frame_seen_tms;
    uint32_t last_encoded_frame_id = 0;
    uint64_t encoded_frames = 0;
    uint64_t recoveries = 0;
    uint64_t dropped_by_encoder_contract = 0;
    uint64_t dropped_for_authority = 0;
    uint64_t dropped_waiting_rotation = 0;
    uint64_t phase_transitions = 0;
    const int frames_per_seg = SEGMENT_LENGTH * MAIN_FPS;
    bool loggerd_segment_baselined = false;
    int last_loggerd_segment = -1;
    int loggerd_segment_seen = -1;
    int loggerd_segment_epoch_seen = -1;
    std::string loggerd_rotate_state = "committed";
    SegmentPhase phase = SegmentPhase::Steady;
    bool vipc_reconnect_triggered = false;
    while (!do_exit) {
      VisionIpcBufExtra extra;
      VisionBuf* buf = vipc_client.recv(&extra);
      if (buf == nullptr) {
        const double now_tms = millis_since_boot();
        const bool stalled = (now_tms - last_frame_seen_tms) > ENCODER_VIPC_STALL_TIMEOUT_MS;
        if (stalled) {
          LOGE("encoder %s reconnecting vipc (%s, no frame for %.1f ms)",
               cam_info.thread_name,
               vipc_client.is_connected() ? "stalled" : "disconnected",
               (now_tms - last_frame_seen_tms));
          vipc_reconnect_triggered = true;
          break;
        }
        continue;
      }
      const double frame_now_tms = millis_since_boot();
      last_frame_seen_tms = frame_now_tms;

      // detect loop around and drop frames when buffer embeds frame_id.
      if (extra.valid && buf->get_frame_id() != extra.frame_id) {
        if (!lagging) {
          LOGE("encoder %s lag  buffer id: %" PRIu64 " extra id: %d", cam_info.thread_name, buf->get_frame_id(), extra.frame_id);
          lagging = true;
        }
        continue;
      }
      lagging = false;

      // KA2 follows loggerd's rotation handshake; frame-threshold synchronization
      // skips real input frames at startup and after each segment rollover.
      if (!deterministic_mode && !sync_encoders(s, cam_info.stream_type, extra.frame_id)) {
        continue;
      }
      if (do_exit) break;

      // Protocol-level sync: follow loggerd's authoritative segment counter.
      const double sync_now_tms = millis_since_boot();
      const int segment_sync_period_ms = deterministic_mode ?
        ENCODER_SEGMENT_SYNC_PERIOD_DETERMINISTIC_MS : ENCODER_SEGMENT_SYNC_PERIOD_MS;
      if ((sync_now_tms - last_segment_sync_tms) >= segment_sync_period_ms) {
        if (authority_reader) {
          std::string loggerd_segment_raw = params.get("LoggerdSegment");
          std::string loggerd_epoch_raw = params.get("LoggerdSegmentEpoch");
          std::string loggerd_rotate_state_raw = params.get("LoggerdRotateState");
          if (deterministic_mode) {
            if (loggerd_rotate_state_raw == "started") {
              s->loggerd_rotate_state.store(0);
            } else if (loggerd_rotate_state_raw == "committed") {
              s->loggerd_rotate_state.store(1);
            } else {
              LOGW_100("encoder %s ignoring invalid LoggerdRotateState value", cam_info.thread_name);
            }
            int parsed_epoch = -1;
            if (parse_nonnegative_int_strict(loggerd_epoch_raw, &parsed_epoch)) {
              s->loggerd_segment_epoch.store(parsed_epoch);
            } else {
              LOGW_100("encoder %s ignoring invalid LoggerdSegmentEpoch value", cam_info.thread_name);
            }
            int parsed_segment = -1;
            if (parse_nonnegative_int_strict(loggerd_segment_raw, &parsed_segment)) {
              s->loggerd_segment_authority.store(parsed_segment);
            } else {
              LOGW_100("encoder %s ignoring invalid LoggerdSegment value", cam_info.thread_name);
            }
          } else {
            if (!loggerd_rotate_state_raw.empty()) {
              s->loggerd_rotate_state.store((loggerd_rotate_state_raw == "started") ? 0 : 1);
            }
            if (!loggerd_epoch_raw.empty()) {
              try {
                s->loggerd_segment_epoch.store(std::stoi(loggerd_epoch_raw));
              } catch (const std::exception&) {}
            }
            if (!loggerd_segment_raw.empty()) {
              try {
                s->loggerd_segment_authority.store(std::stoi(loggerd_segment_raw));
              } catch (const std::exception&) {}
            }
          }
          if (deterministic_mode && s->loggerd_rotate_state.load() == 0 &&
              s->loggerd_segment_authority.load() >= 0) {
            s->forced_segment_drop_pending.store(false);
            update_max_atomic(s->forced_segment_target, s->loggerd_segment_authority.load() + 1);
          }
        }
        const bool was_rotation_barrier = loggerd_rotate_state == "started";
        const int shared_rotate_state = s->loggerd_rotate_state.load();
        loggerd_rotate_state = (shared_rotate_state == 0) ? "started" : "committed";
        if (deterministic_mode && was_rotation_barrier && loggerd_rotate_state == "committed") {
#if defined(RK3588)
          const auto now = std::chrono::steady_clock::now();
          for (size_t i = 0; i < output_recovery.size() && i < cam_info.encoder_infos.size(); ++i) {
            output_recovery[i].rotation_committed(
              now, cam_info.encoder_infos[i].publish_name == std::string("qRoadEncodeData"));
          }
#endif
        }
        loggerd_segment_epoch_seen = s->loggerd_segment_epoch.load();
        const int shared_loggerd_segment = s->loggerd_segment_authority.load();
        if (shared_loggerd_segment >= 0) {
          try {
            int loggerd_segment = shared_loggerd_segment;
            if (!loggerd_segment_baselined) {
              if (std::abs(loggerd_segment - cur_seg) <= 1) {
                loggerd_segment_baselined = true;
                last_loggerd_segment = loggerd_segment;
              } else {
                LOGW("encoder %s ignoring LoggerdSegment baseline %d (cur=%d)", cam_info.thread_name, loggerd_segment, cur_seg);
                last_segment_sync_tms = sync_now_tms;
                continue;
              }
            } else if (loggerd_segment > (last_loggerd_segment + 1) || loggerd_segment < (last_loggerd_segment - 1)) {
              LOGW("encoder %s ignoring LoggerdSegment jump %d -> %d", cam_info.thread_name, last_loggerd_segment, loggerd_segment);
              last_segment_sync_tms = sync_now_tms;
              continue;
            } else {
              last_loggerd_segment = loggerd_segment;
            }
            if (loggerd_segment > cur_seg) {
              // In deterministic KA2 mode, loggerd is authoritative by design.
              // Frame-id based plausibility can under-estimate segment progress when frames are dropped.
              if (!deterministic_mode) {
                // Guard against stale/out-of-band Params updates from another loggerd context.
                const int64_t frame_delta = (int64_t)extra.frame_id - (int64_t)s->start_frame_id.load();
                const int max_plausible_seg = frame_delta >= 0 ? (int)(frame_delta / frames_per_seg) : 0;
                if (loggerd_segment > (max_plausible_seg + 1)) {
                  LOGW("encoder %s ignoring implausible LoggerdSegment %d (cur=%d plausible<=%d frame=%u start=%u)",
                       cam_info.thread_name, loggerd_segment, cur_seg, max_plausible_seg + 1,
                       extra.frame_id, (uint32_t)s->start_frame_id.load());
                  last_segment_sync_tms = sync_now_tms;
                  continue;
                }
              }
              int observed_target = s->forced_segment_target.load();
              while (loggerd_segment > observed_target &&
                     !s->forced_segment_target.compare_exchange_weak(observed_target, loggerd_segment)) {
                // retry until target is raised
              }
              s->forced_segment_drop_pending.store(true);
            }
            loggerd_segment_seen = loggerd_segment;
          } catch (const std::exception&) {
            // Ignore malformed param value and continue.
          }
        }
        last_segment_sync_tms = sync_now_tms;
      }

      // Never publish too far into future loggerd segment; tighten for non-critical streams.
      const int max_ahead_segments = (deterministic_mode && !critical_stream &&
                                      s->forced_segment_drop_pending.load()) ? 0 : 1;
      if (loggerd_segment_seen >= 0 && cur_seg > (loggerd_segment_seen + max_ahead_segments)) {
        dropped_for_authority++;
        continue;
      }

      // Global forced sync: all stream threads converge to the same target segment.
      int forced_target = s->forced_segment_target.load();
      if (forced_target > cur_seg) {
        const bool drop_pending = deterministic_mode ? s->forced_segment_drop_pending.load() : true;
        SegmentPhase next_phase = drop_pending ? SegmentPhase::RecoveryBarrier : SegmentPhase::PreRotateBarrier;
        if (phase != next_phase) {
          phase = next_phase;
          phase_transitions++;
        }
        LOGW("encoder %s applying global segment catch-up %d -> %d phase=%s loggerd_state=%s epoch=%d",
             cam_info.thread_name, cur_seg, forced_target, phase_name(phase), loggerd_rotate_state.c_str(), loggerd_segment_epoch_seen);
        while (!do_exit && cur_seg < forced_target) {
          phase = SegmentPhase::RotateCommit;
#if defined(RK3588) && defined(LOGGERD_TEST_BUILD)
          if (deterministic_mode) {
            for (size_t i = 0; i < encoders.size() && i < output_recovery.size(); ++i) {
              if (output_recovery[i].active()) {
                static_cast<MppEncoder *>(encoders[i].get())->inject_test_drain_timeout(extra);
              }
            }
          }
#endif
          if (!advance_segment_once(drop_pending)) {
            do_exit = true;
            break;
          }
        }
        if (do_exit) break;
        phase = SegmentPhase::Steady;
        reset_sync_for_camera(s, cam_info.stream_type);
        update_max_atomic(s->start_frame_id, extra.frame_id);
      }

      // Non-deterministic fallback keeps local frame-threshold rotation.
      if (!deterministic_mode) {
        const int64_t next_rotate_threshold = ((int64_t)(cur_seg + 1) * frames_per_seg) + (int64_t)s->start_frame_id.load();
        if (cur_seg >= 0 && (int64_t)extra.frame_id >= next_rotate_threshold) {
          // If frame_id jumped far ahead (stall/reconnect), avoid cascading segment jumps.
          if ((int64_t)extra.frame_id >= next_rotate_threshold + frames_per_seg) {
            const uint32_t anchored_start_frame = extra.frame_id - ((cur_seg + 1) * frames_per_seg);
            LOGW("encoder %s re-anchoring segment timeline at frame %u (cur_seg=%d)",
                 cam_info.thread_name, extra.frame_id, cur_seg);
            s->start_frame_id = anchored_start_frame;
          }
          if (!advance_segment_once(false)) {
            do_exit = true;
            break;
          }
        }
      }

#if defined(RK3588)
      if (deterministic_mode) retry_unavailable_outputs();
#endif

      // encode a frame
      bool frame_encoded = false;
#if defined(RK3588)
      if (deterministic_mode) {
        for (size_t i = 0; i < encoders.size(); ++i) {
          auto &health = output_recovery[i];
          if (!health.active()) continue;
          auto *mpp = static_cast<MppEncoder *>(encoders[i].get());
          const int out_id = mpp->encode_frame(buf, &extra);
          const auto now = std::chrono::steady_clock::now();

          if (out_id == -2) {
            dropped_by_encoder_contract++;
          } else if (out_id == -1) {
            encode_failure_counts[i]++;
            encode_failure_burst_counts[i]++;
            health.note_input_error();
            const double now_tms = millis_since_boot();
            if ((now_tms - encode_failure_last_log_tms[i]) >= ENCODER_FAILURE_LOG_PERIOD_MS ||
                encode_failure_burst_counts[i] >= ENCODER_FAILURE_LOG_BATCH_SIZE) {
              LOGE("encoder %s output %s failures=%d consecutive=%d last_frame=%u",
                   cam_info.thread_name, cam_info.encoder_infos[i].publish_name,
                   encode_failure_burst_counts[i], health.consecutive_input_errors, extra.frame_id);
              encode_failure_last_log_tms[i] = now_tms;
              encode_failure_burst_counts[i] = 0;
            }
            if (health.input_error_limit_reached(ENCODER_REOPEN_FAILURE_THRESHOLD)) {
              schedule_output_retry(i, "consecutive input error threshold");
              continue;
            }
          } else {
            encode_failure_counts[i] = 0;
            encode_failure_burst_counts[i] = 0;
            health.note_input_success();
          }

          health.observe_submitted(mpp->input_submitted_count(), now);
          if (health.observe_published(mpp->packet_published_count(), now)) frame_encoded = true;
          health.observe_pending(mpp->pending_frame_count(), now);

          const bool in_rotation_barrier = s->loggerd_rotate_state.load() == 0;
          if (health.packet_watchdog_expired(now, in_rotation_barrier)) {
            schedule_output_retry(i, "packet progress watchdog");
          }
        }
      } else
#endif
      {
      for (size_t i = 0; i < encoders.size(); ++i) {
        int out_id = encoders[i]->encode_frame(buf, &extra);

        if (out_id == -2) {
          dropped_by_encoder_contract++;
          continue;
        } else if (out_id == -1) {
          encode_failure_counts[i]++;
          encode_failure_burst_counts[i]++;
          const double now_tms = millis_since_boot();
          if ((now_tms - encode_failure_last_log_tms[i]) >= ENCODER_FAILURE_LOG_PERIOD_MS ||
              encode_failure_burst_counts[i] >= ENCODER_FAILURE_LOG_BATCH_SIZE) {
            LOGE("encoder %s stream %zu failures=%d consecutive=%d last_frame=%d",
                 cam_info.thread_name, i, encode_failure_burst_counts[i], encode_failure_counts[i], extra.frame_id);
            encode_failure_last_log_tms[i] = now_tms;
            encode_failure_burst_counts[i] = 0;
          }
          if (encode_failure_counts[i] >= ENCODER_REOPEN_FAILURE_THRESHOLD) {
            LOGE("encoder %s stream %zu exceeded failure threshold (%d), reopening",
                 cam_info.thread_name, i, ENCODER_REOPEN_FAILURE_THRESHOLD);
            reopen_all_encoders();
            encode_failure_counts[i] = 0;
            encode_failure_burst_counts[i] = 0;
            recoveries++;
          }
        } else {
          encode_failure_counts[i] = 0;
          encode_failure_burst_counts[i] = 0;
          frame_encoded = true;
        }
      }
      }
      const double now_tms = millis_since_boot();
#if defined(LOGGERD_TEST_BUILD)
      if (test_runtime_enabled && frame_encoded) test_all_output_reopen_tracking = true;
#endif
      if (frame_encoded) {
        last_encode_success_tms = now_tms;
        last_encoded_frame_id = extra.frame_id;
        encoded_frames++;
      }
#if defined(RK3588)
      else if (!deterministic_mode && (now_tms - last_encode_success_tms) > ENCODER_OUTPUT_STALL_TIMEOUT_MS) {
#else
      else if ((now_tms - last_encode_success_tms) > ENCODER_OUTPUT_STALL_TIMEOUT_MS) {
#endif
        LOGE("encoder %s output stalled for %.1f ms, reopening all streams",
             cam_info.thread_name, (now_tms - last_encode_success_tms));
        reopen_all_encoders();
        last_encode_success_tms = now_tms;
        recoveries++;
      }

      if ((now_tms - last_health_log_tms) > ENCODER_HEALTH_LOG_PERIOD_MS) {
        LOGD("encoder %s health: encoded=%" PRIu64 " last_frame=%u recoveries=%" PRIu64
             " phase=%s phase_transitions=%" PRIu64 " drop_authority=%" PRIu64
             " drop_wait_rotate=%" PRIu64 " drop_contract=%" PRIu64 " loggerd_seg=%d loggerd_epoch=%d",
             cam_info.thread_name, encoded_frames, last_encoded_frame_id, recoveries, phase_name(phase),
             phase_transitions, dropped_for_authority, dropped_waiting_rotation, dropped_by_encoder_contract,
             loggerd_segment_seen, loggerd_segment_epoch_seen);
        last_health_log_tms = now_tms;
      }

      if (jpeg_encoder && (extra.frame_id % 1200 == 100)) {
        jpeg_encoder->pushThumbnail(buf, extra);
      }
    }

    // Keep encoders open across reconnects to avoid segment-number resets.
    if (!do_exit) {
      if (vipc_reconnect_triggered && !encoders.empty()) {
        // Reconnect recovery: reopen pipelines and drop in-flight stale packets.
        reopen_all_encoders();
      }
      util::sleep_for(ENCODER_RECONNECT_BACKOFF_MS);
    }
  }
  close_and_reset_encoders(true);
#if defined(LOGGERD_TEST_BUILD)
  if (test_runtime_enabled) {
    LOGW("LOGGERD_TEST_ALL_OUTPUT_REOPEN_COUNT thread=%s count=%llu",
         cam_info.thread_name, static_cast<unsigned long long>(test_all_output_reopen_count));
  }
#endif
}

template <size_t N>
void encoderd_thread(const LogCameraInfo (&cameras)[N]) {
  EncoderdState s;

  std::set<VisionStreamType> streams;
  while (!do_exit) {
    streams = VisionIpcClient::getAvailableStreams("camerad", false);
    if (!streams.empty()) {
      break;
    }
    util::sleep_for(100);
  }

  if (!streams.empty()) {
    std::vector<std::thread> encoder_threads;
    for (auto stream : streams) {
      auto it = std::find_if(std::begin(cameras), std::end(cameras),
                             [stream](auto &cam) { return cam.stream_type == stream; });
      assert(it != std::end(cameras));
      ++s.max_waiting;
      encoder_threads.push_back(std::thread(encoder_thread, &s, *it));
    }

    for (auto &t : encoder_threads) t.join();
  }
}

int main(int argc, char* argv[]) {
  if (!Hardware::PC()) {
#if defined(RK3588)
    const bool deterministic_mode =
      !(getenv("ENCODERD_DETERMINISTIC_DISABLE") != nullptr && atoi(getenv("ENCODERD_DETERMINISTIC_DISABLE")) != 0);
#endif
    int ret = util::set_realtime_priority(
#if defined(RK3588)
      deterministic_mode ? 60 : 52
#else
      52
#endif
    );
    if (ret != 0) {
      LOGW("failed to set encoderd realtime priority: %d", ret);
    }

    std::vector<int> affinity_cores = parse_affinity_cores(getenv("ENCODERD_AFFINITY"));
    if (affinity_cores.empty()) {
      // Avoid single-core pinning; default to a wider core set for better tail latency.
#if defined(RK3588)
      affinity_cores = deterministic_mode ? std::vector<int>({2, 3, 4, 5, 6, 7}) : std::vector<int>({2, 3, 4, 5});
#else
      affinity_cores = {2, 3, 4, 5};
#endif
    }
    ret = util::set_core_affinity(affinity_cores);
    if (ret != 0) {
      LOGW("failed to set encoderd core affinity");
    }
  }
  if (argc > 1) {
    std::string arg1(argv[1]);
    if (arg1 == "--stream") {
      encoderd_thread(stream_cameras_logged);
    } else {
      LOGE("Argument '%s' is not supported", arg1.c_str());
    }
  } else {
    encoderd_thread(cameras_logged);
  }
  return 0;
}
