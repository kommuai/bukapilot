#include <cassert>
#include <cstdlib>
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
  const bool deterministic_mode =
#if defined(RK3588)
    !(getenv("ENCODERD_DETERMINISTIC_DISABLE") != nullptr && atoi(getenv("ENCODERD_DETERMINISTIC_DISABLE")) != 0);
#else
    false;
#endif
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
      jpeg_encoder.reset();
      cur_seg = 0;
      input_width = 0;
      input_height = 0;
    }
    reset_sync_for_camera(s, cam_info.stream_type);
  };

  auto reopen_all_encoders = [&]() {
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
    }
#if defined(RK3588)
    for (auto &e : encoders) {
      static_cast<MppEncoder *>(e.get())->set_flush_pending_on_close(true);
    }
#endif
  };

  auto advance_segment_once = [&](bool drop_pending) {
#if defined(RK3588)
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

      if (!sync_encoders(s, cam_info.stream_type, extra.frame_id)) {
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
          if (!loggerd_rotate_state_raw.empty()) {
            const int rotate_state = (loggerd_rotate_state_raw == "started") ? 0 : 1;
            s->loggerd_rotate_state.store(rotate_state);
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
        const int shared_rotate_state = s->loggerd_rotate_state.load();
        loggerd_rotate_state = (shared_rotate_state == 0) ? "started" : "committed";
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
      const int max_ahead_segments = (deterministic_mode && !critical_stream) ? 0 : 1;
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
          advance_segment_once(drop_pending);
        }
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
          advance_segment_once(false);
        }
      }

      // encode a frame
      bool frame_encoded = false;
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
      const double now_tms = millis_since_boot();
      if (frame_encoded) {
        last_encode_success_tms = now_tms;
        last_encoded_frame_id = extra.frame_id;
        encoded_frames++;
      } else if ((now_tms - last_encode_success_tms) > ENCODER_OUTPUT_STALL_TIMEOUT_MS) {
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
