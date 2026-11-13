#include <sys/xattr.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/params.h"
#include "system/loggerd/encoder/encoder.h"
#include "system/loggerd/loggerd.h"
#include "system/loggerd/memory_pressure.h"
#include "system/loggerd/video_writer.h"

ExitHandler do_exit;

constexpr int DETERMINISTIC_ROTATION_DEADLINE_MS = 5000;
constexpr int DETERMINISTIC_ROTATION_POLL_MS = 50;
constexpr size_t DETERMINISTIC_AUDIO_QUEUE_MAX_BYTES = 256 * 1024;
constexpr auto DETERMINISTIC_AUDIO_MAX_HOLD = std::chrono::seconds(5);

#if defined(LOGGERD_TEST_BUILD)
namespace {
std::atomic<int64_t> test_monotonic_ms{0};
std::atomic<bool> test_monotonic_enabled{false};
const auto test_monotonic_origin = std::chrono::steady_clock::now();

bool loggerd_test_runtime_enabled() {
  const char *test_env = getenv("LOGGERD_TEST");
  return test_env != nullptr && std::strcmp(test_env, "1") == 0;
}
}

void loggerd_test_enable_monotonic_clock() {
  if (loggerd_test_runtime_enabled()) test_monotonic_enabled.store(true, std::memory_order_release);
}

void loggerd_test_advance_monotonic_ms(int64_t delta_ms) {
  if (loggerd_test_runtime_enabled() && test_monotonic_enabled.load(std::memory_order_acquire) && delta_ms > 0) {
    test_monotonic_ms.fetch_add(delta_ms, std::memory_order_relaxed);
  }
}

int64_t loggerd_test_monotonic_ms() {
  return loggerd_test_runtime_enabled() && test_monotonic_enabled.load(std::memory_order_acquire) ?
    test_monotonic_ms.load(std::memory_order_relaxed) : 0;
}
#endif

std::chrono::steady_clock::time_point loggerd_monotonic_now() {
#if defined(LOGGERD_TEST_BUILD)
  if (loggerd_test_runtime_enabled() && test_monotonic_enabled.load(std::memory_order_acquire)) {
    return test_monotonic_origin + std::chrono::milliseconds(loggerd_test_monotonic_ms());
  }
#endif
  return std::chrono::steady_clock::now();
}

struct LoggerdDropCounts {
  uint64_t stale_video_messages = 0;
  uint64_t future_video_queue_drops = 0;
  uint64_t audio_no_writer_events = 0;
  uint64_t audio_expired_events = 0;
  uint64_t audio_capacity_drops = 0;

  bool any() const {
    return stale_video_messages != 0 || future_video_queue_drops != 0 ||
           audio_no_writer_events != 0 || audio_expired_events != 0 || audio_capacity_drops != 0;
  }
};

struct LoggerdState {
  LoggerState logger;
  std::atomic<double> last_camera_seen_tms{0.0};
  std::atomic<int> ready_to_rotate{0};  // count of encoders ready to rotate
  int max_waiting = 0;
  double last_rotate_tms = 0.;      // last rotate time in ms
  int rotate_epoch = 0;
  bool rotation_started = false;
  std::chrono::steady_clock::time_point rotation_started_at{};
  std::chrono::steady_clock::time_point last_rotate_monotonic{};
  LoggerdDropCounts drop_counts;
};

const char *const deterministic_test_env = LOGGERD_TEST ? getenv("LOGGERD_TEST_DETERMINISTIC") : nullptr;
const bool deterministic_test_mode = LOGGERD_TEST && deterministic_test_env != nullptr && atoi(deterministic_test_env) == 1;

bool deterministic_ka2_mode() {
#if defined(RK3588)
  const char *disable = getenv("ENCODERD_DETERMINISTIC_DISABLE");
  return !(disable != nullptr && atoi(disable) != 0);
#else
  return false;
#endif
}

bool deterministic_ka2_protocol_active() {
  return deterministic_ka2_mode() && (!LOGGERD_TEST || deterministic_test_mode);
}

void publish_rotate_state(LoggerdState *s, const char *state) {
  Params params;
  params.putNonBlocking("LoggerdRotateState", state);
  params.putNonBlocking("LoggerdSegment", std::to_string(s->logger.segment()));
  params.putNonBlocking("LoggerdSegmentEpoch", std::to_string(s->rotate_epoch));
}

void logger_rotate(LoggerdState *s) {
  bool ret =s->logger.next();
  assert(ret);
  s->ready_to_rotate = 0;
  s->rotation_started = false;
  s->rotation_started_at = {};
  s->last_rotate_tms = millis_since_boot();
  s->last_rotate_monotonic = loggerd_monotonic_now();
  s->rotate_epoch++;
  publish_rotate_state(s, "committed");
  LOGW((s->logger.segment() == 0) ? "logging to %s" : "rotated to %s", s->logger.segmentPath().c_str());
}

bool rotate_if_needed(LoggerdState *s,
                      std::chrono::steady_clock::time_point *commit_decision_time = nullptr) {
  double tms = millis_since_boot();
  double seg_length_secs = (tms - s->last_rotate_tms) / 1000.;
  const bool deterministic_mode = deterministic_ka2_protocol_active();
  bool all_ready = s->ready_to_rotate == s->max_waiting;

  if (deterministic_mode) {
    const auto monotonic_now = loggerd_monotonic_now();
    seg_length_secs = std::chrono::duration<double>(monotonic_now - s->last_rotate_monotonic).count();
    // Let encoders drain the old segment before committing the clock boundary.
    if (!s->rotation_started && seg_length_secs >= SEGMENT_LENGTH) {
      s->rotation_started_at = monotonic_now;
      s->rotation_started = true;
      publish_rotate_state(s, "started");
    }
    const bool deadline_reached = s->rotation_started &&
      monotonic_now - s->rotation_started_at >=
        std::chrono::milliseconds(DETERMINISTIC_ROTATION_DEADLINE_MS);
    const bool commit_ready = s->rotation_started && (all_ready || deadline_reached);
    if (commit_ready && commit_decision_time != nullptr) *commit_decision_time = monotonic_now;
    return commit_ready;
  }

  // all encoders ready, trigger rotation

  // fallback logic to prevent extremely long segments in the case of camera, encoder, etc. malfunctions
  bool timed_out = false;
  if ((seg_length_secs > SEGMENT_LENGTH) && !LOGGERD_TEST) {
    // TODO: might be nice to put these reasons in the sentinel
    if ((tms - s->last_camera_seen_tms) > NO_CAMERA_PATIENCE) {
      timed_out = true;
      LOGE("no camera packets seen. auto rotating");
    } else if (seg_length_secs > SEGMENT_LENGTH*1.2) {
      timed_out = true;
      LOGE("segment too long. auto rotating");
    }
  }

  if (all_ready || timed_out) {
    publish_rotate_state(s, "started");
    logger_rotate(s);
  }
  return false;
}

struct RemoteEncoder {
  std::unique_ptr<VideoWriter> writer;
  int encoderd_segment_offset = 0;
  int current_segment = -1;
  std::vector<Message *> q;
  int dropped_frames = 0;
  bool recording = false;
  bool marked_ready_to_rotate = false;
  bool seen_first_packet = false;
  bool audio_initialized = false;
};

struct QueuedAudioEvent {
  int segment = -1;
  uint64_t log_mono_time = 0;
  uint32_t sample_rate = 0;
  std::chrono::steady_clock::time_point expires_at{};
  std::vector<uint8_t> data;
};

struct LoggerdAudioQueue {
  std::deque<QueuedAudioEvent> events;
  size_t bytes = 0;
};

std::string json_escape(const std::string &value) {
  std::string escaped;
  escaped.reserve(value.size() + 8);
  for (unsigned char c : value) {
    switch (c) {
      case '"': escaped += "\\\""; break;
      case '\\': escaped += "\\\\"; break;
      case '\b': escaped += "\\b"; break;
      case '\f': escaped += "\\f"; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default:
        if (c < 0x20) {
          static constexpr char hex[] = "0123456789abcdef";
          escaped += "\\u00";
          escaped += hex[(c >> 4) & 0xf];
          escaped += hex[c & 0xf];
        } else {
          escaped += static_cast<char>(c);
        }
    }
  }
  return escaped;
}

void expire_queued_audio(LoggerdState *s, LoggerdAudioQueue *queue,
                         std::chrono::steady_clock::time_point now) {
  for (auto it = queue->events.begin(); it != queue->events.end();) {
    if (now >= it->expires_at) {
      queue->bytes -= it->data.size();
      s->drop_counts.audio_expired_events++;
      LOGW("rawAudioData event expired before qRoad writer opened (segment %d)", it->segment);
      it = queue->events.erase(it);
    } else {
      ++it;
    }
  }
}

void write_audio_event(RemoteEncoder *remote, QueuedAudioEvent &audio) {
  if (remote == nullptr || !remote->writer || remote->current_segment != audio.segment) return;
  remote->writer->write_audio(audio.data.data(), audio.data.size(), audio.log_mono_time / 1000, audio.sample_rate);
  remote->audio_initialized = true;
}

void write_audio_payload(RemoteEncoder *remote, int segment, uint64_t log_mono_time,
                         uint32_t sample_rate, const uint8_t *data, size_t size) {
  if (remote == nullptr || !remote->writer || remote->current_segment != segment) return;
  remote->writer->write_audio(const_cast<uint8_t *>(data), size, log_mono_time / 1000, sample_rate);
  remote->audio_initialized = true;
}

void flush_queued_audio_for_segment(LoggerdState *s, LoggerdAudioQueue *queue,
                                    RemoteEncoder *remote, int segment,
                                    std::chrono::steady_clock::time_point now) {
  expire_queued_audio(s, queue, now);
  if (remote == nullptr || !remote->writer || remote->current_segment != segment) return;

  for (auto it = queue->events.begin(); it != queue->events.end();) {
    if (it->segment == segment) {
      write_audio_event(remote, *it);
      queue->bytes -= it->data.size();
      it = queue->events.erase(it);
    } else if (it->segment < segment) {
      queue->bytes -= it->data.size();
      s->drop_counts.audio_no_writer_events++;
      LOGW("rawAudioData event discarded at segment commit without a matching qRoad writer (segment %d)", it->segment);
      it = queue->events.erase(it);
    } else {
      ++it;
    }
  }
}

void admit_audio_event(LoggerdState *s, LoggerdAudioQueue *queue, RemoteEncoder *remote,
                       int segment, uint64_t log_mono_time, uint32_t sample_rate,
                       const uint8_t *data, size_t size) {
  const auto now = loggerd_monotonic_now();
  expire_queued_audio(s, queue, now);
  if (remote != nullptr && remote->writer && remote->current_segment == segment) {
    write_audio_payload(remote, segment, log_mono_time, sample_rate, data, size);
    return;
  }
  if (queue->bytes > DETERMINISTIC_AUDIO_QUEUE_MAX_BYTES ||
      size > DETERMINISTIC_AUDIO_QUEUE_MAX_BYTES - queue->bytes) {
    s->drop_counts.audio_capacity_drops++;
    LOGW("rawAudioData event rejected atomically at 256 KiB qRoad queue cap (segment %d, %zu bytes)", segment, size);
    return;
  }

  QueuedAudioEvent audio = {
    .segment = segment,
    .log_mono_time = log_mono_time,
    .sample_rate = sample_rate,
    .expires_at = now + DETERMINISTIC_AUDIO_MAX_HOLD,
    .data = {},
  };
  if (size != 0) audio.data.assign(data, data + size);

  auto insert_at = std::upper_bound(queue->events.begin(), queue->events.end(), audio.log_mono_time,
    [](uint64_t timestamp, const QueuedAudioEvent &queued) { return timestamp < queued.log_mono_time; });
  queue->events.insert(insert_at, std::move(audio));
  queue->bytes += size;
}

std::string make_rotation_report(const std::string &route, int epoch, int segment, bool degraded,
                                const std::vector<std::string> &missing_streams, int64_t elapsed_ms,
                                const LoggerdDropCounts &drops) {
  std::string report = "{\"route\":\"" + json_escape(route) + "\",\"epoch\":" + std::to_string(epoch) +
    ",\"segment\":" + std::to_string(segment) + ",\"degraded\":" + (degraded ? "true" : "false") +
    ",\"missing_streams\":[";
  for (size_t i = 0; i < missing_streams.size(); ++i) {
    if (i != 0) report += ",";
    report += "\"" + json_escape(missing_streams[i]) + "\"";
  }
  report += "],\"elapsed_ms\":" + std::to_string(std::max<int64_t>(elapsed_ms, 0)) +
    ",\"known_drop_counts\":{\"stale_video_messages\":" + std::to_string(drops.stale_video_messages) +
    ",\"future_video_queue_drops\":" + std::to_string(drops.future_video_queue_drops) +
    ",\"audio_no_writer_events\":" + std::to_string(drops.audio_no_writer_events) +
    ",\"audio_expired_events\":" + std::to_string(drops.audio_expired_events) +
    ",\"audio_capacity_drops\":" + std::to_string(drops.audio_capacity_drops) + "}}";
  return report;
}

void commit_deterministic_runtime_rotation(LoggerdState *s,
                                          const std::vector<std::string> &configured_streams,
                                          const std::unordered_map<std::string, RemoteEncoder *> &remote_by_name,
                                          const std::map<std::string, EncoderInfo> &encoder_infos,
                                          LoggerdAudioQueue *audio_queue,
                                          RemoteEncoder *audio_encoder,
                                          Params *params,
                                          std::chrono::steady_clock::time_point decision_time) {
  const int64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    decision_time - s->rotation_started_at).count();
  const int old_segment = s->logger.segment();
  const int destination_segment = old_segment + 1;
  std::vector<std::string> missing_streams;
  for (const auto &name : configured_streams) {
    auto it = remote_by_name.find(name);
    if (it == remote_by_name.end() || it->second == nullptr || !it->second->marked_ready_to_rotate) {
      missing_streams.push_back(name);
    }
  }

  flush_queued_audio_for_segment(s, audio_queue, audio_encoder, old_segment, decision_time);
  for (auto it = audio_queue->events.begin(); it != audio_queue->events.end();) {
    if (it->segment < destination_segment) {
      audio_queue->bytes -= it->data.size();
      s->drop_counts.audio_no_writer_events++;
      LOGW("rawAudioData event discarded at segment commit without a matching qRoad writer (segment %d)", it->segment);
      it = audio_queue->events.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto &name : configured_streams) {
    auto remote_it = remote_by_name.find(name);
    if (remote_it == remote_by_name.end() || remote_it->second == nullptr) continue;
    RemoteEncoder &remote = *remote_it->second;
    auto info_it = encoder_infos.find(name);
    if (info_it != encoder_infos.end()) {
      std::vector<Message *> retained;
      retained.reserve(remote.q.size());
      for (Message *queued_msg : remote.q) {
        bool keep = false;
        try {
          capnp::FlatArrayMessageReader reader({
            (capnp::word *)queued_msg->getData(), queued_msg->getSize() / sizeof(capnp::word)});
          const auto event = reader.getRoot<cereal::Event>();
          const auto edata = (event.*(info_it->second.get_encode_data_func))();
          const int mapped_segment = edata.getIdx().getSegmentNum() - remote.encoderd_segment_offset;
          if (mapped_segment < destination_segment) {
            s->drop_counts.stale_video_messages++;
            LOGD("%s: dropping queued stale packet at runtime commit (seg %d, destination %d)",
                 name.c_str(), mapped_segment, destination_segment);
          } else {
            keep = true;
          }
        } catch (const kj::Exception &e) {
          LOGE("%s: dropping malformed queued encode packet at runtime commit (%s)",
               name.c_str(), e.getDescription().cStr());
        }
        if (keep) retained.push_back(queued_msg);
        else delete queued_msg;
      }
      remote.q.swap(retained);
    }
    remote.writer.reset();
    remote.current_segment = -1;
    remote.recording = false;
    remote.audio_initialized = false;
    remote.marked_ready_to_rotate = false;
    remote.dropped_frames = 0;
  }

  logger_rotate(s);
  const bool degraded = !missing_streams.empty() || s->drop_counts.any();
  std::string route = params->get("CurrentRoute");
  if (route.empty()) route = s->logger.routeName();
  const std::string report = make_rotation_report(route, s->rotate_epoch, s->logger.segment(), degraded,
                                                 missing_streams, elapsed_ms, s->drop_counts);
  params->putNonBlocking("LoggerdRotationReport", report);
  s->drop_counts = {};
}

size_t write_encode_data(LoggerdState *s, cereal::Event::Reader event, RemoteEncoder &re, const EncoderInfo &encoder_info) {
  auto edata = (event.*(encoder_info.get_encode_data_func))();
  auto idx = edata.getIdx();
  auto flags = idx.getFlags();

  // if we aren't recording yet, try to start, since we are in the correct segment
  if (!re.recording) {
    if (flags & V4L2_BUF_FLAG_KEYFRAME) {
      // only create on iframe
      if (re.dropped_frames) {
        // this should only happen for the first segment, maybe
        LOGW("%s: dropped %d non iframe packets before init", encoder_info.publish_name, re.dropped_frames);
        re.dropped_frames = 0;
      }
      if (encoder_info.record) {
        // write the header
        auto header = edata.getHeader();
        re.writer->write((uint8_t *)header.begin(), header.size(), idx.getTimestampEof() / 1000, true, false);
      }
      re.recording = true;
    } else {
      // this is a sad case when we aren't recording, but don't have an iframe
      // nothing we can do but drop the frame
      ++re.dropped_frames;
      return 0;
    }
  }

  // we have to be recording if we are here
  assert(re.recording);

  // if we are actually writing the video file, do so
  if (re.writer) {
    auto data = edata.getData();
    re.writer->write((uint8_t *)data.begin(), data.size(), idx.getTimestampEof() / 1000, false, flags & V4L2_BUF_FLAG_KEYFRAME);
  }

  // put it in log stream as the idx packet
  MessageBuilder bmsg;
  auto evt = bmsg.initEvent(event.getValid());
  evt.setLogMonoTime(event.getLogMonoTime());
  (evt.*(encoder_info.set_encode_idx_func))(idx);
  auto new_msg = bmsg.toBytes();
  s->logger.write((uint8_t *)new_msg.begin(), new_msg.size(), true);  // always in qlog?
  return new_msg.size();
}

int handle_encoder_msg(LoggerdState *s, Message *msg, std::string &name, struct RemoteEncoder &re, const EncoderInfo &encoder_info) {
  int bytes_count = 0;
  const bool deterministic_mode = deterministic_ka2_protocol_active();
  const size_t max_encoder_queue = MAIN_FPS * 10;
  auto enqueue_bounded = [&](Message *m, const char *reason) {
    if (re.q.size() >= max_encoder_queue) {
      // Keep freshest packets under prolonged skew/stall; stale packets have lower recovery value.
      Message *oldest = re.q.front();
      if (deterministic_mode) {
        try {
          capnp::FlatArrayMessageReader reader({
            (capnp::word *)oldest->getData(), oldest->getSize() / sizeof(capnp::word)});
          const auto queued_event = reader.getRoot<cereal::Event>();
          const auto queued_data = (queued_event.*(encoder_info.get_encode_data_func))();
          const int queued_segment = queued_data.getIdx().getSegmentNum() - re.encoderd_segment_offset;
          if (queued_segment > s->logger.segment()) s->drop_counts.future_video_queue_drops++;
        } catch (const kj::Exception &) {
          // Malformed queue entries are discarded without inventing a video drop reason.
        }
      }
      delete oldest;
      re.q.erase(re.q.begin());
      LOGW_100("%s: encoder queue full (%zu), dropping oldest while %s", name.c_str(), re.q.size(), reason);
    }
    re.q.push_back(m);
  };

  std::unique_ptr<capnp::FlatArrayMessageReader> cmsg;
  cereal::Event::Reader event;
  decltype((std::declval<cereal::Event::Reader>().*(encoder_info.get_encode_data_func))()) edata;
  decltype(edata.getIdx()) idx;
  try {
    // Extract and validate incoming capnp payload. Drop malformed messages instead of aborting loggerd.
    cmsg = std::make_unique<capnp::FlatArrayMessageReader>(
      kj::ArrayPtr<capnp::word>((capnp::word *)msg->getData(), msg->getSize() / sizeof(capnp::word)));
    event = cmsg->getRoot<cereal::Event>();
    edata = (event.*(encoder_info.get_encode_data_func))();
    idx = edata.getIdx();
  } catch (const kj::Exception &e) {
    LOGE("%s: dropping malformed encode packet (%s)", name.c_str(), e.getDescription().cStr());
    delete msg;
    return 0;
  }

  // encoderd can have started long before loggerd
  if (!re.seen_first_packet) {
    re.seen_first_packet = true;
    re.encoderd_segment_offset = idx.getSegmentNum();
    LOGD("%s: has encoderd offset %d", name.c_str(), re.encoderd_segment_offset);
  }
  int offset_segment_num = idx.getSegmentNum() - re.encoderd_segment_offset;

  if (offset_segment_num == s->logger.segment()) {
    // loggerd is now on the segment that matches this packet

    // if this is a new segment, we close any possible old segments, move to the new, and process any queued packets
    if (re.current_segment != s->logger.segment()) {
      // if we aren't actually recording, don't create the writer
      if (encoder_info.record) {
        assert(encoder_info.filename != NULL);
        re.writer.reset(new VideoWriter(s->logger.segmentPath().c_str(),
                                        encoder_info.filename, idx.getType() != cereal::EncodeIndex::Type::FULL_H_E_V_C,
                                        edata.getWidth(), edata.getHeight(), encoder_info.fps, idx.getType()));
        re.recording = false;
        re.audio_initialized = false;
      }
      re.current_segment = s->logger.segment();
      re.marked_ready_to_rotate = false;
    }
    if (re.audio_initialized || !encoder_info.include_audio) {
      // we are in this segment now, process any queued messages before this one
      if (!re.q.empty()) {
        if (deterministic_mode) {
          std::vector<Message *> future_messages;
          future_messages.reserve(re.q.size());
          for (auto qmsg : re.q) {
            bool keep = false;
            try {
              capnp::FlatArrayMessageReader reader({(capnp::word *)qmsg->getData(), qmsg->getSize() / sizeof(capnp::word)});
              const auto queued_event = reader.getRoot<cereal::Event>();
              const auto queued_data = (queued_event.*(encoder_info.get_encode_data_func))();
              const int queued_segment = queued_data.getIdx().getSegmentNum() - re.encoderd_segment_offset;
              if (queued_segment == s->logger.segment()) {
                bytes_count += write_encode_data(s, queued_event, re, encoder_info);
              } else if (queued_segment < s->logger.segment()) {
                s->drop_counts.stale_video_messages++;
                LOGD("%s: dropping stale queued packet (seg %d, loggerd seg %d)",
                     name.c_str(), queued_segment, s->logger.segment());
              } else {
                keep = true;
              }
            } catch (const kj::Exception &e) {
              LOGE("%s: dropping malformed queued encode packet (%s)", name.c_str(), e.getDescription().cStr());
            }
            if (keep) future_messages.push_back(qmsg);
            else delete qmsg;
          }
          re.q.swap(future_messages);
        } else {
          for (auto qmsg : re.q) {
            try {
              capnp::FlatArrayMessageReader reader({(capnp::word *)qmsg->getData(), qmsg->getSize() / sizeof(capnp::word)});
              bytes_count += write_encode_data(s, reader.getRoot<cereal::Event>(), re, encoder_info);
            } catch (const kj::Exception &e) {
              LOGE("%s: dropping malformed queued encode packet (%s)", name.c_str(), e.getDescription().cStr());
            }
            delete qmsg;
          }
          re.q.clear();
        }
      }
      bytes_count += write_encode_data(s, event, re, encoder_info);
      delete msg;
    } else {
      enqueue_bounded(msg, "waiting for audio initialization");
    }
  } else if (offset_segment_num > s->logger.segment()) {
    // encoderd packet has a newer segment, this means encoderd has rolled over
    if ((!deterministic_mode || s->rotation_started) && !re.marked_ready_to_rotate) {
      re.marked_ready_to_rotate = true;
      ++s->ready_to_rotate;
      LOGD("rotate %d -> %d ready %d/%d for %s",
        s->logger.segment(), offset_segment_num,
        s->ready_to_rotate.load(), s->max_waiting, name.c_str());
    }

    // Queue up all the new-segment messages, but bound memory/latency by dropping oldest first.
    enqueue_bounded(msg, "waiting for logger rotate");
  } else {
    if (!deterministic_mode) {
      // Non-deterministic: re-anchor offset so subsequent packets align.
      LOGE("%s: encoderd packet has a older segment!!! idx.getSegmentNum():%d s->logger.segment():%d re.encoderd_segment_offset:%d",
        name.c_str(), idx.getSegmentNum(), s->logger.segment(), re.encoderd_segment_offset);
      re.encoderd_segment_offset = idx.getSegmentNum() - s->logger.segment();
    } else {
      // Deterministic KA2: encoderd will catch up via Params within ~100ms.
      // Re-anchoring the offset here causes permanent drift (-1, -2, -3...)
      // because each rotation window produces a few stale packets.
      LOGD("%s: dropping stale packet (seg %d, loggerd seg %d, offset %d)",
        name.c_str(), idx.getSegmentNum(), s->logger.segment(), re.encoderd_segment_offset);
      s->drop_counts.stale_video_messages++;
    }
    delete msg;
  }

  return bytes_count;
}

void handle_preserve_segment(LoggerdState *s) {
  static int prev_segment = -1;
  if (s->logger.segment() == prev_segment) return;

  LOGW("preserving %s", s->logger.segmentPath().c_str());

#ifdef __APPLE__
  int ret = setxattr(s->logger.segmentPath().c_str(), PRESERVE_ATTR_NAME, &PRESERVE_ATTR_VALUE, 1, 0, 0);
#else
  int ret = setxattr(s->logger.segmentPath().c_str(), PRESERVE_ATTR_NAME, &PRESERVE_ATTR_VALUE, 1, 0);
#endif
  if (ret) {
    LOGE("setxattr %s failed for %s: %s", PRESERVE_ATTR_NAME, s->logger.segmentPath().c_str(), strerror(errno));
  }

  // mark route for uploading
  Params params;
  std::string routes = params.get("AthenadRecentlyViewedRoutes");
  params.put("AthenadRecentlyViewedRoutes", routes + "," + s->logger.routeName());

  prev_segment = s->logger.segment();
}

void loggerd_thread() {
  // setup messaging
  typedef struct ServiceState {
    std::string name;
    int counter, freq;
    bool encoder, preserve_segment, record_audio;
  } ServiceState;
  std::unordered_map<SubSocket*, ServiceState> service_state;
  std::unordered_map<SubSocket*, struct RemoteEncoder> remote_encoders;
  std::unordered_map<std::string, RemoteEncoder *> remote_by_name;
  std::vector<std::string> configured_encoder_streams;
  LoggerdAudioQueue audio_queue;

  std::unique_ptr<Context> ctx(Context::create());
  std::unique_ptr<Poller> poller(Poller::create());

  // subscribe to all socks
  for (const auto& [_, it] : services) {
    const bool encoder = util::ends_with(it.name, "EncodeData");
    const bool livestream_encoder = util::starts_with(it.name, "livestream");
    const bool record_audio = (it.name == "rawAudioData") && Params().getBool("RecordAudio");
    if (it.should_log || (encoder && !livestream_encoder) || record_audio) {
      LOGD("logging %s", it.name.c_str());

      SubSocket * sock = SubSocket::create(ctx.get(), it.name, "127.0.0.1", false, true, it.queue_size);
      assert(sock != NULL);
      poller->registerSocket(sock);
      service_state[sock] = {
        .name = it.name,
        .counter = 0,
        .freq = it.decimation,
        .encoder = encoder,
        .preserve_segment = (it.name == "userBookmark") || (it.name == "audioFeedback"),
        .record_audio = record_audio,
      };
    }
  }

  LoggerdState s;
  // init logger
  logger_rotate(&s);
  Params params;
  params.put("CurrentRoute", s.logger.routeName());
  params.put("LoggerdSegment", std::to_string(s.logger.segment()));
  params.put("LoggerdSegmentEpoch", std::to_string(s.rotate_epoch));
  params.put("LoggerdRotateState", "committed");

  std::map<std::string, EncoderInfo> encoder_infos_dict;
  std::vector<RemoteEncoder*> encoders_with_audio;
  for (const auto &cam : cameras_logged) {
    for (const auto &encoder_info : cam.encoder_infos) {
      encoder_infos_dict[encoder_info.publish_name] = encoder_info;
      if (std::find(configured_encoder_streams.begin(), configured_encoder_streams.end(), encoder_info.publish_name) ==
          configured_encoder_streams.end()) {
        configured_encoder_streams.emplace_back(encoder_info.publish_name);
        s.max_waiting++;
      }
    }
  }

  for (auto &[sock, service] : service_state) {
    if (service.encoder) remote_by_name[service.name] = &remote_encoders[sock];
    auto it = encoder_infos_dict.find(service.name);
    if (it != encoder_infos_dict.end() && it->second.include_audio) {
      encoders_with_audio.push_back(&remote_encoders[sock]);
    }
  }
  auto qroad_audio_it = remote_by_name.find("qRoadEncodeData");
  RemoteEncoder *audio_remote = qroad_audio_it == remote_by_name.end() ? nullptr : qroad_audio_it->second;

  const bool deterministic_mode = deterministic_ka2_protocol_active();
  auto service_rotation_clock = [&]() {
    std::chrono::steady_clock::time_point decision_time{};
    if (rotate_if_needed(&s, &decision_time) && deterministic_mode) {
      commit_deterministic_runtime_rotation(&s, configured_encoder_streams, remote_by_name,
                                           encoder_infos_dict, &audio_queue, audio_remote, &params, decision_time);
    }
  };
  auto service_audio_expiry = [&]() {
    if (deterministic_mode) expire_queued_audio(&s, &audio_queue, loggerd_monotonic_now());
  };
  auto poll_timeout_ms = [&]() {
    if (!deterministic_mode) return 1000;
    int timeout_ms = DETERMINISTIC_ROTATION_POLL_MS;
    if (s.rotation_started) {
      const auto deadline = s.rotation_started_at + std::chrono::milliseconds(DETERMINISTIC_ROTATION_DEADLINE_MS);
      const auto now = loggerd_monotonic_now();
      if (now >= deadline) return 0;
      timeout_ms = std::min<int>(timeout_ms, static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count()));
    }
    return std::max(timeout_ms, 0);
  };

  uint64_t msg_count = 0, bytes_count = 0;
  double start_ts = millis_since_boot();
  double last_memory_check_ms = 0;
  int last_memory_percent = -1;
  
  while (!do_exit) {
    service_audio_expiry();
    service_rotation_clock();

    // Check memory pressure periodically (every 5 seconds)
    double now_ms = millis_since_boot();
    if (now_ms - last_memory_check_ms > 5000) {
      int mem_percent = MemoryPressure::get_memory_usage_percent();
      if (mem_percent >= 0 && mem_percent != last_memory_percent) {
        if (MemoryPressure::is_memory_pressure_critical()) {
          LOGW("Memory pressure CRITICAL: %d%% - reducing write operations", mem_percent);
        } else if (MemoryPressure::is_memory_pressure_high()) {
          LOGD("Memory pressure HIGH: %d%% - increasing flush frequency", mem_percent);
        }
        last_memory_percent = mem_percent;
      }
      last_memory_check_ms = now_ms;
    }
    
    // Skip operations if memory is critical
    if (MemoryPressure::should_skip_filesystem_operation()) {
      // Keep deterministic deadline and audio expiry service alive even when
      // filesystem work is temporarily suppressed.
      poller->poll(poll_timeout_ms());
      service_audio_expiry();
      service_rotation_clock();
      continue;
    }
    
    // poll for new messages on all sockets
    const auto ready_sockets = poller->poll(poll_timeout_ms());
    service_audio_expiry();
    service_rotation_clock();
    for (auto sock : ready_sockets) {
      if (do_exit) break;

      ServiceState &service = service_state[sock];
      if (service.preserve_segment) {
        handle_preserve_segment(&s);
      }

      // drain socket
      int count = 0;
      Message *msg = nullptr;
      while (!do_exit && (msg = sock->receive(true))) {
        // Check memory again before processing each message
        if (MemoryPressure::should_skip_filesystem_operation()) {
          delete msg;
          break;  // Skip remaining messages in this batch
        }

        try {
          const bool in_qlog = service.freq != -1 && (service.counter++ % service.freq == 0);

          if (service.record_audio) {
            capnp::FlatArrayMessageReader cmsg(kj::ArrayPtr<capnp::word>((capnp::word *)msg->getData(), msg->getSize() / sizeof(capnp::word)));
            auto event = cmsg.getRoot<cereal::Event>();
            auto audio_data = event.getRawAudioData().getData();
            auto sample_rate = event.getRawAudioData().getSampleRate();
            if (deterministic_mode) {
              admit_audio_event(&s, &audio_queue, audio_remote, s.logger.segment(), event.getLogMonoTime(), sample_rate,
                                reinterpret_cast<const uint8_t *>(audio_data.begin()), audio_data.size());
            } else {
              for (auto* encoder : encoders_with_audio) {
                if (encoder && encoder->writer) {
                  encoder->writer->write_audio((uint8_t*)audio_data.begin(), audio_data.size(), event.getLogMonoTime() / 1000, sample_rate);
                  encoder->audio_initialized = true;
                }
              }
            }
          }

          if (service.encoder) {
            s.last_camera_seen_tms = millis_since_boot();
            bytes_count += handle_encoder_msg(&s, msg, service.name, remote_encoders[sock], encoder_infos_dict[service.name]);
            if (deterministic_mode) {
              auto info_it = encoder_infos_dict.find(service.name);
              if (info_it != encoder_infos_dict.end() && info_it->second.include_audio) {
                flush_queued_audio_for_segment(&s, &audio_queue, &remote_encoders[sock],
                                               s.logger.segment(), loggerd_monotonic_now());
              }
            }
          } else {
            s.logger.write((uint8_t *)msg->getData(), msg->getSize(), in_qlog);
            bytes_count += msg->getSize();
            delete msg;
          }

          service_audio_expiry();
          service_rotation_clock();

          if ((++msg_count % 10000) == 0) {
            double seconds = (millis_since_boot() - start_ts) / 1000.0;
            LOGD("%" PRIu64 " messages, %.2f msg/sec, %.2f KB/sec", msg_count, msg_count / seconds, bytes_count * 0.001 / seconds);
          }

          count++;
          if (count >= 200) {
            LOGD("large volume of '%s' messages", service.name.c_str());
            break;
          }
        } catch (const kj::Exception &e) {
          LOGE("dropping malformed packet on %s (%s)", service.name.c_str(), e.getDescription().cStr());
          delete msg;
          continue;
        }
      }
    }
    service_audio_expiry();
    service_rotation_clock();
  }

  LOGW("closing logger");
  s.logger.setExitSignal(do_exit.signal);

  if (do_exit.power_failure) {
    LOGE("power failure");
    sync();
    LOGE("sync done");
  }

  // messaging cleanup
  for (auto &[sock, service] : service_state) delete sock;
}

int main(int argc, char** argv) {
  if (!Hardware::PC()) {
    int ret;
    ret = util::set_core_affinity({0, 1, 2, 3});
    assert(ret == 0);
    // TODO: why does this impact camerad timings?
    //ret = util::set_realtime_priority(1);
    //assert(ret == 0);
  }

  loggerd_thread();

  return 0;
}
