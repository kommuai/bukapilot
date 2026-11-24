#pragma once

#include <rk_mpi.h>
#include <rk_venc_cfg.h>
#include <rk_venc_cmd.h>
#include <chrono>
#include <cstdint>
#include <array>
#include <deque>
#include <limits>
#include <unordered_map>
#include <vector>

#include "system/loggerd/encoder/encoder.h"
#include "system/loggerd/loggerd.h"

class MppEncoder : public VideoEncoder {
public:
  MppEncoder(const EncoderInfo &encoder_info, int in_width, int in_height);
  ~MppEncoder() override;
  int encode_frame(VisionBuf* buf, VisionIpcBufExtra *extra) override;
  void encoder_open() override;
  void encoder_close() override;
  void set_segment_num(int segment_num) override;
  void set_flush_pending_on_close(bool flush_pending);
  bool flush_pending_packets(int timeout_ms = 500, bool log_timeout = true);

  uint64_t input_submitted_count() const { return input_submitted_count_; }
  uint64_t packet_published_count() const { return packet_published_count_; }
  std::chrono::steady_clock::time_point last_packet_time() const { return last_packet_time_; }
  uint32_t last_packet_frame_id() const { return last_packet_frame_id_; }
  bool has_published_packet() const { return has_published_packet_; }
  std::vector<uint32_t> pending_frame_ids() const;
  size_t pending_frame_count() const { return pending_extras.size(); }
  uint64_t rejected_count() const { return rejected_count_; }
  uint64_t empty_packet_count() const { return empty_packet_count_; }
  uint64_t overflow_drop_count() const { return overflow_drop_count_; }
  uint64_t pending_drop_count() const { return pending_drop_count_; }
  uint64_t failure_retry_count() const { return failure_retry_count_; }
  uint64_t failure_recovery_reopen_count() const { return failure_recovery_reopen_count_; }
  uint64_t segment_transition_reopen_count() const { return segment_transition_reopen_count_; }
  bool encoder_is_open() const { return is_open; }

  // Internal encoderd accounting. These counters are per encoder instance and
  // are not exported as Params, cereal fields, or public metrics.
  void note_failure_retry_started() { failure_retry_count_++; }
  void note_failure_recovery_reopen() { failure_recovery_reopen_count_++; }
  void note_segment_transition_reopen() { segment_transition_reopen_count_++; }

#if defined(LOGGERD_TEST_BUILD)
  // These controls exist only in the isolated test build. The production
  // logger/encoderd binaries do not read the fault-injection environment.
  bool inject_test_drain_timeout(const VisionIpcBufExtra &extra);
#endif
private:
  void encoder_open(const char* path);
  bool init_frame_buffer_pool(const char* path);
  MppBuffer import_frame_buffer(VisionBuf *buf);
  MppBuffer acquire_frame_buffer();
  int drain_packets(bool non_block, size_t max_packets = std::numeric_limits<size_t>::max());
  int segment_num = -1;
  int counter = 0;
  bool is_open = false;
  bool is_downscale = false;

  int alw, alh;

  MppCtx mpp_ctx = nullptr;
  MppApi *mpp_mpi = nullptr;
  MppFrame frame = nullptr;
  MppPacket packet = nullptr;
  MppEncCfg cfg = nullptr;
  MppBufferGroup frame_buf_group = nullptr;
  static constexpr size_t FRAME_BUFFER_POOL_SIZE = 6;
  std::array<MppBuffer, FRAME_BUFFER_POOL_SIZE> frame_buffers = {};
  size_t frame_buffer_idx = 0;
  std::unordered_map<int, MppBuffer> imported_buffers;
  std::deque<VisionIpcBufExtra> pending_extras;
  kj::Array<capnp::byte> codec_header;
  bool output_timeout_non_block = false;
  bool flush_pending_on_close = true;
  bool flush_eos_sent = false;
  static constexpr size_t PENDING_EXTRAS_MAX_INFLIGHT = 8;
  static constexpr size_t DRAIN_BUDGET_PER_FRAME = 2;
  uint32_t last_enqueued_frame_id = 0;
  uint32_t last_published_frame_id = 0;
  bool seen_first_enqueued = false;
  bool seen_first_published = false;
  uint64_t drop_overflow_count = 0;
  uint64_t drop_non_monotonic_count = 0;
  uint64_t input_submitted_count_ = 0;
  uint64_t packet_published_count_ = 0;
  std::chrono::steady_clock::time_point last_packet_time_{};
  uint32_t last_packet_frame_id_ = 0;
  bool has_published_packet_ = false;
  uint64_t rejected_count_ = 0;
  uint64_t empty_packet_count_ = 0;
  uint64_t overflow_drop_count_ = 0;
  uint64_t pending_drop_count_ = 0;
  uint64_t failure_retry_count_ = 0;
  uint64_t failure_recovery_reopen_count_ = 0;
  uint64_t segment_transition_reopen_count_ = 0;

#if defined(LOGGERD_TEST_BUILD)
  bool test_drain_timeout_pending = false;
  bool test_drain_timeout_triggered = false;
#endif

  size_t frame_buf_size = 0;
};
