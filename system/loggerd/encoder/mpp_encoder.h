#pragma once

#include <rk_mpi.h>
#include <rk_venc_cfg.h>
#include <rk_venc_cmd.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <array>
#include <deque>
#include <limits>
#include <unordered_map>

#include "system/loggerd/encoder/encoder.h"
#include "system/loggerd/loggerd.h"

#include "rga/rga.h"
#include "rga/im2d.h"

class MppEncoder : public VideoEncoder {
public:
  MppEncoder(const EncoderInfo &encoder_info, int in_width, int in_height);
  ~MppEncoder();
  int encode_frame(VisionBuf* buf, VisionIpcBufExtra *extra);
  void encoder_open() override;
  void encoder_close();
  void set_segment_num(int segment_num) override;
  void set_flush_pending_on_close(bool flush_pending);
private:
  void encoder_open(const char* path);
  MppBuffer acquire_frame_buffer();
  int drain_packets(bool non_block, size_t max_packets = std::numeric_limits<size_t>::max());

  int segment_num = -1;
  int counter = 0;
  bool is_open = false;
  bool is_downscale = false;
  bool use_zero_copy = false;

  int alw, alh;

  rga_buffer_t src, dst;

  MppCtx mpp_ctx = nullptr;
  MppApi *mpp_mpi = nullptr;
  MppFrame frame = nullptr;
  MppPacket packet = nullptr;
  MppEncCfg cfg = nullptr;
  MppBufferGroup frame_buf_group = nullptr;
  MppBuffer mpp_buf = nullptr;
  static constexpr size_t FRAME_BUFFER_POOL_SIZE = 6;
  std::array<MppBuffer, FRAME_BUFFER_POOL_SIZE> frame_buffers = {};
  size_t frame_buffer_idx = 0;
  std::unordered_map<int, MppBuffer> imported_buffers;
  std::deque<VisionIpcBufExtra> pending_extras;
  kj::Array<capnp::byte> codec_header;
  bool import_fallback_logged = false;
  uint32_t zero_copy_import_failures = 0;
  bool output_timeout_non_block = false;
  bool flush_pending_on_close = true;
  static constexpr size_t PENDING_EXTRAS_MAX_INFLIGHT = 8;
  static constexpr size_t DRAIN_BUDGET_PER_FRAME = 2;
  uint32_t last_enqueued_frame_id = 0;
  uint32_t last_published_frame_id = 0;
  bool seen_first_enqueued = false;
  bool seen_first_published = false;
  uint64_t drop_overflow_count = 0;
  uint64_t drop_non_monotonic_count = 0;

  size_t frame_buf_size = 0;
};
