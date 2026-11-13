// RK3588 hardware video encoder (Rockchip MPP + RGA for downscale).
// Analogous to V4LEncoder on TICI and FfmpegEncoder elsewhere; same VideoEncoder interface.

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <inttypes.h>
#include <algorithm>
#include <chrono>
#include <limits>

#define __STDC_CONSTANT_MACROS

#include "system/loggerd/encoder/mpp_encoder.h"
#include "system/loggerd/encoder/mpp_encoder_input.h"

#include <mpp_meta.h>
#include <mpp_packet.h>
#include <rga/rga.h>
#include <rga/im2d.h>

#include "common/swaglog.h"
#include "common/util.h"

#define MPP_ALIGN(x, a) (((x) + ((a) - 1)) & ~((a) - 1))

const int env_debug_encoder = (getenv("DEBUG_ENCODER") != NULL) ? atoi(getenv("DEBUG_ENCODER")) : 0;

#if defined(LOGGERD_TEST_BUILD)
static bool test_fault_selected(const char *publish_name, const char *mode) {
    const char *test_env = getenv("LOGGERD_TEST");
    const char *fault_stream = getenv("LOGGERD_TEST_MPP_FAULT_STREAM");
    const char *fault_mode = getenv("LOGGERD_TEST_MPP_FAULT_MODE");
    return test_env != nullptr && strcmp(test_env, "1") == 0 &&
           fault_stream != nullptr && fault_mode != nullptr &&
           strcmp(fault_stream, publish_name) == 0 && strcmp(fault_mode, mode) == 0;
}
#endif

MppEncoder::MppEncoder(const EncoderInfo &encoder_info, int in_width, int in_height)
    : VideoEncoder(encoder_info, in_width, in_height) {

    if (in_width != out_width || in_height != out_height) {
      is_downscale = true;
    }

    alw = is_downscale ? MPP_ALIGN(out_width, 16) : MPP_ALIGN(in_width, 16);
    alh = is_downscale ? MPP_ALIGN(out_height, 16) : MPP_ALIGN(in_height, 16);
    frame_buf_size = alw * alh * 3 / 2;
}

MppEncoder::~MppEncoder() {
  encoder_close();
}

void MppEncoder::encoder_open() {
  encoder_open(encoder_info.filename);
}

bool MppEncoder::init_frame_buffer_pool(const char *path) {
  if (frame_buf_group != nullptr) {
    return true;
  }

  if (mpp_buffer_group_get_internal(&frame_buf_group, MPP_BUFFER_TYPE_DRM) != MPP_OK) {
    LOGE("mpp_buffer_group_get_internal failed for %s", path);
    if (frame_buf_group != nullptr) {
      mpp_buffer_group_put(frame_buf_group);
      frame_buf_group = nullptr;
    }
    return false;
  }

  for (size_t i = 0; i < frame_buffers.size(); ++i) {
    if (mpp_buffer_get(frame_buf_group, &frame_buffers[i], frame_buf_size) != MPP_OK) {
      LOGE("mpp_buffer_get prealloc failed for %s idx %zu", path, i);
      for (auto &buffer : frame_buffers) {
        if (buffer != nullptr) {
          mpp_buffer_put(buffer);
          buffer = nullptr;
        }
      }
      mpp_buffer_group_put(frame_buf_group);
      frame_buf_group = nullptr;
      return false;
    }
  }

  frame_buffer_idx = 0;
  return true;
}

void MppEncoder::encoder_open(const char* path) {
    encoder_close();

    EncoderSettings settings = encoder_info.get_settings(in_width);
    if (mpp_create(&mpp_ctx, &mpp_mpi) != MPP_OK) {
      LOGE("mpp_create failed for %s", path);
      mpp_ctx = nullptr;
      mpp_mpi = nullptr;
      return;
    }
    LOGD("opened [%d %d %d %d] fps %d %s bitrate %d", in_width, in_height,
        out_width, out_height, encoder_info.fps,
        encoder_info.filename, settings.bitrate);

    const bool is_h264 = settings.encode_type == cereal::EncodeIndex::Type::QCAMERA_H264;
    const bool is_hevc = settings.encode_type == cereal::EncodeIndex::Type::FULL_H_E_V_C;
    if (!is_h264 && !is_hevc) {
      LOGE("unsupported encode type %d for %s", (int)settings.encode_type, path);
      encoder_close();
      return;
    }
    const auto codec_type = is_h264 ? MPP_VIDEO_CodingAVC : MPP_VIDEO_CodingHEVC;
    const char *codec_name = is_h264 ? "AVC" : "HEVC";
    if (mpp_init(mpp_ctx, MPP_CTX_ENC, codec_type) != MPP_OK) {
      LOGE("mpp_init %s failed for %s", codec_name, path);
      encoder_close();
      return;
    }
    if (mpp_enc_cfg_init(&cfg) != MPP_OK) {
      LOGE("mpp_enc_cfg_init %s failed for %s", codec_name, path);
      encoder_close();
      return;
    }
    if (mpp_mpi->control(mpp_ctx, MPP_ENC_GET_CFG, cfg) != MPP_OK) {
      LOGE("MPP_ENC_GET_CFG %s failed for %s", codec_name, path);
      encoder_close();
      return;
    }
    mpp_enc_cfg_set_u32(cfg, "codec:type", codec_type);
    mpp_enc_cfg_set_s32(cfg, "split:mode", MPP_ENC_SPLIT_NONE);

    if (is_h264) {
      //**Profile & Level Settings (Low Quality)**
      mpp_enc_cfg_set_u32(cfg, "h264:profile", 100);
      mpp_enc_cfg_set_u32(cfg, "h264:level", 40);

      // **Entropy Mode (CABAC for better compression)**
      mpp_enc_cfg_set_u32(cfg, "h264:cabac_en", 1);  // Enable CABAC
      mpp_enc_cfg_set_s32(cfg, "h264:cabac_idc", 0);
      mpp_enc_cfg_set_s32(cfg, "h264:trans8x8", 1);
      mpp_enc_cfg_set_s32(cfg, "h264:constraint_set", 0);

      // QP settings
      mpp_enc_cfg_set_s32(cfg, "rc:qp_init", 35);
      mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 45);
      mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 30);
      mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 45);
      mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 30);
      mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 6);
    }
    else {
      // HEVC needs explicit QP bounds for RC to produce output on KA2 MPP.
      mpp_enc_cfg_set_s32(cfg, "rc:qp_init", 26);
      mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 51);
      mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 10);
      mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 46);
      mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 10);
      mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 2);
    }

    mpp_enc_cfg_set_s32(cfg, "prep:width", out_width);
    mpp_enc_cfg_set_s32(cfg, "prep:height", out_height);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", alw);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", alh);
    mpp_enc_cfg_set_s32(cfg, "prep:format", MPP_FMT_YUV420SP);
    mpp_enc_cfg_set_u32(cfg, "rc:fps_in_num", MAIN_FPS);  // input FPS
    mpp_enc_cfg_set_u32(cfg, "rc:fps_out_num", encoder_info.fps); // output FPS
    mpp_enc_cfg_set_u32(cfg, "rc:mode", MPP_ENC_RC_MODE_CBR);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_target", settings.bitrate);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_max", settings.bitrate + 100000);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_min", settings.bitrate - 100000);
    mpp_enc_cfg_set_u32(cfg, "rc:gop", 90); // keyframe interval 2-second GOP for 30 FPS
    if (mpp_mpi->control(mpp_ctx, MPP_ENC_SET_CFG, cfg) != MPP_OK) {
      LOGE("MPP_ENC_SET_CFG failed for %s", path);
      encoder_close();
      return;
    }

    MppEncHeaderMode header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
    if (mpp_mpi->control(mpp_ctx, MPP_ENC_SET_HEADER_MODE, &header_mode) != MPP_OK) {
      LOGW("MPP_ENC_SET_HEADER_MODE failed for %s, headers may only appear on first frame", path);
    }

    {
      MppPacket hdr_pkt = nullptr;
      uint8_t hdr_buf[1024] = {};
      mpp_packet_init(&hdr_pkt, hdr_buf, sizeof(hdr_buf));
      mpp_packet_set_length(hdr_pkt, 0);
      if (mpp_mpi->control(mpp_ctx, MPP_ENC_GET_HDR_SYNC, hdr_pkt) == MPP_OK) {
        size_t hdr_len = mpp_packet_get_length(hdr_pkt);
        if (hdr_len > 0 && hdr_len <= sizeof(hdr_buf)) {
          codec_header = kj::heapArray<capnp::byte>(hdr_buf, hdr_len);
          LOGD("extracted %zu byte codec header for %s", hdr_len, path);
        }
      } else {
        LOGW("MPP_ENC_GET_HDR_SYNC failed for %s", path);
      }
      mpp_packet_deinit(&hdr_pkt);
    }

    RK_S64 output_timeout = MPP_TIMEOUT_NON_BLOCK;
    if (mpp_mpi->control(mpp_ctx, MPP_SET_OUTPUT_TIMEOUT, &output_timeout) == MPP_OK) {
      output_timeout_non_block = true;
    } else {
      output_timeout_non_block = false;
      LOGW("failed to set non-block output timeout for %s", path);
    }
    if (mpp_frame_init(&frame) != MPP_OK) {
      LOGE("mpp_frame_init failed for %s", path);
      encoder_close();
      return;
    }
    mpp_frame_set_width(frame, out_width);
    mpp_frame_set_height(frame, out_height);
    mpp_frame_set_hor_stride(frame, alw);
    mpp_frame_set_ver_stride(frame, alh);
    mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);

    if (is_downscale && !init_frame_buffer_pool(path)) {
      encoder_close();
      return;
    }

    pending_extras.clear();
    seen_first_enqueued = false;
    seen_first_published = false;
    flush_eos_sent = false;
    drop_overflow_count = 0;
    drop_non_monotonic_count = 0;
#if defined(LOGGERD_TEST_BUILD)
    test_drain_timeout_pending = false;
#endif
    is_open = true;
    segment_num++;
    counter = 0;
    LOGD("mpp encoder input: %s", is_downscale ? "RGA resize" : "DMA-BUF import");
}

void MppEncoder::encoder_close() {
    if (!pending_extras.empty()) {
      if (flush_pending_on_close) {
        if (!is_open || mpp_ctx == nullptr || mpp_mpi == nullptr || !flush_pending_packets()) {
          LOGE("encoder %s close timed out with %zu encoded packets pending; frame ids %u..%u will be discarded",
               encoder_info.publish_name, pending_extras.size(),
               pending_extras.front().frame_id, pending_extras.back().frame_id);
          pending_drop_count_ += pending_extras.size();
          pending_extras.clear();
        }
      } else {
        // Recovery/resync path: discard stale packets to avoid cross-segment backlog.
        LOGW("dropping %zu pending encoded packets on forced close", pending_extras.size());
        pending_drop_count_ += pending_extras.size();
        pending_extras.clear();
      }
    }
    for (auto &[fd, buf] : imported_buffers) {
      if (buf != nullptr) {
        mpp_buffer_put(buf);
      }
    }
    imported_buffers.clear();
    for (auto &buf : frame_buffers) {
      if (buf != nullptr) {
        mpp_buffer_put(buf);
        buf = nullptr;
      }
    }
    if (frame_buf_group != nullptr) {
      mpp_buffer_group_put(frame_buf_group);
      frame_buf_group = nullptr;
    }
    if (cfg != nullptr) {
      mpp_enc_cfg_deinit(cfg);
      cfg = nullptr;
    }
    if (frame != nullptr) {
      mpp_frame_deinit(&frame);
      frame = nullptr;
    }
    if (mpp_ctx != nullptr) {
      mpp_destroy(mpp_ctx);
      mpp_ctx = nullptr;
      mpp_mpi = nullptr;
    }
    codec_header = nullptr;
    pending_extras.clear();
    output_timeout_non_block = false;
    seen_first_enqueued = false;
    seen_first_published = false;
    flush_eos_sent = false;
#if defined(LOGGERD_TEST_BUILD)
    test_drain_timeout_pending = false;
#endif
    is_open = false;
}

void MppEncoder::set_segment_num(int segment_num_in) {
    segment_num = segment_num_in;
}

void MppEncoder::set_flush_pending_on_close(bool flush_pending) {
    flush_pending_on_close = flush_pending;
}

bool MppEncoder::flush_pending_packets(int timeout_ms, bool log_timeout) {
    if (pending_extras.empty()) return true;
    if (timeout_ms <= 0 || !is_open || mpp_ctx == nullptr || mpp_mpi == nullptr || frame == nullptr) return false;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    if (!flush_eos_sent) {
      MppFrame eos_frame = nullptr;
      if (mpp_frame_init(&eos_frame) != MPP_OK) return false;
      mpp_frame_set_width(eos_frame, out_width);
      mpp_frame_set_height(eos_frame, out_height);
      mpp_frame_set_hor_stride(eos_frame, alw);
      mpp_frame_set_ver_stride(eos_frame, alh);
      mpp_frame_set_fmt(eos_frame, MPP_FMT_YUV420SP);
      mpp_frame_set_eos(eos_frame, 1);
      const MPP_RET ret = mpp_mpi->encode_put_frame(mpp_ctx, eos_frame);
      mpp_frame_deinit(&eos_frame);
      if (ret != MPP_OK) {
        LOGE("mpp encoder EOS submit failed");
        return false;
      }
      flush_eos_sent = true;
    }

#if defined(LOGGERD_TEST_BUILD)
    if (test_drain_timeout_pending) return false;
#endif

    while (!pending_extras.empty() && std::chrono::steady_clock::now() < deadline) {
      const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
      if (remaining_ms <= 0) break;
      const auto poll_timeout_ms = std::min<std::chrono::milliseconds::rep>(10, remaining_ms);
      if (mpp_mpi->poll(mpp_ctx, MPP_PORT_OUTPUT, static_cast<MppPollType>(poll_timeout_ms)) == MPP_OK) {
        drain_packets(true);
      }
    }
    if (!pending_extras.empty() && log_timeout) {
      LOGE("encoder %s EOS drain timed out after %d ms with %zu frames pending; frame ids %u..%u",
           encoder_info.publish_name, timeout_ms, pending_extras.size(),
           pending_extras.front().frame_id, pending_extras.back().frame_id);
    }
    return pending_extras.empty();
}

#if defined(LOGGERD_TEST_BUILD)
bool MppEncoder::inject_test_drain_timeout(const VisionIpcBufExtra &extra) {
    if (test_drain_timeout_triggered || test_drain_timeout_pending ||
        !test_fault_selected(encoder_info.publish_name, "drain_timeout")) {
      return false;
    }
    pending_extras.push_back(extra);
    input_submitted_count_++;
    test_drain_timeout_pending = true;
    test_drain_timeout_triggered = true;
    return true;
}
#endif

std::vector<uint32_t> MppEncoder::pending_frame_ids() const {
    std::vector<uint32_t> frame_ids;
    frame_ids.reserve(pending_extras.size());
    for (const auto &extra : pending_extras) frame_ids.push_back(extra.frame_id);
    return frame_ids;
}

MppBuffer MppEncoder::acquire_frame_buffer() {
    MppBuffer buffer = frame_buffers[frame_buffer_idx];
    frame_buffer_idx = (frame_buffer_idx + 1) % frame_buffers.size();
    return buffer;
}

MppBuffer MppEncoder::import_frame_buffer(VisionBuf *buf) {
    auto [it, inserted] = imported_buffers.try_emplace(buf->fd, nullptr);
    if (!inserted) return it->second;

    // MPP can accept an ordinary fd but fail to read from it. Reject anything
    // that does not implement the DMA-BUF synchronization API before import.
    if (!mpp_encoder_internal::is_dma_buf_fd(buf->fd)) {
      LOGE("refusing non-DMA-BUF input fd %d for %s", buf->fd, encoder_info.publish_name);
      return nullptr;
    }

    MppBuffer imported = nullptr;
    MppBufferInfo info = {};
    info.type = MPP_BUFFER_TYPE_EXT_DMA;
    info.fd = buf->fd;
    info.size = std::max(frame_buf_size, buf->len);
    MPP_RET import_ret = mpp_buffer_import(&imported, &info);
    if (import_ret != MPP_OK || imported == nullptr) {
      if (imported != nullptr) mpp_buffer_put(imported);
      LOGE("MPP DMA-BUF import failed for %s fd %d: %d",
           encoder_info.publish_name, buf->fd, static_cast<int>(import_ret));
      return nullptr;
    }

    it->second = imported;
    return imported;
}

int MppEncoder::encode_frame(VisionBuf* buf, VisionIpcBufExtra *extra) {
    const auto reject_input = [&]() {
      rejected_count_++;
      return -1;
    };
    if (!is_open || mpp_ctx == nullptr || mpp_mpi == nullptr) {
      return reject_input();
    }
    if (buf->width != this->in_width || buf->height != this->in_height) {
      LOGE("input size mismatch: got %zux%zu expected %dx%d",
           buf->width, buf->height, this->in_width, this->in_height);
      return reject_input();
    }

    if (seen_first_enqueued && extra->frame_id <= last_enqueued_frame_id) {
      drop_non_monotonic_count++;
      if ((drop_non_monotonic_count % 100) == 1) {
        LOGW("dropping non-monotonic frame_id=%u last=%u drops=%" PRIu64,
             extra->frame_id, last_enqueued_frame_id, drop_non_monotonic_count);
      }
      rejected_count_++;
      return -2;
    }
    last_enqueued_frame_id = extra->frame_id;
    seen_first_enqueued = true;

    // Deterministic KA2 policy: never allow in-flight backlog to grow without bound.
    if (pending_extras.size() >= PENDING_EXTRAS_MAX_INFLIGHT) {
      (void)drain_packets(false, DRAIN_BUDGET_PER_FRAME);
    }
    if (pending_extras.size() >= PENDING_EXTRAS_MAX_INFLIGHT) {
      drop_overflow_count++;
      overflow_drop_count_++;
      rejected_count_++;
      if ((drop_overflow_count % 100) == 1) {
        LOGW("dropping frame before submit: in_flight=%zu drops=%" PRIu64,
             pending_extras.size(), drop_overflow_count);
      }
      return -2;
    }

#if defined(LOGGERD_TEST_BUILD)
    // Fault arms are compiled only into an isolated test binary. They model a
    // rejected output, an accepting-but-silent output, or one identity that
    // remains in flight when a boundary sends EOS.
    if (test_fault_selected(encoder_info.publish_name, "reject")) return reject_input();
    if (test_fault_selected(encoder_info.publish_name, "silent")) {
      pending_extras.push_back(*extra);
      input_submitted_count_++;
      return 1;
    }
    if (test_drain_timeout_pending) return 1;
#endif

    MppBuffer input_buffer = nullptr;
    if (is_downscale) {
      input_buffer = acquire_frame_buffer();
      if (input_buffer == nullptr) {
        LOGE("no preallocated mpp frame buffer available");
        return reject_input();
      }
      const auto src = wrapbuffer_virtualaddr(buf->addr, buf->width, buf->height, RK_FORMAT_YCbCr_420_SP);
      const auto dst = wrapbuffer_virtualaddr(mpp_buffer_get_ptr(input_buffer), alw, alh, RK_FORMAT_YCbCr_420_SP);
      if (imresize(src, dst, (double)out_width / buf->width, (double)out_height / buf->height, IM_SYNC) < 0) {
        LOGE("imresize failed");
        return reject_input();
      }
    } else {
      input_buffer = import_frame_buffer(buf);
      if (input_buffer == nullptr) return reject_input();
    }
    mpp_frame_set_buffer(frame, input_buffer);
    if (mpp_mpi->encode_put_frame(mpp_ctx, frame) != MPP_OK) return reject_input();
    pending_extras.push_back(*extra);
    input_submitted_count_++;
    if (drain_packets(output_timeout_non_block, DRAIN_BUDGET_PER_FRAME) < 0) {
      return reject_input();
    }
    if (pending_extras.size() >= PENDING_EXTRAS_MAX_INFLIGHT) {
      (void)drain_packets(false, DRAIN_BUDGET_PER_FRAME);
      // MPP accepted this input. The next input is rejected before submit if
      // the bounded pending queue still cannot drain.
    }
    return 1;
}

int MppEncoder::drain_packets(bool non_block, size_t max_packets) {
    size_t drained = 0;
    while (!pending_extras.empty() && drained < max_packets) {
      if (non_block) {
        MPP_RET poll_ret = mpp_mpi->poll(mpp_ctx, MPP_PORT_OUTPUT, MPP_POLL_NON_BLOCK);
        if (poll_ret != MPP_OK) break;
      }

      if (mpp_mpi->encode_get_packet(mpp_ctx, &packet) != MPP_OK || packet == nullptr) {
        break;
      }

      // Extract the actual encoded payload.
      kj::Array<capnp::byte> owned_payload;
      kj::ArrayPtr<capnp::byte> payload;
      uint8_t *pkt_pos = (uint8_t *)mpp_packet_get_pos(packet);
      size_t pkt_len = mpp_packet_get_length(packet);
      if (pkt_pos != nullptr && pkt_len > 0) {
        payload = kj::arrayPtr<capnp::byte>(pkt_pos, pkt_len);
      } else {
        const MppPktSeg *seg_head = mpp_packet_get_segment_info(packet);
        if (seg_head != nullptr) {
          uint8_t *base = (uint8_t *)mpp_packet_get_data(packet);
          size_t total = 0;
          for (const MppPktSeg *it = seg_head; it != nullptr; it = it->next) {
            total += it->len;
          }
          if (base != nullptr && total > 0) {
            owned_payload = kj::heapArray<capnp::byte>(total);
            size_t off = 0;
            for (const MppPktSeg *it = seg_head; it != nullptr; it = it->next) {
              if (it->len == 0) continue;
              memcpy(owned_payload.begin() + off, base + it->offset, it->len);
              off += it->len;
            }
            payload = owned_payload.asPtr();
          }
        }
      }

      if (payload.size() == 0) {
        const bool eos_packet = mpp_packet_get_eos(packet) != 0;
        mpp_packet_deinit(&packet);
        packet = nullptr;
        drained++;
        if (!eos_packet && !pending_extras.empty()) {
          // MPP packet completion is FIFO for this no-B-frame pipeline. Retire
          // the matching oldest identity once so later output cannot inherit it.
          pending_extras.pop_front();
          empty_packet_count_++;
          pending_drop_count_++;
        }
        continue;
      }

      VisionIpcBufExtra extra = pending_extras.front();
      pending_extras.pop_front();
      if (seen_first_published && extra.frame_id <= last_published_frame_id) {
        drop_non_monotonic_count++;
        if ((drop_non_monotonic_count % 100) == 1) {
          LOGW("dropping non-monotonic publish frame_id=%u last=%u drops=%" PRIu64,
               extra.frame_id, last_published_frame_id, drop_non_monotonic_count);
        }
        pending_drop_count_++;
        mpp_packet_deinit(&packet);
        packet = nullptr;
        continue;
      }
      last_published_frame_id = extra.frame_id;
      seen_first_published = true;

      RK_S32 is_intra = 0;
      if (mpp_packet_has_meta(packet)) {
        MppMeta meta = mpp_packet_get_meta(packet);
        mpp_meta_get_s32_d(meta, KEY_OUTPUT_INTRA, &is_intra, 0);
      }
      unsigned int frame_flags = is_intra ? V4L2_BUF_FLAG_KEYFRAME : 0;

      if (env_debug_encoder) {
        printf("%20s got %8zu bytes idx %4d id %8d%s\n",
               encoder_info.publish_name, payload.size(), counter, extra.frame_id,
               is_intra ? " [I]" : "");
      }

      publisher_publish(segment_num, counter, extra,
        frame_flags,
        codec_header.asPtr(),
        payload);
      packet_published_count_++;
      last_packet_time_ = std::chrono::steady_clock::now();
      last_packet_frame_id_ = extra.frame_id;
      has_published_packet_ = true;
      counter++;

      mpp_packet_deinit(&packet);
      packet = nullptr;
      drained++;
    }
    return static_cast<int>(drained);
}

