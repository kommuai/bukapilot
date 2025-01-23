#include <cassert>
#include <cstdio>
#include <cstdlib>

#define __STDC_CONSTANT_MACROS

#include "system/loggerd/encoder/mpp_encoder.h"
#include "third_party/libyuv/include/libyuv.h"

#include "common/swaglog.h"
#include "common/util.h"

#define MPP_ALIGN(x, a) (((x) + ((a) - 1)) & ~((a) - 1))

MppEncoder::MppEncoder(const EncoderInfo &encoder_info, int in_width, int in_height)
    : VideoEncoder(encoder_info, in_width, in_height) {

    if (in_width != out_width || in_height != out_height) {
      is_downscale = true;
    }

    alw = is_downscale ? MPP_ALIGN(out_width, 16) : MPP_ALIGN(in_width, 16);
    alh = is_downscale ? MPP_ALIGN(out_height, 16) : in_height;

    if (is_downscale) {
      downscale_buf = malloc(alw * alh * 3 / 2);
    }
}

MppEncoder::~MppEncoder() {
  encoder_close();

  if (is_downscale) {
    free(downscale_buf);
    downscale_buf = NULL;
  }
}

void MppEncoder::encoder_open(const char* path) {
    assert(mpp_create(&mpp_ctx, &mpp_mpi) == MPP_OK);
    LOGD("opened [%d %d %d %d] fps %d %s bitrate %d", in_width, in_height,
        out_width, out_height, encoder_info.fps,
        encoder_info.filename, encoder_info.bitrate);

    if (encoder_info.encode_type == cereal::EncodeIndex::Type::QCAMERA_H264) {
      assert(mpp_init(mpp_ctx, MPP_CTX_ENC, MPP_VIDEO_CodingAVC) == MPP_OK);
      mpp_enc_cfg_init(&cfg);
      mpp_mpi->control(mpp_ctx, MPP_ENC_GET_CFG, cfg);
      mpp_enc_cfg_set_u32(cfg, "codec:type", MPP_VIDEO_CodingAVC);
      mpp_enc_cfg_set_s32(cfg, "split:mode", MPP_ENC_SPLIT_NONE);

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
    else if (encoder_info.encode_type == cereal::EncodeIndex::Type::FULL_H_E_V_C) {
      assert(mpp_init(mpp_ctx, MPP_CTX_ENC, MPP_VIDEO_CodingHEVC) == MPP_OK);
      mpp_enc_cfg_init(&cfg);
      mpp_mpi->control(mpp_ctx, MPP_ENC_GET_CFG, cfg);
      mpp_enc_cfg_set_u32(cfg, "codec:type", MPP_VIDEO_CodingHEVC);
    }
    else { return; }

    mpp_enc_cfg_set_s32(cfg, "prep:width", out_width);
    mpp_enc_cfg_set_s32(cfg, "prep:height", out_height);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", alw);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", alh);
    mpp_enc_cfg_set_s32(cfg, "prep:format", MPP_FMT_YUV420SP);
    mpp_enc_cfg_set_u32(cfg, "rc:fps_in_num", MAIN_FPS);  // input FPS
    mpp_enc_cfg_set_u32(cfg, "rc:fps_out_num", encoder_info.fps); // output FPS
    mpp_enc_cfg_set_u32(cfg, "rc:mode", MPP_ENC_RC_MODE_CBR);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_target", encoder_info.bitrate);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_max", encoder_info.bitrate + 100000);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_min", encoder_info.bitrate - 100000);
    mpp_enc_cfg_set_u32(cfg, "rc:gop", 90); // keyframe interval 2-second GOP for 30 FPS
    mpp_mpi->control(mpp_ctx, MPP_ENC_SET_CFG, cfg);

    is_open = true;
    segment_num++;
    counter = 0;
}

void MppEncoder::encoder_close() {
    if (!is_open) return;
    mpp_destroy(mpp_ctx);
    is_open = false;
}

int MppEncoder::encode_frame(VisionBuf* buf, VisionIpcBufExtra *extra) {
    assert(buf->width == this->in_width);
    assert(buf->height == this->in_height);

    // Allocate & populate frame buffer
    assert(mpp_buffer_get(NULL, &mpp_buf, alw * alh * 3 / 2) == MPP_OK);
    mpp_frame_init(&frame);
    mpp_frame_set_width(frame, buf->width);
    mpp_frame_set_height(frame, buf->height);
    mpp_frame_set_hor_stride(frame, alw);
    mpp_frame_set_ver_stride(frame, alh);
    mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);

    if (is_downscale) {
      src = wrapbuffer_virtualaddr(buf->addr, buf->width, buf->height, RK_FORMAT_YCbCr_420_SP);
      dst = wrapbuffer_virtualaddr(downscale_buf, alw, alh, RK_FORMAT_YCbCr_420_SP);
      assert(imresize(src, dst, (double)out_width/buf->width, (double)out_height/buf->height, IM_SYNC) >= 0);
      memcpy(mpp_buffer_get_ptr(mpp_buf), downscale_buf, alw * alh * 3 / 2);
    }
    else {
      memcpy(mpp_buffer_get_ptr(mpp_buf), buf->addr, alw * alh * 3 / 2);
    }

    mpp_frame_set_buffer(frame, mpp_buf);
    assert(mpp_mpi->encode_put_frame(mpp_ctx, frame) == MPP_OK);
    mpp_frame_deinit(&frame);
    assert(mpp_mpi->encode_get_packet(mpp_ctx, &packet) == MPP_OK);

    uint8_t *pkt = (uint8_t*)mpp_packet_get_pos(packet);
    size_t pkt_size = mpp_packet_get_length(packet);

    publisher_publish(this, segment_num, counter, *extra,
      V4L2_BUF_FLAG_KEYFRAME,
      kj::arrayPtr<capnp::byte>(pkt, (size_t)0), // TODO: get header
      kj::arrayPtr<capnp::byte>(pkt, pkt_size));

    counter++;
    mpp_packet_deinit(&packet);
    mpp_buffer_put(mpp_buf);
    return 1;
}

