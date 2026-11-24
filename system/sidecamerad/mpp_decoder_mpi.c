// SPDX-License-Identifier: MIT

#include "mpp_decoder_mpi.h"

#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_packet.h>
#include <rockchip/rk_mpi.h>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct SidecamMppDecoder {
  uint32_t width;
  uint32_t height;
  uint32_t hor_stride;
  uint32_t ver_stride;
  size_t max_input_size;
  size_t input_capacity;
  size_t frame_size;
  MppCtx ctx;
  MppApi *mpi;
  MppBufferGroup input_group;
  MppBufferGroup frame_group;
  MppBuffer input_buffer;
  MppBuffer output_buffer;
  MppFrame output_frame;
  uint8_t *packed_frame;
  uint8_t y_limited_range[256];
  uint8_t uv_limited_range[256];
};

static int set_error(char *error, size_t error_size, const char *message, int ret) {
  if (error && error_size) {
    if (ret == 0) snprintf(error, error_size, "%s", message);
    else snprintf(error, error_size, "%s failed: %d", message, ret);
  }
  return -1;
}

static void convert_y_row(const SidecamMppDecoder *decoder, uint8_t *destination,
                          const uint8_t *source, uint32_t width) {
  uint32_t column = 0;
#if defined(__aarch64__)
  const uint8x8_t scale = vdup_n_u8(219);
  const uint16x8_t rounding = vdupq_n_u16(127);
  const uint16x8_t one = vdupq_n_u16(1);
  const uint16x8_t offset = vdupq_n_u16(16);
  for (; column + 16 <= width; column += 16) {
    uint8x16_t input = vld1q_u8(source + column);
    uint16x8_t low = vmull_u8(vget_low_u8(input), scale);
    uint16x8_t high = vmull_u8(vget_high_u8(input), scale);
    low = vaddq_u16(low, rounding);
    high = vaddq_u16(high, rounding);
    low = vshrq_n_u16(vaddq_u16(vaddq_u16(low, one), vshrq_n_u16(low, 8)), 8);
    high = vshrq_n_u16(vaddq_u16(vaddq_u16(high, one), vshrq_n_u16(high, 8)), 8);
    low = vaddq_u16(low, offset);
    high = vaddq_u16(high, offset);
    vst1q_u8(destination + column, vcombine_u8(vmovn_u16(low), vmovn_u16(high)));
  }
#endif
  for (; column < width; ++column) {
    destination[column] = decoder->y_limited_range[source[column]];
  }
}

static void convert_uv_row(const SidecamMppDecoder *decoder, uint8_t *destination,
                           const uint8_t *source, uint32_t width) {
  uint32_t column = 0;
#if defined(__aarch64__)
  const int16x8_t center = vdupq_n_s16(128);
  const int16x8_t zero = vdupq_n_s16(0);
  const uint16x8_t rounding = vdupq_n_u16(127);
  const uint16x8_t one = vdupq_n_u16(1);
  const int16x8_t offset = vdupq_n_s16(128);
  for (; column + 16 <= width; column += 16) {
    uint8x16_t input = vld1q_u8(source + column);
    int16x8_t low = vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(input))), center);
    int16x8_t high = vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(input))), center);
    uint16x8_t low_abs = vreinterpretq_u16_s16(vabsq_s16(low));
    uint16x8_t high_abs = vreinterpretq_u16_s16(vabsq_s16(high));
    low_abs = vaddq_u16(vmulq_n_u16(low_abs, 224), rounding);
    high_abs = vaddq_u16(vmulq_n_u16(high_abs, 224), rounding);
    low_abs = vshrq_n_u16(vaddq_u16(vaddq_u16(low_abs, one), vshrq_n_u16(low_abs, 8)), 8);
    high_abs = vshrq_n_u16(vaddq_u16(vaddq_u16(high_abs, one), vshrq_n_u16(high_abs, 8)), 8);
    int16x8_t low_scaled = vreinterpretq_s16_u16(low_abs);
    int16x8_t high_scaled = vreinterpretq_s16_u16(high_abs);
    low_scaled = vbslq_s16(vcltq_s16(low, zero), vnegq_s16(low_scaled), low_scaled);
    high_scaled = vbslq_s16(vcltq_s16(high, zero), vnegq_s16(high_scaled), high_scaled);
    vst1q_u8(destination + column,
              vcombine_u8(vqmovun_s16(vaddq_s16(low_scaled, offset)),
                          vqmovun_s16(vaddq_s16(high_scaled, offset))));
  }
#endif
  for (; column < width; ++column) {
    destination[column] = decoder->uv_limited_range[source[column]];
  }
}

static void destroy_decoder(SidecamMppDecoder *decoder) {
  if (!decoder) return;
  if (decoder->ctx) mpp_destroy(decoder->ctx);
  if (decoder->output_frame) mpp_frame_deinit(&decoder->output_frame);
  if (decoder->input_buffer) mpp_buffer_put(decoder->input_buffer);
  if (decoder->output_buffer) mpp_buffer_put(decoder->output_buffer);
  if (decoder->frame_group) mpp_buffer_group_put(decoder->frame_group);
  if (decoder->input_group) mpp_buffer_group_put(decoder->input_group);
  free(decoder->packed_frame);
  free(decoder);
}

SidecamMppDecoder *sidecam_mpp_decoder_create(uint32_t width, uint32_t height,
                                              size_t max_input_size, char *error,
                                              size_t error_size) {
  SidecamMppDecoder *decoder = calloc(1, sizeof(*decoder));
  if (!decoder) {
    set_error(error, error_size, "allocate MPP decoder", 0);
    return NULL;
  }
  decoder->width = width;
  decoder->height = height;
  decoder->hor_stride = (width + 15) & ~15U;
  decoder->ver_stride = (height + 15) & ~15U;
  decoder->max_input_size = max_input_size;
  decoder->frame_size = (size_t)width * height * 3 / 2;
  for (int value = 0; value < 256; ++value) {
    decoder->y_limited_range[value] = (uint8_t)(((value * 219) + 127) / 255 + 16);
    const int chroma = value - 128;
    const int scaled_chroma = chroma >= 0 ? (chroma * 224 + 127) / 255
                                           : (chroma * 224 - 127) / 255;
    decoder->uv_limited_range[value] = (uint8_t)(scaled_chroma + 128);
  }
  decoder->packed_frame = malloc(decoder->frame_size);
  if (!decoder->packed_frame) {
    set_error(error, error_size, "allocate packed NV12 frame", 0);
    destroy_decoder(decoder);
    return NULL;
  }

  MPP_RET ret = mpp_frame_init(&decoder->output_frame);
  if (ret != MPP_OK) {
    set_error(error, error_size, "initialize caller-provided MPP output frame", ret);
    destroy_decoder(decoder);
    return NULL;
  }
  ret = mpp_buffer_group_get_internal(&decoder->frame_group, MPP_BUFFER_TYPE_DMA_HEAP);
  if (ret != MPP_OK) {
    set_error(error, error_size, "create MPP output frame buffer group", ret);
    destroy_decoder(decoder);
    return NULL;
  }
  const size_t output_capacity = (size_t)decoder->hor_stride * decoder->ver_stride * 4;
  ret = mpp_buffer_get(decoder->frame_group, &decoder->output_buffer, output_capacity);
  if (ret != MPP_OK) {
    set_error(error, error_size, "allocate MPP output frame buffer", ret);
    destroy_decoder(decoder);
    return NULL;
  }
  mpp_frame_set_width(decoder->output_frame, width);
  mpp_frame_set_height(decoder->output_frame, height);
  mpp_frame_set_hor_stride(decoder->output_frame, decoder->hor_stride);
  mpp_frame_set_ver_stride(decoder->output_frame, decoder->ver_stride);
  mpp_frame_set_fmt(decoder->output_frame, MPP_FMT_YUV422SP);
  mpp_frame_set_buffer(decoder->output_frame, decoder->output_buffer);

  ret = mpp_create(&decoder->ctx, &decoder->mpi);
  if (ret != MPP_OK) {
    set_error(error, error_size, "mpp_create", ret);
    destroy_decoder(decoder);
    return NULL;
  }
  ret = mpp_init(decoder->ctx, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);
  if (ret != MPP_OK) {
    set_error(error, error_size, "mpp_init MJPEG", ret);
    destroy_decoder(decoder);
    return NULL;
  }
  MppFrameFormat output_format = MPP_FMT_YUV422SP;
  ret = decoder->mpi->control(decoder->ctx, MPP_DEC_SET_OUTPUT_FORMAT, &output_format);
  if (ret != MPP_OK) {
    set_error(error, error_size, "set MPP NV16 output format", ret);
    destroy_decoder(decoder);
    return NULL;
  }
  ret = mpp_buffer_group_get_internal(&decoder->input_group, MPP_BUFFER_TYPE_DMA_HEAP);
  if (ret != MPP_OK) {
    set_error(error, error_size, "create MPP input buffer group", ret);
    destroy_decoder(decoder);
    return NULL;
  }
  return decoder;
}

static int copy_frame(SidecamMppDecoder *decoder, MppFrame frame,
                      char *error, size_t error_size) {
  const uint32_t width = mpp_frame_get_width(frame);
  const uint32_t height = mpp_frame_get_height(frame);
  const uint32_t stride = mpp_frame_get_hor_stride(frame);
  const uint32_t vstride = mpp_frame_get_ver_stride(frame);
  if (width != decoder->width || height != decoder->height || stride < width || vstride < height ||
      mpp_frame_get_fmt(frame) != MPP_FMT_YUV422SP || (height & 1)) {
    return set_error(error, error_size, "MPP returned an unsupported frame layout", 0);
  }

  MppBuffer buffer = mpp_frame_get_buffer(frame);
  if (!buffer) return set_error(error, error_size, "MPP returned a frame without a buffer", 0);
  const size_t required = (size_t)stride * vstride * 2;
  if (mpp_buffer_get_size(buffer) < required) {
    return set_error(error, error_size, "MPP output buffer is smaller than its reported layout", 0);
  }
  const uint8_t *source = mpp_buffer_get_ptr(buffer);
  if (!source) return set_error(error, error_size, "MPP output buffer is not mapped", 0);

  MPP_RET ret = mpp_buffer_sync_ro_begin(buffer);
  if (ret != MPP_OK) return set_error(error, error_size, "sync MPP output for CPU read", ret);
  for (uint32_t row = 0; row < height; ++row) {
    convert_y_row(decoder, decoder->packed_frame + (size_t)row * width,
                  source + (size_t)row * stride, width);
  }
  const size_t output_uv_offset = (size_t)width * height;
  const size_t source_uv_offset = (size_t)stride * vstride;
  for (uint32_t row = 0; row < height / 2; ++row) {
    // Preserve the previous neighbor-downsample phase when converting MJPEG 4:2:2 to NV12.
    convert_uv_row(decoder, decoder->packed_frame + output_uv_offset + (size_t)row * width,
                   source + source_uv_offset + ((size_t)row * 2 + 1) * stride, width);
  }
  ret = mpp_buffer_sync_ro_end(buffer);
  return ret == MPP_OK ? 0 : set_error(error, error_size, "finish MPP output CPU read", ret);
}

int sidecam_mpp_decoder_decode(SidecamMppDecoder *decoder, const uint8_t *jpeg,
                               size_t jpeg_size, const uint8_t **frame,
                               size_t *frame_size, char *error, size_t error_size) {
  if (!decoder || !jpeg || !jpeg_size || !frame || !frame_size) {
    return set_error(error, error_size, "invalid MPP decode arguments", 0);
  }
  if (jpeg_size > decoder->max_input_size) {
    return set_error(error, error_size, "MJPEG frame is larger than the V4L2 image buffer", 0);
  }

  if (!decoder->input_buffer || jpeg_size > decoder->input_capacity) {
    if (decoder->input_buffer) {
      mpp_buffer_put(decoder->input_buffer);
      decoder->input_buffer = NULL;
    }
    if (decoder->input_capacity) {
      mpp_buffer_group_put(decoder->input_group);
      decoder->input_group = NULL;
      MPP_RET ret = mpp_buffer_group_get_internal(&decoder->input_group, MPP_BUFFER_TYPE_DMA_HEAP);
      if (ret != MPP_OK) return set_error(error, error_size, "recreate MPP input buffer group", ret);
    }
    size_t capacity = jpeg_size;
    if (decoder->input_capacity <= decoder->max_input_size / 2 && decoder->input_capacity * 2 > capacity) {
      capacity = decoder->input_capacity * 2;
    }
    MPP_RET ret = mpp_buffer_get(decoder->input_group, &decoder->input_buffer, capacity);
    if (ret != MPP_OK) return set_error(error, error_size, "allocate MPP input buffer", ret);
    decoder->input_capacity = capacity;
    if (mpp_buffer_get_size(decoder->input_buffer) < capacity) {
      return set_error(error, error_size, "MPP allocated an undersized input buffer", 0);
    }
  }

  MPP_RET ret = mpp_buffer_write(decoder->input_buffer, 0, (void *)jpeg, jpeg_size);
  if (ret != MPP_OK) return set_error(error, error_size, "copy MJPEG frame to MPP input", ret);
  void *input = mpp_buffer_get_ptr(decoder->input_buffer);
  if (!input) return set_error(error, error_size, "MPP input buffer is not mapped", 0);
  MppPacket packet = NULL;
  ret = mpp_packet_init_with_buffer(&packet, decoder->input_buffer);
  if (ret != MPP_OK) return set_error(error, error_size, "create MPP packet", ret);
  mpp_packet_set_size(packet, jpeg_size);
  mpp_packet_set_pos(packet, input);
  mpp_packet_set_length(packet, jpeg_size);

  // This installed MPP release needs the task interface to pair a caller-
  // provided output frame with each MJPEG packet.
  MppTask input_task = NULL;
  MppTask output_task = NULL;
  ret = decoder->mpi->poll(decoder->ctx, MPP_PORT_INPUT, MPP_POLL_BLOCK);
  if (ret != MPP_OK) {
    mpp_packet_deinit(&packet);
    return set_error(error, error_size, "poll MPP MJPEG input port", ret);
  }
  ret = decoder->mpi->dequeue(decoder->ctx, MPP_PORT_INPUT, &input_task);
  if (ret != MPP_OK || !input_task) {
    mpp_packet_deinit(&packet);
    return set_error(error, error_size, "dequeue MPP MJPEG input task", ret);
  }
  ret = mpp_task_meta_set_packet(input_task, KEY_INPUT_PACKET, packet);
  if (ret == MPP_OK) ret = mpp_task_meta_set_frame(input_task, KEY_OUTPUT_FRAME, decoder->output_frame);
  if (ret != MPP_OK) {
    decoder->mpi->enqueue(decoder->ctx, MPP_PORT_INPUT, input_task);
    mpp_packet_deinit(&packet);
    return set_error(error, error_size, "attach packet and frame to MPP task", ret);
  }
  ret = decoder->mpi->enqueue(decoder->ctx, MPP_PORT_INPUT, input_task);
  if (ret != MPP_OK) {
    mpp_packet_deinit(&packet);
    return set_error(error, error_size, "submit MPP MJPEG input task", ret);
  }

  ret = decoder->mpi->poll(decoder->ctx, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);
  if (ret != MPP_OK) {
    mpp_packet_deinit(&packet);
    return set_error(error, error_size, "poll MPP MJPEG output port", ret);
  }
  ret = decoder->mpi->dequeue(decoder->ctx, MPP_PORT_OUTPUT, &output_task);
  if (ret != MPP_OK || !output_task) {
    mpp_packet_deinit(&packet);
    return set_error(error, error_size, "dequeue MPP MJPEG output task", ret);
  }

  MppFrame decoded = NULL;
  ret = mpp_task_meta_get_frame(output_task, KEY_OUTPUT_FRAME, &decoded);
  int copy_ret = 0;
  if (ret != MPP_OK || !decoded) {
    copy_ret = set_error(error, error_size, "get MPP MJPEG output frame", ret);
  } else if (decoded != decoder->output_frame) {
    copy_ret = set_error(error, error_size, "MPP did not use the caller-provided output frame", 0);
  } else if (mpp_frame_get_info_change(decoded)) {
    ret = decoder->mpi->control(decoder->ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
    copy_ret = set_error(error, error_size, "MPP output layout changed", ret);
  } else if (mpp_frame_get_errinfo(decoded) || mpp_frame_get_discard(decoded)) {
    copy_ret = set_error(error, error_size, "MPP returned a corrupt or discarded frame", 0);
  } else {
    copy_ret = copy_frame(decoder, decoded, error, error_size);
  }
  MPP_RET enqueue_ret = decoder->mpi->enqueue(decoder->ctx, MPP_PORT_OUTPUT, output_task);
  mpp_packet_deinit(&packet);
  if (copy_ret != 0) return copy_ret;
  if (enqueue_ret != MPP_OK) return set_error(error, error_size, "release MPP MJPEG output task", enqueue_ret);
  *frame = decoder->packed_frame;
  *frame_size = decoder->frame_size;
  return 0;
}

size_t sidecam_mpp_decoder_buffer_bytes(const SidecamMppDecoder *decoder) {
  if (!decoder) return 0;
  return mpp_buffer_group_usage(decoder->frame_group) + mpp_buffer_group_usage(decoder->input_group);
}

void sidecam_mpp_decoder_destroy(SidecamMppDecoder *decoder) {
  destroy_decoder(decoder);
}
