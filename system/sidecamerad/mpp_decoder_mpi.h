#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct SidecamMppDecoder SidecamMppDecoder;

SidecamMppDecoder *sidecam_mpp_decoder_create(uint32_t width, uint32_t height,
                                              size_t max_input_size, char *error,
                                              size_t error_size);
int sidecam_mpp_decoder_decode(SidecamMppDecoder *decoder, const uint8_t *jpeg,
                               size_t jpeg_size, const uint8_t **frame,
                               size_t *frame_size, char *error, size_t error_size);
size_t sidecam_mpp_decoder_buffer_bytes(const SidecamMppDecoder *decoder);
void sidecam_mpp_decoder_destroy(SidecamMppDecoder *decoder);

#ifdef __cplusplus
}
#endif
