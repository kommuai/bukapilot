#pragma once

#include <cstddef>
#include <cstdint>

struct MppEncoderStageSample {
  const char *publish_name = nullptr;
  uint32_t frame_id = 0;
  bool downscale = false;
  int64_t total_us = 0;
  int64_t input_pre_submit_us = 0;
  int64_t rga_us = 0;
  int64_t mpp_submit_us = 0;
  int64_t pre_drain_us = 0;
  int64_t packet_drain_us = 0;
};

void mpp_encoder_probe_clear();
size_t mpp_encoder_probe_size();
MppEncoderStageSample mpp_encoder_probe_at(size_t index);
