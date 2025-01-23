#pragma once

#include <rk_mpi.h>
#include <rk_venc_cfg.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "system/loggerd/encoder/encoder.h"
#include "system/loggerd/loggerd.h"

#include "rga/rga.h"
#include "rga/im2d.h"

class MppEncoder : public VideoEncoder {
public:
  MppEncoder(const EncoderInfo &encoder_info, int in_width, int in_height);
  ~MppEncoder();
  int encode_frame(VisionBuf* buf, VisionIpcBufExtra *extra);
  void encoder_open(const char* path);
  void encoder_close();

private:
  int segment_num = -1;
  int counter = 0;
  bool is_open = false;
  bool is_downscale = false;

  int alw, alh;

  rga_buffer_t src, dst;

  MppCtx mpp_ctx;
  MppApi *mpp_mpi;
  MppFrame frame;
  MppPacket packet;
  MppEncCfg cfg;
  MppBuffer mpp_buf;

  void *downscale_buf;
};
