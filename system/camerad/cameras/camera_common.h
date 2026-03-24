#pragma once

#include <memory>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <sys/mman.h>
#include <fcntl.h>

#include "cereal/messaging/messaging.h"
#include "msgq/visionipc/visionipc_server.h"
#include "common/util.h"


const int VIPC_BUFFER_COUNT = 18;
const int YUV_BUFFER_COUNT = 4;

typedef struct FrameMetadata {
  uint32_t frame_id;
  uint32_t request_id;
  uint64_t timestamp_sof;
  uint64_t timestamp_eof;

  // Exposure
  unsigned int integ_lines;
  bool high_conversion_gain;
  float gain;
  float measured_grey_fraction;
  float target_grey_fraction;

  // Temperature
  float sensor_temp_c;

  float processing_time;
} FrameMetadata;

struct MultiCameraState;
class CameraState;

class CameraBuf {
private:
  mutable std::mutex queue_mtx;
  std::condition_variable queue_cv;
  std::deque<int> frame_idx_queue;
  size_t max_queue_depth = 2;
  size_t max_observed_queue_depth = 0;
  uint64_t dropped_queue_frames = 0;
  int frame_buf_count;
  bool use_external_zerocopy = false;
  bool vipc_buffers_ready = false;

public:
  VisionIpcServer *vipc_server;
  VisionStreamType stream_type;
  int cur_buf_idx;
  FrameMetadata cur_frame_data;
  VisionBuf *cur_yuv_buf;
  VisionBuf *cur_camera_buf;
  std::unique_ptr<VisionBuf[]> camera_bufs;
  std::unique_ptr<FrameMetadata[]> camera_bufs_metadata;
  int rgb_width, rgb_height, nv12_frame_size;
  uint32_t out_img_width, out_img_height;  // for calculate_exposure_value / compatibility

  CameraBuf() = default;
  ~CameraBuf();
  void init(cl_device_id device_id, cl_context context, CameraState *s, VisionIpcServer * v, int frame_cnt, VisionStreamType type);
  void setupVipcBuffers(bool use_external);
  void sendFrameToVipc();
  bool acquire();
  void queue(size_t buf_idx);
  void configure_queue_depth(size_t depth);
};

void camerad_thread();
kj::Array<uint8_t> get_raw_frame_image(const CameraBuf *b);
float calculate_exposure_value(const CameraBuf *b, Rect ae_xywh, int x_skip, int y_skip);
int open_v4l_by_name_and_index(const char name[], int index = 0, int flags = O_RDWR | O_NONBLOCK);

// RK process thread helpers
typedef void (*process_thread_cb)(MultiCameraState *cameras, CameraState *cs, uint32_t cnt);
void *processing_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback);
std::thread start_process_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback);
void fill_frame_data(cereal::FrameData::Builder &framed, const FrameMetadata &frame_data, CameraState *c);
void start_thumbnail_worker(PubMaster *pm);
void stop_thumbnail_worker();
void enqueue_thumbnail(const CameraBuf *buf);

extern ExitHandler do_exit;
