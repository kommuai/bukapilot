#pragma once

#include <memory>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <vector>
#include <cstdint>
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
  // Three queued buffers plus one worker-owned buffer cover all V4L2 slots.
  static constexpr size_t kQueueDepth = 3;
  int frame_buf_count = 0;
  bool use_external_zerocopy = false;
  bool vipc_buffers_ready = false;
  bool cur_yuv_buf_ready = false;

public:
  VisionIpcServer *vipc_server = nullptr;
  VisionStreamType stream_type;
  int cur_buf_idx = -1;
  FrameMetadata cur_frame_data = {};
  VisionBuf *cur_yuv_buf = nullptr;
  VisionBuf *cur_camera_buf = nullptr;
  std::unique_ptr<VisionBuf[]> camera_bufs;
  std::unique_ptr<FrameMetadata[]> camera_bufs_metadata;
  int rgb_width = 0, rgb_height = 0, nv12_frame_size = 0;

  CameraBuf() = default;
  ~CameraBuf() = default;
  void init(VisionIpcServer *v, int frame_cnt, VisionStreamType type);
  void setupVipcBuffers(bool use_external);
  void sendFrameToVipc();
  bool acquire();
  // Returns the previously queued V4L2 index when the queue overflows.
  int queue(size_t buf_idx);
};

void camerad_thread();
float calculate_exposure_value(const uint8_t *pixels, int stride, Rect ae_xywh, int x_skip, int y_skip);
int open_v4l_by_name_and_index(const char name[], int index = 0, int flags = O_RDWR | O_NONBLOCK);

// RK process thread helpers
typedef void (*process_thread_cb)(MultiCameraState *cameras, CameraState *cs, uint32_t cnt);
void *processing_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback);
std::thread start_process_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback);
void fill_frame_data(cereal::FrameData::Builder &framed, const FrameMetadata &frame_data);
void start_thumbnail_worker(PubMaster *pm);
void stop_thumbnail_worker();
void enqueue_thumbnail(const CameraBuf *buf);

extern ExitHandler do_exit;
