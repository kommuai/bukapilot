#include "system/camerad/cameras/camera_common.h"

#include <cassert>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <queue>

#include <jpeglib.h>
#include "third_party/libyuv/include/libyuv.h"
#include "rga/rga.h"
#include "rga/im2d.h"

#include "cereal/messaging/messaging.h"
#include "common/clutil.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "system/camerad/cameras/camera_rk.h"
#ifdef QCOM2
#include "CL/cl_ext_qcom.h"
#endif

ExitHandler do_exit;

static const int env_queue_depth = getenv("CAMERAD_QUEUE_DEPTH") ? std::max(1, atoi(getenv("CAMERAD_QUEUE_DEPTH"))) : 2;

struct ThumbnailJob {
  uint32_t frame_id = 0;
  uint64_t timestamp_eof = 0;
  int width = 0;
  int height = 0;
  int stride = 0;
  std::vector<uint8_t> nv12;
};

static void publish_thumbnail(PubMaster *pm, const ThumbnailJob &job);

static bool resize_nv12_with_rga(const uint8_t *src_nv12, int src_w, int src_h, int src_stride,
                                 uint8_t *dst_nv12, int dst_w, int dst_h, int dst_stride) {
  (void)src_stride;
  (void)dst_stride;
  if (!src_nv12 || !dst_nv12) return false;
  rga_buffer_t src = wrapbuffer_virtualaddr(const_cast<uint8_t *>(src_nv12), src_w, src_h, RK_FORMAT_YCbCr_420_SP);
  rga_buffer_t dst = wrapbuffer_virtualaddr(dst_nv12, dst_w, dst_h, RK_FORMAT_YCbCr_420_SP);
  int ret = imresize(src, dst, (double)dst_w / src_w, (double)dst_h / src_h, IM_SYNC);
  return ret >= 0;
}

void CameraBuf::init(cl_device_id device_id, cl_context context, CameraState *s, VisionIpcServer * v, int frame_cnt, VisionStreamType type) {
  (void)device_id;
  (void)context;
  (void)s;
  vipc_server = v;
  stream_type = type;
  frame_buf_count = frame_cnt;

  rgb_width = 1920;
  rgb_height = 1200;
  out_img_width = (uint32_t)rgb_width;
  out_img_height = (uint32_t)rgb_height;

  // NV12 frame
  nv12_frame_size = (rgb_width * rgb_height * 3)/2;
  camera_bufs = std::make_unique<VisionBuf[]>(frame_buf_count);
  camera_bufs_metadata = std::make_unique<FrameMetadata[]>(frame_buf_count);
  configure_queue_depth((size_t)env_queue_depth);

  int nv12_width = rgb_width;
  int nv12_height = rgb_height;
  size_t nv12_size = nv12_frame_size;
  size_t nv12_uv_offset = nv12_width * nv12_height;
  (void)nv12_size;
  (void)nv12_uv_offset;
  vipc_buffers_ready = false;
  use_external_zerocopy = false;
}

void CameraBuf::setupVipcBuffers(bool use_external) {
  if (vipc_buffers_ready) return;
  use_external_zerocopy = use_external;

  int nv12_width = rgb_width;
  int nv12_height = rgb_height;
  size_t nv12_size = nv12_frame_size;
  size_t nv12_uv_offset = nv12_width * nv12_height;

  if (use_external_zerocopy) {
    std::vector<VisionBuf *> ext_buffers;
    ext_buffers.reserve(frame_buf_count);
    for (int i = 0; i < frame_buf_count; ++i) {
      camera_bufs[i].type = stream_type;
      camera_bufs[i].idx = i;
      ext_buffers.push_back(&camera_bufs[i]);
    }
    vipc_server->register_external_buffers(stream_type, ext_buffers);
    LOGD("registered %d external v4l2 dmabuf buffers for stream %d", frame_buf_count, stream_type);
  } else {
    vipc_server->create_buffers_with_sizes(stream_type, YUV_BUFFER_COUNT, rgb_width, rgb_height, nv12_size, nv12_width, nv12_uv_offset);
    LOGD("created %d YUV vipc buffers with size %dx%d", YUV_BUFFER_COUNT, nv12_width, nv12_height);
  }
  vipc_buffers_ready = true;
}

CameraBuf::~CameraBuf() {
  // RK path: buffers are mmap'd and freed by camera_close
}

bool CameraBuf::acquire() {
  {
    std::unique_lock lk(queue_mtx);
    if (repeat_output_enabled) {
      if (!queue_cv.wait_for(lk, std::chrono::milliseconds(100), [this] {
            return !repeat_output_enabled || repeat_snapshot_valid;
          })) {
        return false;
      }
      if (repeat_output_enabled) {
        const auto now = std::chrono::steady_clock::now();
        if (!repeat_output_started) {
          repeat_output_started = true;
          repeat_next_publish = now;
          repeat_next_frame_id = last_output_frame_id_valid
              ? last_output_frame_id + 1 : repeat_snapshot_metadata.frame_id;
        }
        while (repeat_output_enabled && std::chrono::steady_clock::now() < repeat_next_publish) {
          queue_cv.wait_until(lk, repeat_next_publish);
        }
        if (!repeat_output_enabled) {
          // The sensor returned to nominal cadence while the processing
          // thread was waiting for a repeat tick; consume real buffers below.
        } else {
          // The mapped V4L2 buffer was requeued immediately after dequeue.
          // Keep the snapshot lock through this copy so the producer cannot
          // overwrite it before VisionIPC owns the destination buffer.
          cur_frame_data = repeat_snapshot_metadata;
          cur_frame_data.frame_id = repeat_next_frame_id++;
          cur_frame_data.request_id = cur_frame_data.frame_id;
          last_output_frame_id = cur_frame_data.frame_id;
          last_output_frame_id_valid = true;
          const uint64_t publish_ns = nanos_since_boot();
          cur_frame_data.timestamp_sof = publish_ns;
          cur_frame_data.timestamp_eof = publish_ns;
          cur_camera_buf = nullptr;
          cur_yuv_buf = vipc_server->get_buffer(stream_type);
          memcpy(cur_yuv_buf->addr, repeat_snapshot_nv12.data(), nv12_frame_size);
          cur_yuv_buf_ready = true;

          constexpr auto kOutputPeriod = std::chrono::milliseconds(50);
          repeat_next_publish += kOutputPeriod;
          if (repeat_next_publish <= std::chrono::steady_clock::now()) {
            repeat_next_publish = std::chrono::steady_clock::now() + kOutputPeriod;
          }
          lk.unlock();
          sendFrameToVipc();
          return true;
        }
      }
    }
  }

  int idx;
  {
    std::unique_lock lk(queue_mtx);
    if (!queue_cv.wait_for(lk, std::chrono::milliseconds(100), [this] { return !frame_idx_queue.empty(); })) {
      return false;
    }
    idx = frame_idx_queue.front();
    frame_idx_queue.pop_front();
  }
  cur_buf_idx = idx;
  cur_frame_data = camera_bufs_metadata[idx];
  if (last_output_frame_id_valid && cur_frame_data.frame_id <= last_output_frame_id) {
    cur_frame_data.frame_id = last_output_frame_id + 1;
    cur_frame_data.request_id = cur_frame_data.frame_id;
  }
  last_output_frame_id = cur_frame_data.frame_id;
  last_output_frame_id_valid = true;
  cur_yuv_buf_ready = false;
  sendFrameToVipc();
  return true;
}

void CameraBuf::sendFrameToVipc() {
  if (!cur_yuv_buf_ready) {
    assert(cur_buf_idx >=0 && cur_buf_idx < frame_buf_count);
    cur_camera_buf = &camera_bufs[cur_buf_idx];
    if (use_external_zerocopy) {
      cur_yuv_buf = cur_camera_buf;
    } else {
      cur_yuv_buf = vipc_server->get_buffer(stream_type);
      memcpy(cur_yuv_buf->addr, cur_camera_buf->addr, nv12_frame_size);
    }
  }

  VisionIpcBufExtra extra = {
    .frame_id = cur_frame_data.frame_id,
    .timestamp_sof = cur_frame_data.timestamp_sof,
    .timestamp_eof = cur_frame_data.timestamp_eof,
    .valid = !use_external_zerocopy,  // valid indicates whether frame_id is readable from shared buffer payload.
  };

  cur_yuv_buf->set_frame_id(cur_frame_data.frame_id);
  vipc_server->send(cur_yuv_buf, &extra, false);
  cur_yuv_buf_ready = false;
}

void CameraBuf::queue(size_t buf_idx) {
  {
    std::lock_guard lk(queue_mtx);
    if (repeat_output_enabled) {
      // Snapshot before CameraState requeues this buffer to the V4L2 driver.
      memcpy(repeat_snapshot_nv12.data(), camera_bufs[buf_idx].addr, nv12_frame_size);
      repeat_snapshot_metadata = camera_bufs_metadata[buf_idx];
      repeat_snapshot_valid = true;
      queue_cv.notify_one();
      return;
    }
    if (frame_idx_queue.size() >= max_queue_depth) {
      frame_idx_queue.pop_front();
      dropped_queue_frames++;
    }
    frame_idx_queue.push_back((int)buf_idx);
    max_observed_queue_depth = std::max(max_observed_queue_depth, frame_idx_queue.size());
  }
  queue_cv.notify_one();
}

void CameraBuf::set_repeat_output(bool enabled) {
  std::lock_guard lk(queue_mtx);
  if (repeat_output_enabled == enabled) return;

  repeat_output_enabled = enabled;
  repeat_output_started = false;
  repeat_snapshot_valid = false;
  frame_idx_queue.clear();
  if (enabled && repeat_snapshot_nv12.size() != static_cast<size_t>(nv12_frame_size)) {
    repeat_snapshot_nv12.resize(nv12_frame_size);
  }
  queue_cv.notify_all();
}

void CameraBuf::configure_queue_depth(size_t depth) {
  std::lock_guard lk(queue_mtx);
  max_queue_depth = std::max<size_t>(1, depth);
}

// common functions

void fill_frame_data(cereal::FrameData::Builder &framed, const FrameMetadata &frame_data, CameraState *c) {
  framed.setFrameId(frame_data.frame_id);
  framed.setRequestId(frame_data.request_id);
  framed.setTimestampEof(frame_data.timestamp_eof);
  framed.setTimestampSof(frame_data.timestamp_sof);
  framed.setIntegLines(frame_data.integ_lines);
  framed.setGain(frame_data.gain);
  framed.setHighConversionGain(frame_data.high_conversion_gain);
  framed.setMeasuredGreyFraction(frame_data.measured_grey_fraction);
  framed.setTargetGreyFraction(frame_data.target_grey_fraction);
  framed.setProcessingTime(frame_data.processing_time);
  framed.setSensor(cereal::FrameData::ImageSensor::OX03C10);

  std::vector<float> temps = {frame_data.sensor_temp_c};
  kj::ArrayPtr<const float> temp_array(temps.data(), temps.size());
  framed.setTemperaturesC(temp_array);
}

kj::Array<uint8_t> get_raw_frame_image(const CameraBuf *b) {
  const uint8_t *dat = (const uint8_t *)b->cur_camera_buf->addr;

  kj::Array<uint8_t> frame_image = kj::heapArray<uint8_t>(b->cur_camera_buf->len);
  uint8_t *resized_dat = frame_image.begin();

  memcpy(resized_dat, dat, b->cur_camera_buf->len);

  return kj::mv(frame_image);
}

float calculate_exposure_value(const CameraBuf *b, Rect ae_xywh, int x_skip, int y_skip) {
  int lum_med;
  uint32_t lum_binning[256] = {0};
  const uint8_t *pix_ptr = b->cur_yuv_buf->y;

  unsigned int lum_total = 0;
  for (int y = ae_xywh.y; y < ae_xywh.y + ae_xywh.h; y += y_skip) {
    for (int x = ae_xywh.x; x < ae_xywh.x + ae_xywh.w; x += x_skip) {
      uint8_t lum = pix_ptr[(y * b->out_img_width) + x];
      lum_binning[lum]++;
      lum_total += 1;
    }
  }

  // Find mean lumimance value
  unsigned int lum_cur = 0;
  for (lum_med = 255; lum_med >= 0; lum_med--) {
    lum_cur += lum_binning[lum_med];

    if (lum_cur >= lum_total / 2) {
      break;
    }
  }

  return lum_med / 256.0;
}

class ThumbnailWorker {
public:
  void start(PubMaster *pm_) {
    std::lock_guard lk(mtx);
    pm = pm_;
    if (running) return;
    running = true;
    worker = std::thread(&ThumbnailWorker::run, this);
  }

  void stop() {
    {
      std::lock_guard lk(mtx);
      running = false;
      pm = nullptr;
    }
    cv.notify_all();
    if (worker.joinable()) {
      worker.join();
    }
  }

  void enqueue(const CameraBuf *buf) {
    if (!buf || !buf->cur_yuv_buf) return;
    const VisionBuf *vb = buf->cur_yuv_buf;
    if (!vb->y || !vb->uv) return;

    ThumbnailJob job;
    job.frame_id = buf->cur_frame_data.frame_id;
    job.timestamp_eof = buf->cur_frame_data.timestamp_eof;
    job.width = (int)vb->width;
    job.height = (int)vb->height;
    job.stride = (int)vb->stride;
    // Keep stride-aware backing so SIMD path can safely read padded rows.
    const size_t nv12_bytes = (size_t)job.stride * job.height * 3 / 2;
    job.nv12.resize(nv12_bytes);
    memcpy(job.nv12.data(), vb->addr, job.nv12.size());

    {
      std::lock_guard lk(mtx);
      if (!running || pm == nullptr) return;
      if (jobs.size() >= 2) {
        jobs.pop();
      }
      jobs.push(std::move(job));
    }
    cv.notify_one();
  }

private:
  void run() {
    util::set_thread_name("CamThumbnail");
    while (true) {
      ThumbnailJob job;
      PubMaster *local_pm = nullptr;
      {
        std::unique_lock lk(mtx);
        cv.wait(lk, [this] { return !running || !jobs.empty(); });
        if (!running && jobs.empty()) {
          return;
        }
        job = std::move(jobs.front());
        jobs.pop();
        local_pm = pm;
      }
      if (local_pm != nullptr) {
        publish_thumbnail(local_pm, job);
      }
    }
  }

  std::mutex mtx;
  std::condition_variable cv;
  std::queue<ThumbnailJob> jobs;
  std::thread worker;
  PubMaster *pm = nullptr;
  bool running = false;
};

static ThumbnailWorker g_thumbnail_worker;

void start_thumbnail_worker(PubMaster *pm) {
  g_thumbnail_worker.start(pm);
}

void stop_thumbnail_worker() {
  g_thumbnail_worker.stop();
}

void enqueue_thumbnail(const CameraBuf *buf) {
  g_thumbnail_worker.enqueue(buf);
}

void *processing_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback) {
  const char *thread_name = nullptr;
  if (cs == &cameras->road_cam) {
    thread_name = "RoadCamera";
  } else if (cs == &cameras->driver_cam) {
    thread_name = "DriverCamera";
  } else {
    thread_name = "WideRoadCamera";
  }
  util::set_thread_name(thread_name);

  uint32_t cnt = 0;
  while (!do_exit) {
    if (!cs->buf.acquire()) continue;

    callback(cameras, cs, cnt);

    if (cs == &(cameras->road_cam) && cameras->pm && cnt % 100 == 3) {
      constexpr uint64_t kThumbnailStabilizationNs = 500'000'000ULL;
      const uint64_t now = nanos_since_boot();
      if (cs->stream_start_ns > 0 && (now - cs->stream_start_ns) >= kThumbnailStabilizationNs) {
        enqueue_thumbnail(&(cs->buf));
      }
    }
    ++cnt;
  }
  return NULL;
}

std::thread start_process_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback) {
  return std::thread(processing_thread, cameras, cs, callback);
}

// Publish road camera thumbnail for app preview (same format as loggerd's JpegEncoder)
static void publish_thumbnail(PubMaster *pm, const ThumbnailJob &job) {
  if (!pm || job.nv12.empty()) return;
  const int tw = 480, th = 240;  // thumbnail size (width/4, height/4 for 1920x1200)
  const int w = job.width, h = job.height, stride = job.stride;
  if (w < tw || h < th) return;

  // RGA-only path: resize NV12 to thumbnail, then convert thumbnail NV12 -> I420.
  std::vector<uint8_t> src_thumb_nv12((size_t)tw * th * 3 / 2);
  if (!resize_nv12_with_rga(job.nv12.data(), w, h, stride, src_thumb_nv12.data(), tw, th, tw)) {
    static bool rga_warned = false;
    if (!rga_warned) {
      rga_warned = true;
      LOGW("thumbnail RGA resize failed; dropping thumbnail frame");
    }
    return;
  }

  const size_t y_size = (size_t)tw * ((th + 15) & ~15);
  const size_t uv_size = y_size / 4;
  std::vector<uint8_t> y_plane(y_size), u_plane(uv_size), v_plane(uv_size);
  const uint8_t *ty = src_thumb_nv12.data();
  const uint8_t *tuv = ty + (size_t)tw * th;
  int cvt_small = libyuv::NV12ToI420(ty, tw,
                                     tuv, tw,
                                     y_plane.data(), tw,
                                     u_plane.data(), tw / 2,
                                     v_plane.data(), tw / 2,
                                     tw, th);
  if (cvt_small != 0) return;

  unsigned char *out_buffer = nullptr;
  unsigned long out_size = 0;
  {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &out_buffer, &out_size);

    cinfo.image_width = tw;
    cinfo.image_height = th;
    cinfo.input_components = 3;
    jpeg_set_defaults(&cinfo);
    jpeg_set_colorspace(&cinfo, JCS_YCbCr);
    cinfo.comp_info[0].h_samp_factor = 2;
    cinfo.comp_info[0].v_samp_factor = 2;
    cinfo.comp_info[1].h_samp_factor = 1;
    cinfo.comp_info[1].v_samp_factor = 1;
    cinfo.comp_info[2].h_samp_factor = 1;
    cinfo.comp_info[2].v_samp_factor = 1;
    cinfo.raw_data_in = TRUE;
    jpeg_set_quality(&cinfo, 50, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW y_rows[16], u_rows[8], v_rows[8];
    JSAMPARRAY planes[3] = {y_rows, u_rows, v_rows};
    for (int line = 0; line < th; line += 16) {
      for (int i = 0; i < 16; i++) {
        y_rows[i] = y_plane.data() + (line + i) * tw;
        if (i % 2 == 0) {
          int off = (tw / 2) * ((line + i) / 2);
          u_rows[i / 2] = u_plane.data() + off;
          v_rows[i / 2] = v_plane.data() + off;
        }
      }
      jpeg_write_raw_data(&cinfo, planes, 16);
    }
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
  }

  MessageBuilder msg;
  auto ev = msg.initEvent().initThumbnail();
  ev.setFrameId(job.frame_id);
  ev.setTimestampEof(job.timestamp_eof);
  ev.setThumbnail(kj::arrayPtr(reinterpret_cast<const uint8_t *>(out_buffer), out_size));
  pm->send("thumbnail", msg);

  free(out_buffer);
}

void camerad_thread() {
  cl_device_id device_id = nullptr;
  cl_context context = nullptr;

  cl_device_id cl_device = cl_get_device_id_optional(CL_DEVICE_TYPE_DEFAULT);
  if (cl_device) {
    cl_platform_id device_platform;
    if (clGetDeviceInfo(cl_device, CL_DEVICE_PLATFORM, sizeof(cl_platform_id), &device_platform, NULL) == CL_SUCCESS) {
      const cl_context_properties props[] = {CL_CONTEXT_PLATFORM, (cl_context_properties)device_platform, 0};
      cl_int cl_err = CL_INVALID_VALUE;
      context = clCreateContext(props, 1, &cl_device, NULL, NULL, &cl_err);
      if (context && cl_err == CL_SUCCESS) {
        device_id = cl_device;
      } else {
        if (context) {
          clReleaseContext(context);
          context = nullptr;
        }
        LOGW("OpenCL context creation failed (err=%d), running without OpenCL", cl_err);
      }
    }
  } else {
    LOGW("No OpenCL device found, running without OpenCL");
  }

  {
    MultiCameraState cameras = {};
    VisionIpcServer vipc_server("camerad", device_id, context);

    cameras_open(&cameras);
    cameras_init(&vipc_server, &cameras, device_id, context);
    start_thumbnail_worker(cameras.pm);

    vipc_server.start_listener();

    cameras_run(&cameras);
  }
  if (context) {
    CL_CHECK(clReleaseContext(context));
  }
}

int open_v4l_by_name_and_index(const char name[], int index, int flags) {
  for (int v4l_index = 0; /**/; ++v4l_index) {
    std::string v4l_name = util::read_file(util::string_format("/sys/class/video4linux/video%d/name", v4l_index));
    if (v4l_name.empty()) return -1;
    if (v4l_name.find(name) == 0) {
      if (index == 0) {
        return HANDLE_EINTR(open(util::string_format("/dev/video%d", v4l_index).c_str(), flags));
      }
      index--;
    }
  }
}
