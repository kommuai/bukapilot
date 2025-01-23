#include "system/camerad/cameras/camera_common.h"

#include <cassert>
#include <string>
#include <iostream>

#include "third_party/libyuv/include/libyuv.h"
#include <jpeglib.h>
#include <signal.h>

#include "common/clutil.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "third_party/linux/include/msm_media_info.h"

#include "system/camerad/cameras/camera_rk.h"
#ifdef QCOM2
#include "CL/cl_ext_qcom.h"
#endif

ExitHandler do_exit;

void CameraBuf::init(cl_device_id device_id, cl_context context, CameraState *s, VisionIpcServer * v, int frame_cnt, VisionStreamType type) {
  vipc_server = v;
  stream_type = type;
  frame_buf_count = frame_cnt;

  rgb_width = 1920;
  rgb_height = 1200;

  // NV12 frame
  nv12_frame_size = (rgb_width * rgb_height * 3)/2;
  camera_bufs = std::make_unique<VisionBuf[]>(frame_buf_count);
  camera_bufs_metadata = std::make_unique<FrameMetadata[]>(frame_buf_count);

  int nv12_width = rgb_width;
  int nv12_height = rgb_height;
  size_t nv12_size = nv12_frame_size;
  size_t nv12_uv_offset = nv12_width * nv12_height;

  for (int i = 0; i < frame_buf_count; i++) {
    camera_bufs[i].allocate(nv12_frame_size);
  }
  LOGD("allocated %d buffers", frame_buf_count);

  vipc_server->create_buffers_with_sizes(stream_type, YUV_BUFFER_COUNT, false, rgb_width, rgb_height, nv12_size, nv12_width, nv12_uv_offset);
  LOGD("created %d YUV vipc buffers with size %dx%d", YUV_BUFFER_COUNT, nv12_width, nv12_height);
}

CameraBuf::~CameraBuf() {
  for (int i = 0; i < frame_buf_count; i++) {
    camera_bufs[i].free();
  }

  // needed due to (maybe) buggy opencl
  kill(getpid(), SIGKILL);
}

bool CameraBuf::acquire() {
  if (!safe_queue.try_pop(cur_buf_idx, 50)) return false;

  cur_frame_data = camera_bufs_metadata[cur_buf_idx];

  cur_yuv_buf = vipc_server->get_buffer(stream_type);
  cur_camera_buf = &camera_bufs[cur_buf_idx];

  memcpy(cur_yuv_buf->addr, cur_camera_buf->addr, nv12_frame_size);

  VisionIpcBufExtra extra = {
    cur_frame_data.frame_id,
    cur_frame_data.timestamp_sof,
    cur_frame_data.timestamp_eof,
  };

  cur_yuv_buf->set_frame_id(cur_frame_data.frame_id);
  vipc_server->send(cur_yuv_buf, &extra, false);
  return true;
}

void CameraBuf::queue(size_t buf_idx) {
  safe_queue.push(buf_idx);
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
}

kj::Array<uint8_t> get_raw_frame_image(const CameraBuf *b) {
  const uint8_t *dat = (const uint8_t *)b->cur_camera_buf->addr;

  kj::Array<uint8_t> frame_image = kj::heapArray<uint8_t>(b->cur_camera_buf->len);
  uint8_t *resized_dat = frame_image.begin();

  memcpy(resized_dat, dat, b->cur_camera_buf->len);

  return kj::mv(frame_image);
}

static kj::Array<capnp::byte> yuv420_to_jpeg(const CameraBuf *b, int thumbnail_width, int thumbnail_height) {
  int downscale = b->cur_yuv_buf->width / thumbnail_width;
  assert(downscale * thumbnail_height == b->cur_yuv_buf->height);
  int in_stride = b->cur_yuv_buf->stride;

  // make the buffer big enough. jpeg_write_raw_data requires 16-pixels aligned height to be used.
  std::unique_ptr<uint8[]> buf(new uint8_t[(thumbnail_width * ((thumbnail_height + 15) & ~15) * 3) / 2]);
  uint8_t *y_plane = buf.get();
  uint8_t *u_plane = y_plane + thumbnail_width * thumbnail_height;
  uint8_t *v_plane = u_plane + (thumbnail_width * thumbnail_height) / 4;
  {
    // subsampled conversion from nv12 to yuv
    for (int hy = 0; hy < thumbnail_height/2; hy++) {
      for (int hx = 0; hx < thumbnail_width/2; hx++) {
        int ix = hx * downscale + (downscale-1)/2;
        int iy = hy * downscale + (downscale-1)/2;
        y_plane[(hy*2 + 0)*thumbnail_width + (hx*2 + 0)] = b->cur_yuv_buf->y[(iy*2 + 0) * in_stride + ix*2 + 0];
        y_plane[(hy*2 + 0)*thumbnail_width + (hx*2 + 1)] = b->cur_yuv_buf->y[(iy*2 + 0) * in_stride + ix*2 + 1];
        y_plane[(hy*2 + 1)*thumbnail_width + (hx*2 + 0)] = b->cur_yuv_buf->y[(iy*2 + 1) * in_stride + ix*2 + 0];
        y_plane[(hy*2 + 1)*thumbnail_width + (hx*2 + 1)] = b->cur_yuv_buf->y[(iy*2 + 1) * in_stride + ix*2 + 1];
        u_plane[hy*thumbnail_width/2 + hx] = b->cur_yuv_buf->uv[iy*in_stride + ix*2 + 0];
        v_plane[hy*thumbnail_width/2 + hx] = b->cur_yuv_buf->uv[iy*in_stride + ix*2 + 1];
      }
    }
  }

  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  uint8_t *thumbnail_buffer = nullptr;
  size_t thumbnail_len = 0;
  jpeg_mem_dest(&cinfo, &thumbnail_buffer, &thumbnail_len);

  cinfo.image_width = thumbnail_width;
  cinfo.image_height = thumbnail_height;
  cinfo.input_components = 3;

  jpeg_set_defaults(&cinfo);
  jpeg_set_colorspace(&cinfo, JCS_YCbCr);
  // configure sampling factors for yuv420.
  cinfo.comp_info[0].h_samp_factor = 2;  // Y
  cinfo.comp_info[0].v_samp_factor = 2;
  cinfo.comp_info[1].h_samp_factor = 1;  // U
  cinfo.comp_info[1].v_samp_factor = 1;
  cinfo.comp_info[2].h_samp_factor = 1;  // V
  cinfo.comp_info[2].v_samp_factor = 1;
  cinfo.raw_data_in = TRUE;

  jpeg_set_quality(&cinfo, 50, TRUE);
  jpeg_start_compress(&cinfo, TRUE);

  JSAMPROW y[16], u[8], v[8];
  JSAMPARRAY planes[3]{y, u, v};

  for (int line = 0; line < cinfo.image_height; line += 16) {
    for (int i = 0; i < 16; ++i) {
      y[i] = y_plane + (line + i) * cinfo.image_width;
      if (i % 2 == 0) {
        int offset = (cinfo.image_width / 2) * ((i + line) / 2);
        u[i / 2] = u_plane + offset;
        v[i / 2] = v_plane + offset;
      }
    }
    jpeg_write_raw_data(&cinfo, planes, 16);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);

  kj::Array<capnp::byte> dat = kj::heapArray<capnp::byte>(thumbnail_buffer, thumbnail_len);
  free(thumbnail_buffer);
  return dat;
}

static void publish_thumbnail(PubMaster *pm, const CameraBuf *b) {
  auto thumbnail = yuv420_to_jpeg(b, b->rgb_width / 4, b->rgb_height / 4);
  if (thumbnail.size() == 0) return;

  MessageBuilder msg;
  auto thumbnaild = msg.initEvent().initThumbnail();
  thumbnaild.setFrameId(b->cur_frame_data.frame_id);
  thumbnaild.setTimestampEof(b->cur_frame_data.timestamp_eof);
  thumbnaild.setThumbnail(thumbnail);

  pm->send("thumbnail", msg);
}

float set_exposure_target(const CameraBuf *b, int x_start, int x_end, int x_skip, int y_start, int y_end, int y_skip) {
  int lum_med;
  uint32_t lum_binning[256] = {0};
  const uint8_t *pix_ptr = b->cur_yuv_buf->y;

  unsigned int lum_total = 0;
  for (int y = y_start; y < y_end; y += y_skip) {
    for (int x = x_start; x < x_end; x += x_skip) {
      uint8_t lum = pix_ptr[(y * b->rgb_width) + x];
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
      // this takes 10ms???
      publish_thumbnail(cameras->pm, &(cs->buf));
    }
    ++cnt;
  }
  return NULL;
}

std::thread start_process_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback) {
  return std::thread(processing_thread, cameras, cs, callback);
}

void camerad_thread() {
  cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);

  cl_platform_id device_platform;
  clGetDeviceInfo(device_id, CL_DEVICE_PLATFORM, sizeof(cl_platform_id), &device_platform, NULL);
  const cl_context_properties props[] = {CL_CONTEXT_PLATFORM, (cl_context_properties)device_platform, 0};
  cl_context context = CL_CHECK_ERR(clCreateContext(props, 1, &device_id, NULL, NULL, &err));

  {
    MultiCameraState cameras = {};
    VisionIpcServer vipc_server("camerad", device_id, context);
    cameras_open(&cameras);
    cameras_init(&vipc_server, &cameras, device_id, context);

    vipc_server.start_listener();

    cameras_run(&cameras);
  }
  CL_CHECK(clReleaseContext(context));
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
