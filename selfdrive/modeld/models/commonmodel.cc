#include "selfdrive/modeld/models/commonmodel.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "common/clutil.h"

DrivingModelFrame::DrivingModelFrame(cl_device_id device_id, cl_context context, int _temporal_skip, cl_command_queue shared_q) : ModelFrame(device_id, context, shared_q) {
  input_frames = std::make_unique<uint8_t[]>(buf_size);
  temporal_skip = _temporal_skip;
  input_frames_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, buf_size, NULL, &err));
  img_buffer_20hz_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, (temporal_skip+1)*frame_size_bytes, NULL, &err));
  region.origin = temporal_skip * frame_size_bytes;
  region.size = frame_size_bytes;
  last_img_cl = CL_CHECK_ERR(clCreateSubBuffer(img_buffer_20hz_cl, CL_MEM_READ_WRITE, CL_BUFFER_CREATE_TYPE_REGION, &region, &err));

  loadyuv_init(&loadyuv, context, device_id, MODEL_WIDTH, MODEL_HEIGHT);
  init_transform(device_id, context, MODEL_WIDTH, MODEL_HEIGHT);
}

cl_mem* DrivingModelFrame::prepare(cl_mem yuv_cl, int frame_width, int frame_height, int frame_stride, int frame_uv_offset, const mat3& projection) {
  run_transform(yuv_cl, MODEL_WIDTH, MODEL_HEIGHT, frame_width, frame_height, frame_stride, frame_uv_offset, projection);

  for (int i = 0; i < temporal_skip; i++) {
    CL_CHECK(clEnqueueCopyBuffer(q, img_buffer_20hz_cl, img_buffer_20hz_cl, (i+1)*frame_size_bytes, i*frame_size_bytes, frame_size_bytes, 0, nullptr, nullptr));
  }
  loadyuv_queue(&loadyuv, q, y_cl, u_cl, v_cl, last_img_cl);

  copy_queue(&loadyuv, q, img_buffer_20hz_cl, input_frames_cl, 0, 0, frame_size_bytes);
  copy_queue(&loadyuv, q, last_img_cl, input_frames_cl, 0, frame_size_bytes, frame_size_bytes);

  // With shared queue, in-order execution means tinygrad runs after prepare; no finish needed. With own queue, finish so tinygrad (different queue) sees ready buffer.
  if (own_queue && std::getenv("MODELD_SKIP_PREPARE_FINISH") == nullptr) {
    clFinish(q);
  }
  return &input_frames_cl;
}

DrivingModelFrame::~DrivingModelFrame() {
  deinit_transform();
  loadyuv_destroy(&loadyuv);
  CL_CHECK(clReleaseMemObject(input_frames_cl));
  CL_CHECK(clReleaseMemObject(img_buffer_20hz_cl));
  CL_CHECK(clReleaseMemObject(last_img_cl));
  // q is released by ModelFrame destructor when own_queue
}


MonitoringModelFrame::MonitoringModelFrame(cl_device_id device_id, cl_context context, cl_command_queue shared_q) : ModelFrame(device_id, context, shared_q) {
  input_frames = std::make_unique<uint8_t[]>(buf_size);
  input_frame_cl = CL_CHECK_ERR(clCreateBuffer(context, CL_MEM_READ_WRITE, buf_size, NULL, &err));

  init_transform(device_id, context, MODEL_WIDTH, MODEL_HEIGHT);
}

cl_mem* MonitoringModelFrame::prepare(cl_mem yuv_cl, int frame_width, int frame_height, int frame_stride, int frame_uv_offset, const mat3& projection) {
  run_transform(yuv_cl, MODEL_WIDTH, MODEL_HEIGHT, frame_width, frame_height, frame_stride, frame_uv_offset, projection);
  if (own_queue && std::getenv("MODELD_SKIP_PREPARE_FINISH") == nullptr) {
    clFinish(q);
  }
  return &y_cl;
}

MonitoringModelFrame::~MonitoringModelFrame() {
  deinit_transform();
  CL_CHECK(clReleaseMemObject(input_frame_cl));
  // q is released by ModelFrame destructor when own_queue
}
