#include "msgq/visionipc/visionbuf.h"

void VisionBuf::init_yuv(size_t init_width, size_t init_height, size_t init_stride, size_t init_uv_offset){
  this->width = init_width;
  this->height = init_height;
  this->stride = init_stride;
  this->uv_offset = init_uv_offset;

  this->y = (uint8_t *)this->addr;
  this->uv = this->y + this->uv_offset;
}


uint64_t VisionBuf::get_frame_id() {
  if (frame_id_in_buf && frame_id != nullptr) {
    return *frame_id;
  }
  return frame_id_fallback;
}

void VisionBuf::set_frame_id(uint64_t id) {
  if (frame_id_in_buf && frame_id != nullptr) {
    *frame_id = id;
  } else {
    frame_id_fallback = id;
  }
}
