#pragma once

#include "rknn_api.h"
#include "common/swaglog.h"

typedef unsigned short half;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef union suf32
{
  int      i;
  unsigned u;
  float    f;
} suf32;

uint as_uint(const float x);
float as_float(const uint x);

float half_to_float(half x);
void float_to_half_array(float *src, half *dst, int size);
void half_to_float_array(half *src, float *dst, int size);

int _rknn_app_nchw_2_nhwc(rknn_tensor_attr *src, unsigned char* src_ptr, rknn_tensor_attr *dst, unsigned char* dst_ptr, int type_size);
int _rknn_app_nhwc_2_nchw(rknn_tensor_attr *src, unsigned char* src_ptr, rknn_tensor_attr *dst, unsigned char* dst_ptr, int type_size);
int _rknn_app_nchw_2_nc1hwc2(rknn_tensor_attr *src, unsigned char* src_ptr, rknn_tensor_attr *dst, unsigned char* dst_ptr, int type_size);
int _rknn_app_nc1hwc2_2_nchw(rknn_tensor_attr *src, unsigned char* src_ptr, rknn_tensor_attr *dst, unsigned char* dst_ptr, int type_size);
int _rknn_app_nhwc_2_nc1hwc2(rknn_tensor_attr *src, unsigned char* src_ptr, rknn_tensor_attr *dst, unsigned char* dst_ptr, int type_size);
int rknn_app_layout_convert(unsigned char* src_ptr,
                            rknn_tensor_attr *src,
                            unsigned char* dst_ptr,
                            rknn_tensor_attr *dst,
                            int type_size,
                            bool verbose=false);

int rknn_app_dtype_convert(unsigned char* src_ptr,
                         rknn_tensor_type src_dtype,
                         unsigned char* dst_ptr,
                         rknn_tensor_type dst_dtype,
                         int n_elems, float scale, int zero_point, bool verbose = false);

void dump_tensor_attr(rknn_tensor_attr* attr);

void print_execution_time(rknn_perf_run* obj);

