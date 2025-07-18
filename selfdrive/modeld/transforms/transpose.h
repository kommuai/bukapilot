#pragma once

#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

typedef struct {
  cl_kernel krnl;
} Transpose;

void transpose_init(Transpose* s, cl_context ctx, cl_device_id device_id);

void transpose_destroy(Transpose* transpose);

void transpose_queue(Transpose* s, cl_command_queue q,
                     cl_mem in_nchw, cl_mem out_nhwc);
