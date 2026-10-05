#include "selfdrive/modeld/transforms/transpose.h"

#include <cassert>
#include <cstring>

#include "common/clutil.h"

void transpose_init(Transpose* s, cl_context ctx, cl_device_id device_id) {
  memset(s, 0, sizeof(*s));

  cl_program prg = cl_program_from_file(ctx, device_id, TRANSPOSE_PATH, "");
  s->krnl = CL_CHECK_ERR(clCreateKernel(prg, "nchw_to_nhwc", &err));

  // done with this
  CL_CHECK(clReleaseProgram(prg));
}

void transpose_destroy(Transpose* s) {
  CL_CHECK(clReleaseKernel(s->krnl));
}

void transpose_queue(Transpose* s,
                     cl_command_queue q,
                     cl_mem in_nchw, cl_mem out_nhwc) {

  const int N = 1;
  const int C = 12;
  const int H = 128;
  const int W = 256;
  const int total_elem = N * C * H * W;

  CL_CHECK(clSetKernelArg(s->krnl, 0, sizeof(cl_mem), &in_nchw));
  CL_CHECK(clSetKernelArg(s->krnl, 1, sizeof(cl_mem), &out_nhwc));
  CL_CHECK(clSetKernelArg(s->krnl, 2, sizeof(cl_int), &N));
  CL_CHECK(clSetKernelArg(s->krnl, 3, sizeof(cl_int), &C));
  CL_CHECK(clSetKernelArg(s->krnl, 4, sizeof(cl_int), &H));
  CL_CHECK(clSetKernelArg(s->krnl, 5, sizeof(cl_int), &W));

  const size_t global_work_size = total_elem;

  CL_CHECK(clEnqueueNDRangeKernel(q, s->krnl, 1, NULL, &global_work_size, NULL, 0, 0, NULL));
}
