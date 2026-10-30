"""OpenCL compatibility adjustments for the desktop MetaDrive Tinygrad runner."""

import os

import tinygrad.renderer.cstyle as cstyle
from tinygrad.dtype import dtypes

cstyle.OpenCLRenderer.extra_matcher = (
  cstyle.create_non_native_float_pats((dtypes.half,)) + cstyle.OpenCLRenderer.extra_matcher
)

# tinygrad's Intel renderer hardcodes an 8-lane subgroup for its XMX kernels.
# This MetaDrive path adapts it to the subgroup-16 Intel matrix extension API.
if os.getenv("METADRIVE_GENERIC_OPENCL") == "1":
  import tinygrad.runtime.ops_cl as ops_cl
  from tinygrad.codegen.opt.tc import TensorCore

  # The Intel matrix extension supports subgroup size 16 on this Arc device.
  # tinygrad's stock Intel descriptor assumes 8 lanes, so adapt the tile
  # mapping and the per-lane OpenCL argument shapes to the size-16 API.
  intel_xmx16 = TensorCore(
    dims=(16, 8, 16), threads=16, elements_per_thread=(8, 16, 8),
    dtype_in=dtypes.half, dtype_out=dtypes.float,
    opts=("l0", "l0", "l0", "l0", "u1", "u1", "u1"),
    swizzle=((('r0', 'r1', 'r2', 'r3'), ('u0', 'u1', 'u2'), ('l0', 'l1', 'l2', 'l3')),
             (('l0', 'l1', 'l2', 'l3'), ('r1', 'r2', 'r3'), ('u0', 'u1', 'u2', 'r0'))),
  )

  class MetaDriveIntelXMXRenderer(cstyle.IntelRenderer):
    tensor_cores = [intel_xmx16]
    kernel_typedef = "__attribute__((intel_reqd_sub_group_size(16)))\n__kernel void"

    def render_kernel(self, function_name, kernel, bufs, uops, prefix=None):
      prefix = [
        "#pragma OPENCL EXTENSION cl_khr_fp16 : enable",
        "#pragma OPENCL EXTENSION cl_intel_subgroup_matrix_multiply_accumulate : enable",
      ]
      for name, _, dtype_in, dtype_out, _, _, _, _ in cstyle.wmma_args(uops):
        if dtype_in != dtypes.half or dtype_out != dtypes.float:
          raise TypeError(f"Unsupported Intel XMX types: {dtype_in} -> {dtype_out}")
        prefix.append(f"""float8 __{name}(half8 a, half16 b, float8 c) {{
    return intel_sub_group_f16_f16_matrix_mad_k16(as_short8(a), as_int8(b), c);
}}""")
      return cstyle.OpenCLRenderer.render_kernel(self, function_name, kernel, bufs, uops, prefix)

  # ops_cl selects IntelRenderer only when the device reports the matrix
  # extension; devices without it continue to use the generic OpenCL renderer.
  ops_cl.IntelRenderer = MetaDriveIntelXMXRenderer
