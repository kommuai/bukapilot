"""Work around NVIDIA OpenCL half-precision exp2 code generation in tinygrad."""

import tinygrad.renderer.cstyle as cstyle
from tinygrad.dtype import dtypes

cstyle.OpenCLRenderer.extra_matcher = (
  cstyle.create_non_native_float_pats((dtypes.half,)) + cstyle.OpenCLRenderer.extra_matcher
)
