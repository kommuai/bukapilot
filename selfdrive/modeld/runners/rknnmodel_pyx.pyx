# distutils: language = c++
# cython: c_string_encoding=ascii

import os
from libcpp cimport bool
from libcpp.string cimport string

from .rknnmodel cimport RKNNModel as cppRKNNModel
from selfdrive.modeld.models.commonmodel_pyx cimport CLContext
from selfdrive.modeld.runners.runmodel_pyx cimport RunModel
from selfdrive.modeld.runners.runmodel cimport RunModel as cppRunModel

os.environ['RKNN_LIBRARY_PATH'] = "/data/pythonpath/third_party/rknpu/aarch64"

cdef class RKNNModel(RunModel):
  def __cinit__(self, string path, float[:] output, int runtime, bool use_tf8, CLContext context):
    self.model = <cppRunModel *> new cppRKNNModel(path, &output[0], len(output), runtime, use_tf8, context.context)