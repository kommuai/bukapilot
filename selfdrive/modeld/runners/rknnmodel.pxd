# distutils: language = c++

from libcpp.string cimport string

from cereal.visionipc.visionipc cimport cl_context

cdef extern from "selfdrive/modeld/runners/rknnmodel.h":
  cdef cppclass RKNNModel:
    RKNNModel(string, float*, size_t, int, bool, cl_context)