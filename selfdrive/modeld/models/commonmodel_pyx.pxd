# distutils: language = c++

from msgq.visionipc.visionipc cimport cl_mem
from msgq.visionipc.visionipc_pyx cimport CLContext as BaseCLContext
from .commonmodel cimport cl_command_queue

cdef class CLContext(BaseCLContext):
  cdef cl_command_queue queue

cdef class CLMem:
  cdef cl_mem * mem

  @staticmethod
  cdef create(void*)
