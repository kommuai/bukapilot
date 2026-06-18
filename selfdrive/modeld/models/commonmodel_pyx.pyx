# distutils: language = c++
# cython: c_string_encoding=ascii, language_level=3

import numpy as np
cimport numpy as cnp
from libc.string cimport memcpy
from libc.stdint cimport uintptr_t

from msgq.visionipc.visionipc cimport cl_mem
from msgq.visionipc.visionipc_pyx cimport VisionBuf, CLContext as BaseCLContext
from .commonmodel cimport CL_DEVICE_TYPE_DEFAULT, cl_get_device_id, cl_create_context, cl_release_context
from .commonmodel cimport cl_create_command_queue, cl_release_command_queue
from .commonmodel cimport mat3, ModelFrame as cppModelFrame, DrivingModelFrame as cppDrivingModelFrame, MonitoringModelFrame as cppMonitoringModelFrame


cdef class CLContext(BaseCLContext):
  def __cinit__(self):
    self.device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT)
    self.context = cl_create_context(self.device_id)
    self.queue = cl_create_command_queue(self.context, self.device_id)

  def __dealloc__(self):
    if self.queue:
      cl_release_command_queue(self.queue)
    if self.context:
      cl_release_context(self.context)

  # Integer pointers for tinygrad external context (zero-copy vision inputs on same CL context)
  @property
  def context_ptr(self):
    return <uintptr_t>self.context

  @property
  def device_id_ptr(self):
    return <uintptr_t>self.device_id

  @property
  def queue_ptr(self):
    return <uintptr_t>self.queue

cdef class CLMem:
  @staticmethod
  cdef create(void * cmem):
    mem = CLMem()
    mem.mem = <cl_mem*> cmem
    return mem

  @property
  def mem_address(self):
    return <uintptr_t>(self.mem)

  # CL buffer handle (cl_mem value) for zero-copy Tensor.from_blob(..., device='CL')
  @property
  def mem_handle(self):
    return <uintptr_t>(self.mem[0])

def cl_from_visionbuf(VisionBuf buf):
  return CLMem.create(<void*>&buf.buf.buf_cl)


cdef class ModelFrame:
  cdef cppModelFrame * frame
  cdef int buf_size

  def __dealloc__(self):
    del self.frame

  def prepare(self, VisionBuf buf, float[:] projection):
    cdef mat3 cprojection
    memcpy(cprojection.v, &projection[0], 9*sizeof(float))
    cdef cl_mem * data
    data = self.frame.prepare(buf.buf.buf_cl, buf.width, buf.height, buf.stride, buf.uv_offset, cprojection)
    return CLMem.create(data)

  def buffer_from_cl(self, CLMem in_frames):
    cdef unsigned char * data2
    data2 = self.frame.buffer_from_cl(in_frames.mem, self.buf_size)
    return np.asarray(<cnp.uint8_t[:self.buf_size]> data2)


cdef class DrivingModelFrame(ModelFrame):
  cdef cppDrivingModelFrame * _frame

  def __cinit__(self, CLContext context, int temporal_skip):
    self._frame = new cppDrivingModelFrame(context.device_id, context.context, temporal_skip, context.queue)
    self.frame = <cppModelFrame*>(self._frame)
    self.buf_size = self._frame.buf_size

cdef class MonitoringModelFrame(ModelFrame):
  cdef cppMonitoringModelFrame * _frame

  def __cinit__(self, CLContext context):
    self._frame = new cppMonitoringModelFrame(context.device_id, context.context, context.queue)
    self.frame = <cppModelFrame*>(self._frame)
    self.buf_size = self._frame.buf_size
