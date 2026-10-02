# Cython wrapper for DMonitoringRKNNModel (C++ RKNN dmonitoring runner)
# Inputs cast to float16 in C++. NPU cores 0+1 (modeld uses core 2).

# distutils: language = c++
# cython: c_string_encoding=ascii

import numpy as np
cimport numpy as np
from libcpp.string cimport string

from .dmonitoring_rknnmodel cimport DMonitoringRKNNModel

cdef class DMonitoringRKNNRunnerCpp:
    """Runs dmonitoring model via C++ RKNN. input_img and calib converted to float16 in C++. NPU cores 0+1."""
    cdef DMonitoringRKNNModel* _model
    cdef float[:] _out
    cdef int _output_size

    def __cinit__(self, str model_path, int output_size):
        self._output_size = output_size
        cdef float[::1] out = np.zeros(output_size, dtype=np.float32)
        self._out = out
        self._model = new DMonitoringRKNNModel(model_path.encode('utf-8'), &out[0])

    def __dealloc__(self):
        if self._model is not NULL:
            del self._model
            self._model = NULL

    def run(self, np.ndarray input_img, np.ndarray calib):
        """input_img: uint8 numpy (driver camera frame). calib: float32 (e.g. 3 elements)."""
        cdef unsigned char[::1] img_flat = np.ascontiguousarray(input_img).reshape(-1).astype(np.uint8)
        cdef float[::1] calib_flat = np.ascontiguousarray(calib, dtype=np.float32).reshape(-1)
        self._model.run(&img_flat[0], &calib_flat[0])
        return np.asarray(self._out[:self._output_size]).copy()

    def get_run_us(self):
        return self._model.get_run_us()
