# Cython declarations for DMonitoringRKNNModel

from libcpp.string cimport string

cdef extern from "selfdrive/modeld/runners/dmonitoring_rknnmodel.h":
    cdef cppclass DMonitoringRKNNModel:
        DMonitoringRKNNModel(const string& model_path, float* output)
        void run(const unsigned char* input_img, const float* calib)
        long long get_run_us()
