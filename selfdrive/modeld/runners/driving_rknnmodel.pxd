# Cython declarations for DrivingRKNNModel (C++ RKNN driving runner)

from libcpp.string cimport string

cdef extern from "selfdrive/modeld/runners/driving_rknnmodel.h":
    cdef cppclass DrivingRKNNModel:
        DrivingRKNNModel(const string& vision_path,
                        const string& policy_path,
                        float* vision_output,
                        float* policy_output)
        void run_vision(const unsigned char* img, const unsigned char* big_img)
        void run_policy(const float* desire_pulse,
                        const float* traffic_convention,
                        const float* features_buffer)
        long long get_vision_run_us()
        long long get_policy_run_us()
