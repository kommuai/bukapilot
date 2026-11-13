#pragma once

#include <cstddef>
#include <string>

/**
 * Driving model runner using RKNN C++ API (vision + policy).
 * All inputs are converted to float16 before inference.
 * Use when USE_RKNN=1 and driving_vision.rknn / driving_policy.rknn exist.
 */
class DrivingRKNNModel {
 public:
  /**
   * @param vision_path Path to driving_vision.rknn
   * @param policy_path Path to driving_policy.rknn
   * @param vision_output Pre-allocated float buffer for vision output (size 1576)
   * @param policy_output Pre-allocated float buffer for policy output (size 1000)
   */
  DrivingRKNNModel(const std::string& vision_path,
                   const std::string& policy_path,
                   float* vision_output,
                   float* policy_output);
  ~DrivingRKNNModel();

  /** Run vision model. img and big_img are uint8 NCHW (1,12,128,256). Converted to float16 (raw 0..255) internally. */
  void run_vision(const unsigned char* img, const unsigned char* big_img);

  /** Run policy model. All inputs float32; converted to float16 internally. */
  void run_policy(const float* desire_pulse,   // (1, 25, 8) = 200
                 const float* traffic_convention,  // (1, 2) = 2
                 const float* features_buffer);    // (1, 25, 512) = 12800

  /** Last vision run duration in microseconds (from rknn_query PERF_RUN). */
  int64_t get_vision_run_us() const { return vision_run_us_; }
  /** Last policy run duration in microseconds. */
  int64_t get_policy_run_us() const { return policy_run_us_; }

 private:
  struct ModelCtx;
  static void load_model(const std::string& path, ModelCtx* out);
  ModelCtx* vision_ctx_;
  ModelCtx* policy_ctx_;
  float* vision_output_;
  float* policy_output_;
  int64_t vision_run_us_;
  int64_t policy_run_us_;
};
