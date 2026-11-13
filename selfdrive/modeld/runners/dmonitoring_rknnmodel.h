#pragma once

#include <cstddef>
#include <string>

/**
 * DMonitoring model runner using RKNN C++ API.
 * Inputs: input_img (uint8) and calib (float32), both converted to float16 before inference.
 * Runs on NPU cores 0 and 1 (RKNN_NPU_CORE_0_1). modeld uses core 2 (RKNN_NPU_CORE_2) so cores are disjoint.
 */
class DMonitoringRKNNModel {
 public:
  /**
   * @param model_path Path to dmonitoring_model.rknn
   * @param output Pre-allocated float buffer for model output (size from model, typically 84)
   */
  DMonitoringRKNNModel(const std::string& model_path, float* output);
  ~DMonitoringRKNNModel();

  /** Run model. input_img uint8; calib float32. Both converted to float16 internally. */
  void run(const unsigned char* input_img, const float* calib);

  /** Last run duration in microseconds (from rknn_query PERF_RUN). */
  int64_t get_run_us() const { return run_us_; }

 private:
  struct ModelCtx;
  static void load_model(const std::string& path, ModelCtx* out);
  ModelCtx* ctx_;
  float* output_;
  int64_t run_us_;
};
