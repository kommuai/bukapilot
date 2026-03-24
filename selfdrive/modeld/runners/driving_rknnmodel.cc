#pragma clang diagnostic ignored "-Wexceptions"

#include "selfdrive/modeld/runners/driving_rknnmodel.h"

#include <assert.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#include "common/util.h"
#include "common/rkutil.h"
#include "common/swaglog.h"

#define RKNN_CHECK(_expr) do { assert((_expr) == RKNN_SUCC); } while (0)

struct DrivingRKNNModel::ModelCtx {
  rknn_context ctx = 0;
  rknn_input_output_num io_num = {};
  std::vector<rknn_tensor_attr> input_attrs;
  std::vector<rknn_tensor_attr> output_attrs;
  std::vector<rknn_input> rknn_inputs;
  std::vector<rknn_output> rknn_outputs;
  std::vector<std::vector<half>> input_bufs;  // float16 input buffers
  rknn_perf_run perf_run = {};
  // Cached input indices to avoid per-frame string lookup.
  int idx_img = -1;
  int idx_big = -1;
  int idx_dp = -1;
  int idx_tc = -1;
  int idx_fb = -1;
};

namespace {
rknn_core_mask parse_driving_core_mask() {
  // Stability default: pin driving model to NPU core 2.
  // Can override with RKNN_DRIVING_CORE_MASK=0|1|2|0_1|0_1_2
  std::string v = std::getenv("RKNN_DRIVING_CORE_MASK") ? std::getenv("RKNN_DRIVING_CORE_MASK") : "2";
  if (v == "0") return RKNN_NPU_CORE_0;
  if (v == "1") return RKNN_NPU_CORE_1;
  if (v == "2") return RKNN_NPU_CORE_2;
  if (v == "0_1") return RKNN_NPU_CORE_0_1;
  if (v == "0_1_2") return RKNN_NPU_CORE_0_1_2;
  return RKNN_NPU_CORE_0_1_2;
}

std::string lower_copy(const char *s) {
  std::string out = s ? s : "";
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return std::tolower(c); });
  return out;
}

bool enforce_vision_nchw_contract() {
  // Opt-in strict NCHW contract check for debugging/export validation.
  return std::string(std::getenv("RKNN_ENFORCE_VISION_NCHW") ? std::getenv("RKNN_ENFORCE_VISION_NCHW") : "0") != "0";
}

int find_input_index(const std::vector<rknn_tensor_attr> &attrs, uint32_t n_input, const std::vector<std::string> &needles) {
  // Pass 1: exact name match.
  for (uint32_t i = 0; i < n_input; i++) {
    const std::string n = lower_copy(attrs[i].name);
    for (const auto &needle : needles) {
      if (n == needle) {
        return static_cast<int>(i);
      }
    }
  }
  // Pass 2: substring fallback for variant names.
  for (uint32_t i = 0; i < n_input; i++) {
    const std::string n = lower_copy(attrs[i].name);
    for (const auto &needle : needles) {
      if (n.find(needle) != std::string::npos) {
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}

const std::array<half, 256> &u8_to_half_lut() {
  static const std::array<half, 256> lut = [] {
    std::array<half, 256> t = {};
    for (int i = 0; i < 256; i++) t[i] = float_to_half(static_cast<float>(i));
    return t;
  }();
  return lut;
}

const std::array<half, 256> &u8_to_half_bigimg_affine_lut() {
  // Empirical NHWC stabilization for big_img branch. Defaults can be overridden.
  // y = clip(scale*x + bias, 0, 255)
  static const std::array<half, 256> lut = [] {
    const float scale = std::getenv("RKNN_NHWC_BIGIMG_SCALE") ? std::atof(std::getenv("RKNN_NHWC_BIGIMG_SCALE")) : 0.55f;
    const float bias = std::getenv("RKNN_NHWC_BIGIMG_BIAS") ? std::atof(std::getenv("RKNN_NHWC_BIGIMG_BIAS")) : -6.0f;
    std::array<half, 256> t = {};
    for (int i = 0; i < 256; i++) {
      float v = scale * static_cast<float>(i) + bias;
      v = std::min(255.0f, std::max(0.0f, v));
      t[i] = float_to_half(v);
    }
    return t;
  }();
  return lut;
}

bool nhwc_bigimg_affine_enabled() {
  return std::string(std::getenv("RKNN_NHWC_BIGIMG_AFFINE_ENABLE") ? std::getenv("RKNN_NHWC_BIGIMG_AFFINE_ENABLE") : "1") != "0";
}

void fill_vision_input_half_from_u8(const unsigned char* src_nchw_u8, const rknn_tensor_attr &dst_attr, half* dst_half,
                                    const std::array<half, 256> &lut) {
  // Modeld produces NCHW packed tensors (1,12,128,256).
  // Convert directly to model-native layout without temporary allocations.
  constexpr int N = 1;
  constexpr int C = 12;
  constexpr int H = 128;
  constexpr int W = 256;
  constexpr int HW = H * W;
  constexpr int n = N * C * H * W;
  if (dst_attr.fmt == RKNN_TENSOR_NCHW) {
    // Unrolled hot loop: uint8 -> fp16 LUT conversion.
    int i = 0;
    for (; i + 8 <= n; i += 8) {
      dst_half[i + 0] = lut[src_nchw_u8[i + 0]];
      dst_half[i + 1] = lut[src_nchw_u8[i + 1]];
      dst_half[i + 2] = lut[src_nchw_u8[i + 2]];
      dst_half[i + 3] = lut[src_nchw_u8[i + 3]];
      dst_half[i + 4] = lut[src_nchw_u8[i + 4]];
      dst_half[i + 5] = lut[src_nchw_u8[i + 5]];
      dst_half[i + 6] = lut[src_nchw_u8[i + 6]];
      dst_half[i + 7] = lut[src_nchw_u8[i + 7]];
    }
    for (; i < n; i++) dst_half[i] = lut[src_nchw_u8[i]];
    return;
  }

  if (dst_attr.fmt == RKNN_TENSOR_NHWC) {
    // NCHW -> NHWC (N=1): [C,H,W] -> [H,W,C]
    for (int h = 0; h < H; h++) {
      for (int w = 0; w < W; w++) {
        const int dst_base = (h * W + w) * C;
        const int src_hw = h * W + w;
        // Manual C=12 unroll to minimize loop overhead in the hot path.
        const unsigned char *p = src_nchw_u8 + src_hw;
        __builtin_prefetch(p + 32, 0, 1);
        dst_half[dst_base + 0] = lut[p[0 * HW]];
        dst_half[dst_base + 1] = lut[p[1 * HW]];
        dst_half[dst_base + 2] = lut[p[2 * HW]];
        dst_half[dst_base + 3] = lut[p[3 * HW]];
        dst_half[dst_base + 4] = lut[p[4 * HW]];
        dst_half[dst_base + 5] = lut[p[5 * HW]];
        dst_half[dst_base + 6] = lut[p[6 * HW]];
        dst_half[dst_base + 7] = lut[p[7 * HW]];
        dst_half[dst_base + 8] = lut[p[8 * HW]];
        dst_half[dst_base + 9] = lut[p[9 * HW]];
        dst_half[dst_base + 10] = lut[p[10 * HW]];
        dst_half[dst_base + 11] = lut[p[11 * HW]];
      }
    }
    return;
  }

  // Unknown layout: keep fail-fast so we don't silently corrupt input semantics.
  LOGE("Unsupported RKNN input format for vision conversion: %d", dst_attr.fmt);
  assert(false);
}
}  // namespace

void DrivingRKNNModel::load_model(const std::string& path, DrivingRKNNModel::ModelCtx* out) {
  std::string model_data = util::read_file(path);
  std::vector<unsigned char> buffer(model_data.begin(), model_data.end());
  unsigned char* modelptr = buffer.data();
  size_t model_len = buffer.size();
  assert(model_len > 0);

  RKNN_CHECK(rknn_init(&out->ctx, (void*)modelptr, model_len, RKNN_FLAG_EXECUTE_FALLBACK_PRIOR_DEVICE_GPU, NULL));
  rknn_set_core_mask(out->ctx, parse_driving_core_mask());
  rknn_write_driver_version_to_shm(out->ctx);

  RKNN_CHECK(rknn_query(out->ctx, RKNN_QUERY_IN_OUT_NUM, &out->io_num, sizeof(out->io_num)));
  out->input_attrs.resize(out->io_num.n_input);
  out->output_attrs.resize(out->io_num.n_output);
  out->rknn_inputs.resize(out->io_num.n_input);
  out->rknn_outputs.resize(out->io_num.n_output);
  out->input_bufs.resize(out->io_num.n_input);

  for (uint32_t i = 0; i < out->io_num.n_input; i++) {
    out->input_attrs[i].index = i;
    RKNN_CHECK(rknn_query(out->ctx, RKNN_QUERY_INPUT_ATTR, &out->input_attrs[i], sizeof(rknn_tensor_attr)));
    size_t n_elems = out->input_attrs[i].n_elems;
    out->input_bufs[i].resize(n_elems);
  }
  for (uint32_t i = 0; i < out->io_num.n_output; i++) {
    out->output_attrs[i].index = i;
    RKNN_CHECK(rknn_query(out->ctx, RKNN_QUERY_NATIVE_OUTPUT_ATTR, &out->output_attrs[i], sizeof(rknn_tensor_attr)));
  }

  memset(out->rknn_inputs.data(), 0, out->rknn_inputs.size() * sizeof(rknn_input));
  for (uint32_t i = 0; i < out->io_num.n_input; i++) {
    out->rknn_inputs[i].index = i;
    out->rknn_inputs[i].fmt = out->input_attrs[i].fmt;
    // Match Python RKNN "safe default" path: no explicit pass-through behavior.
    out->rknn_inputs[i].pass_through = 0;
    out->rknn_inputs[i].type = RKNN_TENSOR_FLOAT16;
    out->rknn_inputs[i].size = out->input_attrs[i].size;
    out->rknn_inputs[i].buf = out->input_bufs[i].data();
  }
  memset(out->rknn_outputs.data(), 0, out->rknn_outputs.size() * sizeof(rknn_output));
  for (uint32_t i = 0; i < out->io_num.n_output; i++) {
    out->rknn_outputs[i].want_float = 1;
    out->rknn_outputs[i].index = i;
    out->rknn_outputs[i].is_prealloc = 0;
  }
}

DrivingRKNNModel::DrivingRKNNModel(const std::string& vision_path,
                                   const std::string& policy_path,
                                   float* vision_output,
                                   float* policy_output)
    : vision_ctx_(new ModelCtx()),
      policy_ctx_(new ModelCtx()),
      vision_output_(vision_output),
      policy_output_(policy_output),
      vision_run_us_(0),
      policy_run_us_(0) {
  load_model(vision_path, vision_ctx_);
  load_model(policy_path, policy_ctx_);
  vision_ctx_->idx_img = std::max(0, find_input_index(vision_ctx_->input_attrs, vision_ctx_->io_num.n_input, {"img"}));
  vision_ctx_->idx_big = find_input_index(vision_ctx_->input_attrs, vision_ctx_->io_num.n_input, {"big_img", "big"});
  if (vision_ctx_->idx_big < 0 || vision_ctx_->idx_big == vision_ctx_->idx_img) vision_ctx_->idx_big = (vision_ctx_->idx_img == 0) ? 1 : 0;

  policy_ctx_->idx_dp = find_input_index(policy_ctx_->input_attrs, policy_ctx_->io_num.n_input, {"desire_pulse", "desire"});
  policy_ctx_->idx_tc = find_input_index(policy_ctx_->input_attrs, policy_ctx_->io_num.n_input, {"traffic_convention", "traffic"});
  policy_ctx_->idx_fb = find_input_index(policy_ctx_->input_attrs, policy_ctx_->io_num.n_input, {"features_buffer", "features"});
  if (policy_ctx_->idx_dp < 0) policy_ctx_->idx_dp = 0;
  if (policy_ctx_->idx_tc < 0) policy_ctx_->idx_tc = 1;
  if (policy_ctx_->idx_fb < 0) policy_ctx_->idx_fb = 2;
  // Prealloc output buffers so rknn_outputs_get writes directly (avoids extra memcpy).
  // With want_float=1, RKNN expects size = n_elems * sizeof(float), not native tensor size.
  assert(vision_ctx_->io_num.n_output == 1 && policy_ctx_->io_num.n_output == 1);
  vision_ctx_->rknn_outputs[0].buf = vision_output_;
  vision_ctx_->rknn_outputs[0].size = vision_ctx_->output_attrs[0].n_elems * sizeof(float);
  vision_ctx_->rknn_outputs[0].is_prealloc = 1;
  policy_ctx_->rknn_outputs[0].buf = policy_output_;
  policy_ctx_->rknn_outputs[0].size = policy_ctx_->output_attrs[0].n_elems * sizeof(float);
  policy_ctx_->rknn_outputs[0].is_prealloc = 1;
  LOGD("DrivingRKNNModel: vision %u in / %u out, policy %u in / %u out\n",
       vision_ctx_->io_num.n_input, vision_ctx_->io_num.n_output,
       policy_ctx_->io_num.n_input, policy_ctx_->io_num.n_output);
}

DrivingRKNNModel::~DrivingRKNNModel() {
  if (vision_ctx_ && vision_ctx_->ctx) {
    rknn_destroy(vision_ctx_->ctx);
  }
  if (policy_ctx_ && policy_ctx_->ctx) {
    rknn_destroy(policy_ctx_->ctx);
  }
  delete vision_ctx_;
  delete policy_ctx_;
}

void DrivingRKNNModel::run_vision(const unsigned char* img, const unsigned char* big_img) {
  ModelCtx* m = vision_ctx_;
  assert(m->io_num.n_input >= 2);
  const int idx_img = m->idx_img;
  const int idx_big = m->idx_big;

  if (enforce_vision_nchw_contract()) {
    if (m->input_attrs[idx_img].fmt != RKNN_TENSOR_NCHW || m->input_attrs[idx_big].fmt != RKNN_TENSOR_NCHW) {
      LOGE("RKNN vision contract violation: expected NCHW inputs, got img fmt=%d big_img fmt=%d",
           m->input_attrs[idx_img].fmt, m->input_attrs[idx_big].fmt);
      assert(false);
    }
  }

  half* buf_img = m->input_bufs[idx_img].data();
  half* buf_big = m->input_bufs[idx_big].data();
  const auto &lut_default = u8_to_half_lut();
  const auto &lut_big = nhwc_bigimg_affine_enabled() ? u8_to_half_bigimg_affine_lut() : lut_default;
  fill_vision_input_half_from_u8(img, m->input_attrs[idx_img], buf_img, lut_default);
  fill_vision_input_half_from_u8(big_img, m->input_attrs[idx_big], buf_big, lut_big);
  RKNN_CHECK(rknn_inputs_set(m->ctx, m->io_num.n_input, m->rknn_inputs.data()));
  RKNN_CHECK(rknn_run(m->ctx, NULL));
  RKNN_CHECK(rknn_outputs_get(m->ctx, m->io_num.n_output, m->rknn_outputs.data(), NULL));
  RKNN_CHECK(rknn_query(m->ctx, RKNN_QUERY_PERF_RUN, &m->perf_run, sizeof(m->perf_run)));
  vision_run_us_ = m->perf_run.run_duration;
  // Output already in vision_output_ (is_prealloc=1)
}

void DrivingRKNNModel::run_policy(const float* desire_pulse,
                                 const float* traffic_convention,
                                 const float* features_buffer) {
  ModelCtx* m = policy_ctx_;
  assert(m->io_num.n_input >= 3);
  const int idx_dp = m->idx_dp;
  const int idx_tc = m->idx_tc;
  const int idx_fb = m->idx_fb;
  float_to_half_array(const_cast<float*>(desire_pulse), m->input_bufs[idx_dp].data(), m->input_attrs[idx_dp].n_elems);
  float_to_half_array(const_cast<float*>(traffic_convention), m->input_bufs[idx_tc].data(), m->input_attrs[idx_tc].n_elems);
  float_to_half_array(const_cast<float*>(features_buffer), m->input_bufs[idx_fb].data(), m->input_attrs[idx_fb].n_elems);
  RKNN_CHECK(rknn_inputs_set(m->ctx, m->io_num.n_input, m->rknn_inputs.data()));
  RKNN_CHECK(rknn_run(m->ctx, NULL));
  RKNN_CHECK(rknn_outputs_get(m->ctx, m->io_num.n_output, m->rknn_outputs.data(), NULL));
  RKNN_CHECK(rknn_query(m->ctx, RKNN_QUERY_PERF_RUN, &m->perf_run, sizeof(m->perf_run)));
  policy_run_us_ = m->perf_run.run_duration;
  // Output already in policy_output_ (is_prealloc=1)
}
