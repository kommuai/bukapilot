#pragma clang diagnostic ignored "-Wexceptions"

#include "selfdrive/modeld/runners/dmonitoring_rknnmodel.h"

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

struct DMonitoringRKNNModel::ModelCtx {
  rknn_context ctx = 0;
  rknn_input_output_num io_num = {};
  std::vector<rknn_tensor_attr> input_attrs;
  std::vector<rknn_tensor_attr> output_attrs;
  std::vector<rknn_input> rknn_inputs;
  std::vector<rknn_output> rknn_outputs;
  std::vector<std::vector<half>> input_bufs;
  rknn_perf_run perf_run = {};
  // Cached indices to avoid per-frame name lookup.
  int idx_img = -1;
  int idx_calib = -1;
};

namespace {
std::string lower_copy(const char *s) {
  std::string out = s ? s : "";
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return std::tolower(c); });
  return out;
}

const std::array<half, 256> &u8_to_half_lut() {
  static const std::array<half, 256> lut = [] {
    std::array<half, 256> t = {};
    for (int i = 0; i < 256; i++) t[i] = float_to_half(static_cast<float>(i));
    return t;
  }();
  return lut;
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

}  // namespace

void DMonitoringRKNNModel::load_model(const std::string& path, DMonitoringRKNNModel::ModelCtx* out) {
  std::string model_data = util::read_file(path);
  std::vector<unsigned char> buffer(model_data.begin(), model_data.end());
  unsigned char* modelptr = buffer.data();
  size_t model_len = buffer.size();
  assert(model_len > 0);

  RKNN_CHECK(rknn_init(&out->ctx, (void*)modelptr, model_len, RKNN_FLAG_EXECUTE_FALLBACK_PRIOR_DEVICE_GPU, NULL));
  rknn_set_core_mask(out->ctx, RKNN_NPU_CORE_0_1);
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

DMonitoringRKNNModel::DMonitoringRKNNModel(const std::string& model_path, float* output)
    : ctx_(new ModelCtx()),
      output_(output),
      run_us_(0) {
  load_model(model_path, ctx_);
  ctx_->idx_img = find_input_index(ctx_->input_attrs, ctx_->io_num.n_input, {"input_img", "img"});
  ctx_->idx_calib = find_input_index(ctx_->input_attrs, ctx_->io_num.n_input, {"calib"});
  if (ctx_->idx_img < 0) ctx_->idx_img = 0;
  if (ctx_->idx_calib < 0 || ctx_->idx_calib == ctx_->idx_img) ctx_->idx_calib = (ctx_->idx_img == 0) ? 1 : 0;
  assert(ctx_->io_num.n_input >= 2 && ctx_->io_num.n_output == 1);
  ctx_->rknn_outputs[0].buf = output_;
  ctx_->rknn_outputs[0].size = ctx_->output_attrs[0].n_elems * sizeof(float);
  ctx_->rknn_outputs[0].is_prealloc = 1;
  LOGD("DMonitoringRKNNModel: %u in / %u out, NPU cores 0+1\n",
       ctx_->io_num.n_input, ctx_->io_num.n_output);
}

DMonitoringRKNNModel::~DMonitoringRKNNModel() {
  if (ctx_ && ctx_->ctx) {
    rknn_destroy(ctx_->ctx);
  }
  delete ctx_;
}

void DMonitoringRKNNModel::run(const unsigned char* input_img, const float* calib) {
  ModelCtx* m = ctx_;
  assert(m->io_num.n_input >= 2);
  const int idx_img = m->idx_img;
  const int idx_calib = m->idx_calib;

  const uint32_t n_img = m->input_attrs[idx_img].n_elems;
  const uint32_t n_calib = m->input_attrs[idx_calib].n_elems;
  half* buf_img = m->input_bufs[idx_img].data();
  half* buf_calib = m->input_bufs[idx_calib].data();
  const auto &lut = u8_to_half_lut();
  for (uint32_t i = 0; i < n_img; i++) {
    buf_img[i] = lut[input_img[i]];
  }
  float_to_half_array(const_cast<float*>(calib), buf_calib, n_calib);
  RKNN_CHECK(rknn_inputs_set(m->ctx, m->io_num.n_input, m->rknn_inputs.data()));
  RKNN_CHECK(rknn_run(m->ctx, NULL));
  RKNN_CHECK(rknn_outputs_get(m->ctx, m->io_num.n_output, m->rknn_outputs.data(), NULL));
  RKNN_CHECK(rknn_query(m->ctx, RKNN_QUERY_PERF_RUN, &m->perf_run, sizeof(m->perf_run)));
  run_us_ = m->perf_run.run_duration;
}
