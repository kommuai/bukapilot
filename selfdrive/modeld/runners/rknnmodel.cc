#pragma clang diagnostic ignored "-Wexceptions"

#include "selfdrive/modeld/runners/rknnmodel.h"

#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/util.h"
#include "common/rkutil.h"
#include "common/timing.h"

RKNNModel::RKNNModel(const std::string path, float *_output, size_t _output_size, int runtime, bool _use_tf8, cl_context context) {
  output = _output;
  output_size = _output_size;
  ctx = 0;

  std::string model_data;
  model_data = util::read_file(path);
  std::vector<unsigned char> buffer(model_data.begin(), model_data.end());
  unsigned char* modelptr = buffer.data();
  size_t model_len = buffer.size();
  assert(model_len > 0);

  // load model
  RKNN_CHECK(rknn_init(&ctx, (void *) modelptr, model_len, RKNN_FLAG_EXECUTE_FALLBACK_PRIOR_DEVICE_GPU, NULL));

  // TODO: NPU core, supercombo CORE0, dmonitoring CORE1, nav CORE2, does it get speed up?
  if (runtime == 1) {
    rknn_set_core_mask(ctx, RKNN_NPU_CORE_0);
  }
  else {
    rknn_set_core_mask(ctx, RKNN_NPU_CORE_2);
  }

  // get sdk and driver version
  rknn_sdk_version sdk_ver;
  RKNN_CHECK(rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &sdk_ver, sizeof(sdk_ver)));
  LOGD("rknnrt version: %s, driver version: %s\n", sdk_ver.api_version, sdk_ver.drv_version);

  // get model input output info
  RKNN_CHECK(rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)));
  LOGD("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

  LOGD("input tensors: \n");
  memset(input_attrs, 0, io_num.n_input * sizeof(rknn_tensor_attr));
  for (uint32_t i = 0; i < io_num.n_input; i++) {
    input_attrs[i].index = i;
    RKNN_CHECK(rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr)));
    dump_tensor_attr(&input_attrs[i]);
  }

  LOGD("output tensors:\n");
  memset(output_attrs, 0, io_num.n_output * sizeof(rknn_tensor_attr));
  for (uint32_t i = 0; i < io_num.n_output; i++) {
    output_attrs[i].index = i;
    RKNN_CHECK(rknn_query(ctx, RKNN_QUERY_NATIVE_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr)));
    dump_tensor_attr(&output_attrs[i]);
  }

  // initialise inputs
  memset(rknn_inputs, 0, io_num.n_input * sizeof(rknn_input));
  for (uint32_t i = 0; i < io_num.n_input; i++) {
    rknn_inputs[i].index = i;
    rknn_inputs[i].fmt = input_attrs[i].fmt;

    // TODO: Standardize it
    if (runtime == 1) {
      rknn_inputs[i].pass_through = 0;
      rknn_inputs[i].type = RKNN_TENSOR_FLOAT32;
      rknn_inputs[i].size = input_attrs[i].size * 2;
    }
    else {
      rknn_inputs[i].pass_through = 1;
      rknn_inputs[i].type = RKNN_TENSOR_FLOAT16;
      rknn_inputs[i].size = input_attrs[i].size;
    }

    rknn_inputs[i].buf = NULL;
  }

  // initialises output
  memset(rknn_outputs, 0, io_num.n_output * sizeof(rknn_output));
  for (uint32_t i = 0; i < io_num.n_output; ++i) {
    rknn_outputs[i].want_float  = 1;    // Convert to FP32
    rknn_outputs[i].index       = i;
    rknn_outputs[i].is_prealloc = 0;
  }
}

void RKNNModel::addInput(const std::string name, float *buffer, int size) {
  inputs.push_back(std::unique_ptr<RKNNModelInput>(new RKNNModelInput(name, buffer, size, nullptr)));
}

void RKNNModel::execute() {
  for (uint32_t i = 0; i < io_num.n_input; i++) {
    rknn_inputs[i].buf = (unsigned char *) inputs[i]->buffer;
  }

  RKNN_CHECK(rknn_inputs_set(ctx, io_num.n_input, rknn_inputs));
  RKNN_CHECK(rknn_run(ctx, NULL));
  RKNN_CHECK(rknn_outputs_get(ctx, io_num.n_output, rknn_outputs, NULL));

  // print out total model execution time (including rknn api's pre and post processing time)
  RKNN_CHECK(rknn_query(ctx, RKNN_QUERY_PERF_RUN, &(perf_run), sizeof(rknn_perf_run)));
  //print_execution_time(&perf_run);

  // Model only has one output
  assert(io_num.n_output == 1);
  memcpy(output, (float *) rknn_outputs[0].buf, rknn_outputs[0].size);
  output_size = rknn_outputs[0].size;
}
