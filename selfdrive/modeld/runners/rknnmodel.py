#!/usr/bin/env python3

import numpy as np

from openpilot.selfdrive.modeld.runners.runmodel_pyx import RunModel  #(TODO add 1)
from openpilot.selfdrive.modeld.constants import ModelConstants
from rknnlite.api import RKNNLite

class RKNNModel(RunModel):
  def __init__(self, path, output, runtime, use_tf8, cl_context): # runtime and cl_context are not used in this case
    self.inputs = {}
    self.output = output
    self.use_tf8 = use_tf8
    self.input_names = ['input_imgs','big_input_imgs','desire','traffic_convention','lateral_control_params','prev_desired_curv','nav_features','nav_instructions','features_buffer']  # noqa: E501

    self.input_shapes = {
    'input_imgs': [1, 12, 128, 256],
    'big_input_imgs': [1, 12, 128, 256],
    'desire': [1, 100, 8],
    'traffic_convention': [1, 2],
    'lateral_control_params': [1, 2],
    'prev_desired_curv': [1, 100, 1],
    'nav_features': [1, 256],
    'nav_instructions': [1, 150],
    'features_buffer': [1, 99, 512]
    }

    self.input_dtypes = {
    'input_imgs': np.float16,
    'big_input_imgs': np.float16,
    'desire': np.float16,
    'traffic_convention': np.float16,
    'lateral_control_params': np.float16,
    'prev_desired_curv': np.float16,
    'nav_features': np.float16,
    'nav_instructions': np.float16,
    'features_buffer': np.float16
    }

    self.rknn = RKNNLite(verbose=False)
    self.rknn.load_rknn(path)
    self.rknn.init_runtime()

  def addInput(self, name, buffer):
    assert name in self.input_names
    self.inputs[name] = buffer

  def setInputBuffer(self, name, buffer):
    assert name in self.inputs
    self.inputs[name] = buffer

  def getCLBuffer(self, name):
    return None

  def execute(self):
    # input shaping and formatting
    inputs = {k: (v.view(np.uint8) / 255. if self.use_tf8 and k == 'input_img' else v) for k,v in self.inputs.items()}
    inputs = {k: v.reshape(self.input_shapes[k]).astype(self.input_dtypes[k]) for k,v in inputs.items()}
    inputs = [inputs[input_name] for input_name in self.input_names]

    # running inputs through model
    outputs = self.rknn.inference(inputs=inputs, data_type="float16", inputs_pass_through=[0,0,0,0,0,0,0,0,0], \
                                  data_format=['nchw', 'nchw', 'nchw', 'nchw', 'nchw', 'nchw', 'nchw', 'nchw', 'nchw'])
    # check that the output is valid
    assert len(outputs) == 1, "Only single model outputs are supported"
    self.output[:] = outputs[0]
    return self.output

if __name__ == '__main__':
  OUTPUT_SIZE = 6504

  # initialise input and output size, and start NPU chip
  output = np.zeros(OUTPUT_SIZE, dtype=np.float32)
  inputs = {
      'desire': np.zeros(ModelConstants.DESIRE_LEN * (ModelConstants.HISTORY_BUFFER_LEN+1), dtype=np.float32),
      'traffic_convention': np.zeros(ModelConstants.TRAFFIC_CONVENTION_LEN, dtype=np.float32),
      'lateral_control_params': np.zeros(ModelConstants.LATERAL_CONTROL_PARAMS_LEN, dtype=np.float32),
      'prev_desired_curv': np.zeros(ModelConstants.PREV_DESIRED_CURV_LEN * (ModelConstants.HISTORY_BUFFER_LEN+1), dtype=np.float32),
      'nav_features': np.zeros(ModelConstants.NAV_FEATURE_LEN, dtype=np.float32),
      'nav_instructions': np.zeros(ModelConstants.NAV_INSTRUCTION_LEN, dtype=np.float32),
      'features_buffer': np.zeros(ModelConstants.HISTORY_BUFFER_LEN * ModelConstants.FEATURE_LEN, dtype=np.float32),
      }


  model = RKNNModel(path = '../models/supercombo.rknn', output = output, runtime = 0, use_tf8 = False, cl_context= None)

  # prefill all values with zeros
  model.addInput("input_imgs", np.zeros(393216, dtype=np.float32))
  model.addInput("big_input_imgs", np.zeros(393216, dtype=np.float32))

  for k,v in inputs.items():
      model.addInput(k, v)

  # run model with all zeros inputs

  for k,v in inputs.items():
      inputs[k][:] = v

  model.execute()
