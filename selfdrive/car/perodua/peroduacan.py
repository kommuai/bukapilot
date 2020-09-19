from common.numpy_fast import clip
from cereal import car

VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_steer_command(packer, command):
  """Creates a CAN message for the steering command."""

  values = {
    "Counter": idx,
    "LKAS_Output": apply_steer,
    "LKAS_Request": 1 if apply_steer != 0 else 0,
    "SET_1": 1
  }

  return packer.make_can_msg("ES_LKAS", 0, values)


def create_ui_command(packer, main_on, enabled, steer_alert):
  """Creates a CAN message for the Ford Steer Ui."""

  return None
