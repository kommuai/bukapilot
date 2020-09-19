
from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.perodua.peroduacan import create_steer_command, create_ui_command
from opendbc.can.packer import CANPacker

class CarControllerParams():
  def __init__(self):
    self.STEER_MAX = 2047              # need to find this out, the max allowable steer analog out
    self.STEER_STEP = 2                # how often we update the steer cmd

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.params = CarControllerParams()
    self.packer = CANPacker(dbc_name)

  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):

    can_sends = []

    # generate steering command
    if (frame % self.params.STEER_STEP) == 0 and enabled:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))                # range from -1.0 - 1.0
      apply_steer = apply_perodua_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, SteerLimitParams)
      self.steer_rate_limited = new_steer != apply_steer
    else:
      apply_steer = 0

    self.apply_last_steer = apply_steer
    can_sends.append(packer, steering_command)

    return can_sends
