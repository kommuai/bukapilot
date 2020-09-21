
from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.perodua.peroduacan import create_steer_command
from opendbc.can.packer import CANPacker
from selfdrive.car.perodua.values import DBC

class CarControllerParams():
  def __init__(self):
    self.STEER_MAX = 32               # 
    self.STEER_STEP = 1               # how often we update the steer cmd
    self.STEER_DELTA_UP = 1           # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 3         # torque decrease per refresh
    self.STEER_DRIVER_ALLOWANCE = 2   # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 1  # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1      # from dbc

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.params = CarControllerParams()
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):

    can_sends = []

    # generate steering command
    if (frame % self.params.STEER_STEP) == 0 and enabled:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))                # actuator.steer is from range from -1.0 - 1.0
      apply_steer = apply_perodua_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, SteerLimitParams)
      self.steer_rate_limited = new_steer != apply_steer
    else:
      apply_steer = 0

    # limit steering
    if (abs(CS.out.steeringAngle) > 180):
      apply_steer = 0

    is_enable = (not CS.out.steeringPressed) and enabled

    self.last_steer = apply_steer
    can_sends.append(create_steer_command(self.packer, apply_steer, is_enable, frame))

    return can_sends
