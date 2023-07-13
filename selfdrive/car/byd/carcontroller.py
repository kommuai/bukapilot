from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.byd.bydcan import create_can_steer_command, create_accel_command, send_buttons
from selfdrive.car.byd.values import CAR, DBC
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip

import cereal.messaging as messaging

def apply_byd_steer_angle_limits(apply_angle, actual_angle, v_ego, LIMITS):
  # pick angle rate limits based on wind up/down
  steer_up = actual_angle * apply_angle >= 0. and abs(apply_angle) > abs(actual_angle)
  rate_limits = LIMITS.ANGLE_RATE_LIMIT_UP if steer_up else LIMITS.ANGLE_RATE_LIMIT_DOWN

  return clip(apply_angle, actual_angle - rate_limits, actual_angle + rate_limits)

class CarControllerParams():
  def __init__(self, CP):
    self.ANGLE_RATE_LIMIT_UP = 6       # maximum allow 150 degree per second, 100Hz loop means 1.5
    self.ANGLE_RATE_LIMIT_DOWN = 6

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.steer_rate_limited = False

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw):
    can_sends = []

    # steer
    apply_angle = apply_byd_steer_angle_limits(actuators.steeringAngleDeg, CS.out.steeringAngleDeg, CS.out.vEgo, self.params)
    #self.steer_rate_limited = ((actuators.steeringAngleDeg != apply_angle) and (apply_angle != 0)) or (abs(apply_angle - CS.out.steeringAngleDeg) > 2.5)
    self.steer_rate_limited = (abs(apply_angle - CS.out.steeringAngleDeg) > 2.5)

    # BYD CAN controlled lateral running at 50hz
    if (frame % 2) == 0:
      lat_active = (enabled and abs(CS.out.steeringAngleDeg) < 85) # temporary hardcode 85 degrees because if 90 degrees it will fault
      can_sends.append(create_can_steer_command(self.packer, apply_angle, lat_active, (frame/2) % 16))

#      if CS.out.genericToggle:
#        can_sends.append(send_buttons(self.packer, frame % 16))
      #if CS.out.leftBlinker:
      #  # accel
      #  can_sends.append(create_accel_command(self.packer, 30, 1, CS.out.standstill, (frame/2) % 16))
      #elif CS.out.rightBlinker and False:
      #  # decel
      #  can_sends.append(create_accel_command(self.packer, -30, 1, CS.out.standstill, (frame/2) % 16))
      #else:
      #  can_sends.append(create_accel_command(self.packer, 0, 0, CS.out.standstill, (frame/2) % 16))
      #  a = list(create_accel_command(self.packer, 0, 0, (frame/2) % 16)[2])
      #  b = [hex(i) for i in a]
      #  print(b)

#    if CS.out.standstill and enabled and (frame % 50 == 0):
      # Spam resume button to resume from standstill at max freq of 20 Hz.
 #     can_sends.append(send_buttons(self.packer, frame % 16))

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    return new_actuators, can_sends
