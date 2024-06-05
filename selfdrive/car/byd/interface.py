#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.byd.values import CAR, HUD_MULTIPLIER
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "byd"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.byd)]
    ret.safetyConfigs[0].safetyParam = 1
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    ret.radarOffCan = True
    ret.enableApgs = False                 # advanced parking guidance system
    ret.enableDsu = False                  # driving support unit

    ret.steerRateCost = 0.1               # Lateral MPC cost on steering rate, higher value = sharper turn
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.01          # Steering wheel actuator delay in seconds

    ret.enableGasInterceptor = False
    ret.openpilotLongitudinalControl = True

    if candidate == CAR.ATTO3:
      ret.wheelbase = 2.72
      ret.steerRatio = 16.0
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.9871
      ret.mass = 2090. + STD_CARGO_KG
      ret.wheelSpeedFactor = HUD_MULTIPLIER           # the HUD odo is exactly 1 to 1 with gps speed

      # currently not in use, byd is using stock long
      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [1.5, 1.3, 1.0]
      ret.longitudinalActuatorDelayLowerBound = 0.3
      ret.longitudinalActuatorDelayUpperBound = 0.4

    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    # currently not in use, byd is using stock long
    ret.longitudinalTuning.deadzoneBP = [0., 8.05, 20]
    ret.longitudinalTuning.deadzoneV = [0., 0., 0.]
    ret.longitudinalTuning.kiBP = [0., 5., 20.]
    ret.longitudinalTuning.kiV = [0.32, 0.23, 0.12]
    ret.longitudinalTuning.kiV = [0., 0., 0.]

    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.05 # reach stopping target smoothly

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # to receive CAN Messages
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # events
    events = self.create_common_events(ret)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c):

    isLdw = c.hudControl.leftLaneDepart or c.hudControl.rightLaneDepart

    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators, c.hudControl.leadVisible, c.hudControl.rightLaneVisible, c.hudControl.leftLaneVisible, c.cruiseControl.cancel, isLdw, c.laneActive)
    self.frame += 1
    return can_sends
