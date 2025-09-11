#!/usr/bin/env python3
from cereal import car
from openpilot.selfdrive.car import STD_CARGO_KG, get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.proton.values import CAR
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN

from common.params import Params

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "proton"

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.proton)]
    ret.safetyConfigs[0].safetyParam = 1   # TODO: add actual safetyParam

    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerActuatorDelay = 0.30          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')

    ret.lateralTuning.pid.kpBP = [0., 25., 35., 40.]
    ret.lateralTuning.pid.kpV = [0.05, 0.15, 0.15, 0.16]
    ret.lateralTuning.pid.kiBP = [0., 20., 30.]
    ret.lateralTuning.pid.kiV = [0.10, 0.20, 0.40]
    ret.lateralTuning.pid.kf = 0.00007

    ret.longitudinalTuning.kpBP = [0., 5., 20.]
    ret.longitudinalTuning.kpV = [1.0, 0.8, 0.6]
    ret.longitudinalActuatorDelayLowerBound = 0.4
    ret.longitudinalActuatorDelayUpperBound = 0.5
    ret.longitudinalTuning.kiBP = [0., 5., 20.]
    ret.longitudinalTuning.kiV = [0.2, 0.2, 0.2]

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.7933

    ret.openpilotLongitudinalControl = True
    ret.wheelSpeedFactor = 1.02

    if candidate == CAR.X50:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [545]]
      ret.longitudinalTuning.kpV = [0.5, 0.4, 0.3]
      ret.longitudinalTuning.kiV = [0.05, 0.05, 0.05]
    elif candidate == CAR.S70:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [530]]
    elif candidate == CAR.X70:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [500]]
    elif candidate == CAR.X90:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [545]]
    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput


    ret.stopAccel = -0.8
    ret.vEgoStarting = 3.0
    ret.stoppingControl = True

    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.3 # reach stopping target smoothly

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    # events
    events = self.create_common_events(ret)

    ret.events = events.to_msg()
    return ret

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
