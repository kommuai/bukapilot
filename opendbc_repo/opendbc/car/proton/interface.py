#!/usr/bin/env python3
from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.proton.carcontroller import CarController
from opendbc.car.proton.carstate import CarState
from opendbc.car.proton.values import CAR

from openpilot.common.features import Features


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    ret.brand = "proton"

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.proton)]
    ret.safetyConfigs[0].safetyParam = 2 if Features().has("stock-acc") else 1

    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerLimitTimer = 0.1
    ret.steerActuatorDelay = 0.30

    ret.lateralTuning.init("pid")

    ret.lateralTuning.pid.kpBP = [0.0, 5.0, 25.0, 35.0, 40.0]
    ret.lateralTuning.pid.kpV = [0.05, 0.05, 0.15, 0.15, 0.16]
    ret.lateralTuning.pid.kiBP = [0.0, 5.0, 20.0, 30.0]
    ret.lateralTuning.pid.kiV = [0.05, 0.10, 0.20, 0.40]
    ret.lateralTuning.pid.kf = 0.00007

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.7933
    ret.longitudinalActuatorDelay = 0.6

    ret.openpilotLongitudinalControl = True
    ret.wheelSpeedFactor = 1.02

    if candidate == CAR.PROTON_X50:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.0], [545]]
    elif candidate == CAR.PROTON_S70:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.0], [530]]
    elif candidate == CAR.PROTON_X70:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.0], [500]]
    elif candidate == CAR.PROTON_X90:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.0], [256]]
      ret.lateralTuning.pid.kiV = [0.05, 0.05, 0.05, 0.05]
    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    ret.stopAccel = -0.8
    ret.startingState = True
    ret.startAccel = 1.2
    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.3

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    events = self.create_common_events(ret)
    ret.events = events.to_msg()
    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
