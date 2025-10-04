#!/usr/bin/env python3
from cereal import car
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.dnga.values import CAR

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "dnga"

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.dnga)]
    ret.safetyConfigs[0].safetyParam = 1

    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerActuatorDelay = 0.48          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.7933       # arbitruary number, don't touch

    ret.openpilotLongitudinalControl = True
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [255]]
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.longitudinalTuning.kpBP = [0., 5., 20.]
    ret.longitudinalTuning.kiBP = [5, 7, 28]
    ret.longitudinalActuatorDelayLowerBound = 0.40
    ret.longitudinalActuatorDelayUpperBound = 0.50

    ret.longitudinalTuning.deadzoneBP = [0., 9.]
    ret.longitudinalTuning.deadzoneV = [0., .20]

    if candidate == CAR.ALZA:
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 20], [0., 20]]
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.14, 0.18], [0.18, 0.25]]
      ret.lateralTuning.pid.kf = 0.00015
      ret.longitudinalTuning.kpV = [0.15, 0.5, 0.6]
      ret.longitudinalTuning.kiV = [0.10, 0.15, 0.20]
      ret.wheelSpeedFactor = 1.425

    elif candidate == CAR.ATIVA:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.12], [0.22]]
      ret.lateralTuning.pid.kf = 0.000188
      ret.longitudinalTuning.kpV = [0.6, 0.5, 0.05]
      ret.longitudinalTuning.kiV = [0.15, 0.14, 0.01]
      ret.wheelSpeedFactor = 1.505

    elif candidate == CAR.MYVI:
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 20], [0., 20]]
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.09, 0.12], [0.10, 0.16]]
      ret.lateralTuning.pid.kf = 0.00012
      ret.longitudinalTuning.kpV = [1.2, 1.1, 1.0]
      ret.longitudinalTuning.kiBP = [0.0]
      ret.longitudinalTuning.kiV = [0.0]
      ret.wheelSpeedFactor = 1.31

    elif candidate == CAR.VIOS:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.16], [0.30]]
      ret.lateralTuning.pid.kf = 0.00018
      ret.longitudinalTuning.kpV = [0.65, 0.6, 0.6]
      ret.longitudinalTuning.kiV = [0.12, 0.12, 0.12]
      ret.wheelSpeedFactor = 1.43

    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    ret.minEnableSpeed = -1
    ret.stoppingControl = True
    ret.stopAccel = -1.0
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.10 # reach stopping target smoothly

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
