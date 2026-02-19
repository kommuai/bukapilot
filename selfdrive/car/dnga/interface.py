#!/usr/bin/env python3
from cereal import car
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.dnga.values import CAR
from openpilot.selfdrive.car.dnga.carcontroller import CarControllerParams

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "dnga"

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.dnga)]
    ret.safetyConfigs[0].safetyParam = 1

    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerLimitTimer = 0.01             # Immediate warning since DNGA has low torque
    ret.steerActuatorDelay = 0.48          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.7933       # arbitruary number, don't touch

    ret.openpilotLongitudinalControl = True
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [255]]
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.longitudinalTuning.kpBP = [0., 5., 20.]
    # perodua uses speed for CAN control, internally the car already has a
    # controller on its own. If integral is enabled it will always windup,
    # causing brake to not magnitude when needed.
    ret.longitudinalTuning.kiBP = [0.0]
    ret.longitudinalTuning.kiV = [0.0]
    ret.longitudinalActuatorDelayLowerBound = 0.40
    ret.longitudinalActuatorDelayUpperBound = 0.50
    ret.longitudinalTuning.deadzoneBP = [0., 9.]
    ret.longitudinalTuning.deadzoneV = [0., .20]
    ret.longitudinalTuning.kpV = [2.2, 2.0, 1.8]

    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 20], [0., 20]]
    # if integral too low it will take too long to saturate and steerLimit won't sound
    ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.08, 0.12], [0.10, 0.14]]

    if candidate == CAR.ALZA:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.14, 0.18], [0.18, 0.25]]
      ret.lateralTuning.pid.kf = 0.00015
      ret.longitudinalTuning.kpV = [0.1, 1.2, 1.2]
      ret.wheelSpeedFactor = 1.425

    elif candidate == CAR.ATIVA:
      ret.lateralTuning.pid.kf = 0.000188
      ret.wheelSpeedFactor = 1.505

    elif candidate == CAR.MYVI:
      ret.lateralTuning.pid.kf = 0.00012
      ret.wheelSpeedFactor = 1.31

    elif candidate == CAR.VIOS:
      ret.lateralTuning.pid.kf = 0.00018
      ret.wheelSpeedFactor = 1.43

    elif candidate == CAR.QC:
      ret.lateralTuning.pid.kf = 0.00018
      ret.wheelSpeedFactor = 1.43
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.noOutput)]

    else:
      ret.dashcamOnly = True

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
