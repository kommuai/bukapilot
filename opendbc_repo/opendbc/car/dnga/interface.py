from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.dnga.carcontroller import CarController
from opendbc.car.dnga.values import CarControllerParams
from opendbc.car.dnga.carstate import CarState
from opendbc.car.dnga.values import CAR

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    ret.brand = "dnga"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.dnga)]
    ret.safetyConfigs[0].safetyParam = 1

    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.0], [255]]
    ret.lateralTuning.init("pid")
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.0, 20.0], [0.0, 20.0]]
    ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.04, 0.08], [0.10, 0.12]]

    ret.steerLimitTimer = 0.01  # DNGA EPS torque authority is low.
    ret.steerSaturationThreshold = 0.25  # warn before hitting full normalized torque (PID still clips at ±1)

    ret.steerActuatorDelay = 0.48
    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.7933

    ret.openpilotLongitudinalControl = True
    ret.longitudinalActuatorDelay = 0.6
    ret.longitudinalTuning.kiBP = [0., 20]
    ret.longitudinalTuning.kiV = [0.05, 0.02]
    ret.longitudinalTuning.kpV = [0.4]

    wheel_speed_factor = 1.0

    if candidate == CAR.PERODUA_ALZA:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.12, 0.14], [0.18, 0.25]]
      ret.lateralTuning.pid.kf = 0.00018
      wheel_speed_factor = 1.425
    elif candidate == CAR.PERODUA_ATIVA:
      ret.lateralTuning.pid.kf = 0.00018
      wheel_speed_factor = 1.505
    elif candidate == CAR.PERODUA_MYVI:
      ret.lateralTuning.pid.kf = 0.00012
      wheel_speed_factor = 1.31
    elif candidate == CAR.TOYOTA_VIOS:
      ret.lateralTuning.pid.kf = 0.00018
      wheel_speed_factor = 1.43
    else:
      ret.dashcamOnly = True

    ret.wheelSpeedFactor = wheel_speed_factor
    ret.minEnableSpeed = -1
    ret.enableBsm = True

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    events = self.create_common_events(ret)
    ret.events = events.to_msg()
    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
