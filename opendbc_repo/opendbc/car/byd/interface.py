from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.radar_interface import RadarInterface
from opendbc.car.byd.values import CAR

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    ret.brand = "byd"

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.byd)]
    ret.safetyConfigs[0].safetyParam = 1

    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerLimitTimer = 0.6
    ret.steerActuatorDelay = 0.01

    ret.lateralTuning.init("pid")

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.9871

    ret.openpilotLongitudinalControl = True
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.0], [530]]
    ret.lateralTuning.pid.kpBP = [0.0, 5.0, 20.0]
    ret.lateralTuning.pid.kiBP = [0.0, 5.0, 20.0]
    ret.lateralTuning.pid.kf = 0.00015
    ret.wheelSpeedFactor = 0.66

    if candidate == CAR.BYD_ATTO3:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.52, 0.43, 0.32], [1.5, 1.4, 1.1]]
    elif candidate == CAR.BYD_M6:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.52, 0.43, 0.32], [1.5, 1.4, 1.1]]
      ret.safetyConfigs[0].safetyParam = 3
    elif candidate in (CAR.BYD_SEAL, CAR.BYD_SEALION7):
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.52, 0.43, 0.32], [1.5, 1.4, 1.1]]
      ret.safetyConfigs[0].safetyParam = 2
      ret.openpilotLongitudinalControl = False
      ret.radarUnavailable = True
    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    ret.startingState = True
    ret.startAccel = 3.0
    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.2
    ret.longitudinalActuatorDelay = 0.3

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    events = self.create_common_events(ret)
    ret.events = events.to_msg()
    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
