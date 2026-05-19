from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.chery.carcontroller import CarController
from opendbc.car.chery.carstate import CarState
from opendbc.car.chery.radar_interface import RadarInterface


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    del fingerprint, car_fw, alpha_long, is_release, docs, candidate
    ret.brand = "chery"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.chery)]
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerLimitTimer = 0.6
    ret.steerActuatorDelay = 0.01
    ret.lateralTuning.init("pid")
    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.9871
    ret.openpilotLongitudinalControl = False
    ret.radarUnavailable = True
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.0], [530]]
    ret.lateralTuning.pid.kpBP = ret.lateralTuning.pid.kiBP = [0.0, 5.0, 20.0]
    ret.lateralTuning.pid.kf = 0.00015
    ret.lateralTuning.pid.kiV = [0.52, 0.43, 0.32]
    ret.lateralTuning.pid.kpV = [1.5, 1.4, 1.1]
    ret.wheelSpeedFactor = 0.832
    ret.enableBsm = True
    return ret
