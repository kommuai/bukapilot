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
    del candidate, fingerprint, car_fw, alpha_long, is_release, docs
    ret.brand = "chery"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.chery)]
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerLimitTimer = 0.6
    ret.steerActuatorDelay = 0.01
    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.9871
    ret.wheelSpeedFactor = 0.832
    ret.openpilotLongitudinalControl = False
    ret.radarUnavailable = True
    ret.enableBsm = True
    return ret
