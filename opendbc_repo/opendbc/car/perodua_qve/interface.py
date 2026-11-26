from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.perodua_qve.carcontroller import CarController
from opendbc.car.perodua_qve.carstate import CarState
from opendbc.car.perodua_qve.radar_interface import RadarInterface
from opendbc.car.perodua_qve.values import PeroduaQveSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    del fingerprint, car_fw, alpha_long, is_release, docs, candidate

    ret.brand = "perodua_qve"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.peroduaQve)]
    ret.safetyConfigs[0].safetyParam = int(PeroduaQveSafetyFlags.IGNORE_IGNITION_LINE)
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerLimitTimer = 0.6
    ret.steerActuatorDelay = 0.01

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.9871
    ret.openpilotLongitudinalControl = False
    ret.radarUnavailable = True
    ret.wheelSpeedFactor = 1.0
    ret.enableBsm = False
    ret.minEnableSpeed = -1

    return ret
