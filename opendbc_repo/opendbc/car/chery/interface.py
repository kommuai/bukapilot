from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.chery.carcontroller import CarController
from opendbc.car.chery.carstate import CarState
from opendbc.car.chery.radar_interface import RadarInterface
from opendbc.car.chery.values import (
  CAR,
  CHERY_ICAUR_SAFETY_PARAM,
  CHERY_OMODA_NO_TORQUE_SPOOF_PARAM,
  CHERY_OMODA_SAFETY_PARAM,
  ICAUR_DISABLE_TORQUE_SPOOF,
  OMODA_DISABLE_TORQUE_SPOOF,
  TIGGO_DISABLE_TORQUE_SPOOF,
  TIGGO_DISABLE_HUD_OVERRIDE,
  TIGGO_DISABLE_STOP_AND_GO,
  CHERY_NATIVE_HUD_FWD_PARAM,
  CHERY_TIGGO22_SAFETY_PARAM,
  Tiggo22SteerLimits,
)


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    del fingerprint, car_fw, alpha_long, is_release, docs
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
    if candidate == CAR.CHERY_OMODA_5:
      ret.safetyConfigs[0].safetyParam = CHERY_OMODA_SAFETY_PARAM
      if OMODA_DISABLE_TORQUE_SPOOF:
        ret.safetyConfigs[0].safetyParam |= CHERY_OMODA_NO_TORQUE_SPOOF_PARAM
    elif candidate == CAR.CHERY_TIGGO_8_PRO_2025:
      if TIGGO_DISABLE_TORQUE_SPOOF:
        ret.safetyConfigs[0].safetyParam |= CHERY_OMODA_NO_TORQUE_SPOOF_PARAM
      if TIGGO_DISABLE_HUD_OVERRIDE:
        ret.safetyConfigs[0].safetyParam |= CHERY_NATIVE_HUD_FWD_PARAM
      if TIGGO_DISABLE_STOP_AND_GO:
        ret.autoResumeSng = False
    elif candidate == CAR.CHERY_TIGGO_8_PRO_2022_2024:
      # 2022-24: HUD on bus 1, 0x220 EPS torque (PI), stock LKAS not Jaecoo 0x345.
      ret.steerControlType = car.CarParams.SteerControlType.torque
      ret.steerActuatorDelay = 0.10
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0], [Tiggo22SteerLimits.STEER_MAX]]
      ret.lateralTuning.init("pid")
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kpV = [[0.], [0.22]]
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kiV = [[0.], [0.02]]
      ret.lateralTuning.pid.kf = 0.00010
      if TIGGO_DISABLE_TORQUE_SPOOF:
        ret.safetyConfigs[0].safetyParam |= CHERY_OMODA_NO_TORQUE_SPOOF_PARAM
      if TIGGO_DISABLE_HUD_OVERRIDE:
        ret.safetyConfigs[0].safetyParam |= CHERY_NATIVE_HUD_FWD_PARAM
      ret.safetyConfigs[0].safetyParam |= CHERY_TIGGO22_SAFETY_PARAM
      ret.autoResumeSng = True
    elif candidate == CAR.CHERY_ICAUR_03:
      ret.safetyConfigs[0].safetyParam = CHERY_ICAUR_SAFETY_PARAM
      if ICAUR_DISABLE_TORQUE_SPOOF:
        ret.safetyConfigs[0].safetyParam |= CHERY_OMODA_NO_TORQUE_SPOOF_PARAM
      # GPS origin-fit on 2026-07-13--04-13-59: 13-bit DBC raw (scale 1) × ~0.01756 ≈ gpsLocation.speed
      # (supersedes 8-bit×0.55; 0.55/32=0.0171875 was a close prior guess).
      ret.wheelSpeedFactor = 0.01756
      # No BSM signals reverse-engineered yet.
      ret.enableBsm = False
    return ret
