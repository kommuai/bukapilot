#!/usr/bin/env python3
from cereal import car
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.perodua.values import CAR, ACC_CAR
from selfdrive.controls.lib.lateral_planner import LANE_CHANGE_SPEED_MIN
from common.params import Params

EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "perodua"
    ret.safetyModel = car.CarParams.SafetyModel.perodua

    # perodua port is a community feature
    ret.communityFeature = True

    disableLong = Params().get("DisableBukapilotLongitudinal") == b'1'
    ret.radarOffCan = True

    ret.steerRateCost = 0.7                # Lateral MPC cost on steering rate, higher value = sharper turn
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerActuatorDelay = 0.48           # Steering wheel actuator delay in seconds

    # Tire stiffness factor fictitiously lower if it includes the steering column torsion effect.
    # For modeling details, see p.198-200 in "The Science of Vehicle Dynamics (2014), M. Guiggiani"
    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.gasMaxBP = [0., 9., 35]
    ret.gasMaxV = [0.4, 0.4, 0.55]
    ret.longitudinalTuning.kpV = [0.9, 0.8, 0.8]
    ret.startAccel = 1                     # Required acceleraton to overcome creep braking

    # common interfaces
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    ret.enableApgs = False                 # advanced parking guidance system

    if disableLong:
      ret.safetyParam = 2
    else:
      ret.safetyParam = 1

    ret.enableGasInterceptor = 0x201 in fingerprint[0] or 0x401 in fingerprint[0]
    ret.openpilotLongitudinalControl = True

    if candidate == CAR.AXIA:
      ret.wheelbase = 2.455                # meter
      ret.steerRatio = 16.54
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.8371
      ret.mass = 850. + STD_CARGO_KG

      ret.lateralTuning.pid.kf = 0.0000715
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.08], [0.32]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[11, 26], [480, 830]]

      ret.longitudinalTuning.kpV = [0.8, 0.9, 1.0]

    elif candidate == CAR.MYVI:
      ret.wheelbase = 2.5
      ret.steerRatio = 12.14
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.8371
      ret.mass = 1015. + STD_CARGO_KG

      ret.gasMaxV = [0.5, 0.5, 0.6]
      ret.lateralTuning.pid.kf = 0.0000917
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.10], [0.32]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[11  , 28], [390, 580]]

    elif candidate == CAR.BEZZA:
      ret.wheelbase = 2.455
      ret.steerRatio = 12.14
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.8371
      ret.mass = 940. + STD_CARGO_KG

      ret.lateralTuning.pid.kf = 0.0000918
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.05], [0.45]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[11, 22], [400, 630]]

    elif candidate == CAR.ARUZ:
      ret.wheelbase = 2.685
      ret.steerRatio = 16.14
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.68371
      ret.mass = 1310. + STD_CARGO_KG

      ret.lateralTuning.pid.kf = 0.0000917
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.098], [0.135]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[11, 22], [380, 600]]

      ret.longitudinalTuning.kpV = [1.6, 1.1, 1.1]

    elif candidate == CAR.MYVI_PSD:
      ret.wheelbase = 2.5
      ret.steerRatio = 17.24
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.9871
      ret.mass = 1025. + STD_CARGO_KG

      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.12], [0.20]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [255]]
      ret.lateralTuning.pid.kf = 0.0000007

      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [1.2, 1.2, 0.8]
      ret.gasMaxBP = [0.]
      ret.gasMaxV = [0.5]
    elif candidate == CAR.ATIVA:
      ret.wheelbase = 2.525
      ret.steerRatio = 16.74
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.9871
      ret.mass = 1035. + STD_CARGO_KG

      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.12], [0.20]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [255]]
      ret.lateralTuning.pid.kf = 0.0000007

      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [1.6, 1.6, 1.2]
      ret.gasMaxBP = [0.]
      ret.gasMaxV = [0.8]
    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    ret.enableDsu = False

    if candidate in ACC_CAR:
      ret.longitudinalTuning.deadzoneBP = [0., 8.05]
      ret.longitudinalTuning.deadzoneV = [.0, .14]
      ret.longitudinalTuning.kiBP = [0., 5., 20.]
      ret.longitudinalTuning.kiV = [.14, .10, .08]
      #ret.longitudinalTuning.kiV = [.32, .20, .17, .14, .07]
      ret.startAccel = 1.2  # Accelerate from 0 faster

      ret.minEnableSpeed = -1
      ret.steerActuatorDelay = 0.30           # Steering wheel actuator delay in seconds
      ret.enableBsm = True
      ret.stoppingBrakeRate = 0.005 # reach stopping target smoothly
      ret.startingBrakeRate = 0.2  # release brakes fast

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # to receive CAN Messages
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)
    ret.yawRate = self.VM.yaw_rate(ret.steeringAngleDeg * CV.DEG_TO_RAD, ret.vEgo)
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # events
    events = self.create_common_events(ret)

    # create events for auto lane change below allowable speed
    if ret.vEgo < LANE_CHANGE_SPEED_MIN and (ret.leftBlinker or ret.rightBlinker):
      events.add(EventName.belowLaneChangeSpeed)

    # events for non ACC cars
    if self.CP.carFingerprint not in ACC_CAR:
      # create events to warn user that their vehicle doesn't have brakes
      if self.CC.brake_pressed and (ret.cruiseState.speed >= ret.vEgo):
        events.add(EventName.promptDriverBrake)

    # events for ACC cars
    if self.CP.carFingerprint in ACC_CAR:
      # warning about the 3s only standstill brake
      if ret.standstill and self.CC.pump_saturated:
        events.add(EventName.promptDriverBrake)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c):

    isLdw = c.hudControl.leftLaneDepart or c.hudControl.rightLaneDepart

    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators, c.hudControl.leadVisible, c.hudControl.rightLaneVisible, c.hudControl.leftLaneVisible, c.cruiseControl.cancel, c.cruiseControl.speedOverride, isLdw)

    self.frame += 1
    return can_sends
