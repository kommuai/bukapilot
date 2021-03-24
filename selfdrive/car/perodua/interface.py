#!/usr/bin/env python3
from cereal import car
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.perodua.values import CAR

class CarInterface(CarInterfaceBase):

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "perodua"
    ret.safetyModel = car.CarParams.SafetyModel.perodua
    #ret.safetyModel = car.CarParams.SafetyModel.allOutput

    # perodua port is a community feature
    ret.communityFeature = True

    ret.steerRateCost = 0.7 # Lateral MPC cost on steering rate, higher value = sharper turn
    ret.steerLimitTimer = 0.9 # time before steerLimitAlert is issued
    ret.steerControlType = car.CarParams.SteerControlType.torque # or car.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.4 # Steering wheel actuator delay in seconds, it was 0.1
    
    # Tire stiffness factor fictitiously lower if it includes the steering column torsion effect.
    # For modeling details, see p.198-200 in "The Science of Vehicle Dynamics (2014), M. Guiggiani"
    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.23], [0.1]]
    ret.lateralTuning.pid.kf = 0.0000112   # full torque for 20 deg at 80mph means 0.00007818594
    
    ret.gasMaxBP = [0., 9., 35]
    ret.gasMaxV = [0.2, 0.5, 0.7]
    ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
    ret.startAccel = 0.2 # Required acceleraton to overcome creep braking

    # adding support for Perodua Axia 2019
    if candidate == CAR.PERODUA_AXIA:
      stop_and_go = False
      # force openpilot to fake the stock camera, make it True when we want can to spoof adas cam
      ret.enableCamera = True
 
      # force openpilot to inject gas command through gas interceptor
      ret.enableGasInterceptor = 0x201 in fingerprint[0]
      # since using gas interceptor means there is no cruise control
      # Make it False so OP calculates the set speed logic, see openpilot/selfdrive/controls/controlsd.py#L277
      ret.enableCruise = not ret.enableGasInterceptor
      ret.enableDsu = not ret.enableGasInterceptor
      ret.enableApgs = False
      
      # NEED TO FIND OUT
      ret.safetyParam = 1                           # see conversion factor for STEER_TORQUE_EPS in dbc file
      ret.wheelbase = 2.455
      ret.steerRatio = 16.54                        # 360:degree change, it was 18.94
      ret.centerToFront = ret.wheelbase * 0.44      # wild guess
      tire_stiffness_factor = 0.6371                # Need to handtune
      ret.mass = 1870. * CV.LB_TO_KG + STD_CARGO_KG # curb weight is given in pounds,lb
      ret.openpilotLongitudinalControl = True
      ret.transmissionType = car.CarParams.TransmissionType.automatic

    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput
    
    cloudlog.warning("Gas Interceptor: %r", ret.enableGasInterceptor)
    cloudlog.warning("Camera Simulated: %r", ret.enableCamera)

    # min speed to enable ACC. if car can do stop and go or has gas interceptor,
    # then set enabling speed to a negative value, so it won't matter.
    ret.minEnableSpeed = -1. if (stop_and_go or ret.enableGasInterceptor) else 25.5 * CV.MPH_TO_MS

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # to receive CAN Messages
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)
    ret.yawRate = self.VM.yaw_rate(ret.steeringAngle * CV.DEG_TO_RAD, ret.vEgo)
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # events
    events = self.create_common_events(ret)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c):

    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                               c.hudControl.visualAlert, c.cruiseControl.cancel)

    self.frame += 1
    return can_sends
