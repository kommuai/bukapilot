from cereal import car
from selfdrive.car import make_can_msg, apply_std_steer_torque_limits
from selfdrive.car.perodua.peroduacan import create_steer_command
from opendbc.can.packer import CANPacker
from selfdrive.car.perodua.values import DBC
import cereal.messaging as messaging

# livetuner import
from selfdrive.livetune_conf import livetune_conf
livetune = livetune_conf()

class CarControllerParams():
  def __init__(self):

    self.STEER_MAX = int(livetune.conf['steerMax'])                          # dac write value
    self.STEER_STEP = 1                                                      # how often we update the steer cmd
    self.STEER_DELTA_UP = int(livetune.conf['steerDeltaUp'])                 # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = int(livetune.conf['steerDeltaDown'])             # torque decrease per refresh
    self.STEER_DRIVER_ALLOWANCE = int(livetune.conf['steerDriverAllowance']) # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = int(livetune.conf['steerDriverMult'])     # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1                                             # from dbc

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.params = CarControllerParams()
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):

    can_sends = []

    # generate steering command
    if (frame % self.params.STEER_STEP) == 0 and enabled:
      new_steer = int(round(actuators.steer * self.params.STEER_MAX))                # actuator.steer is from range from -1.0 - 1.0
      apply_steer = apply_std_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)
      self.steer_rate_limited = ( new_steer != apply_steer ) and (apply_steer != 0)
    else:
      apply_steer = 0

    apply_steer = apply_steer*10

    # limit steering based on angle
    if (abs(CS.out.steeringAngle) > 220):
      apply_steer = 0

    # lower the fighting torque during manual steer
    if (CS.out.steeringPressed):
      apply_steer = apply_steer / 100
      print("Steering pressed")

    self.last_steer = apply_steer
    if apply_steer >= 0:
      self.steering_direction = True
    else:
      self.steering_direction = False

    can_sends.append(create_steer_command(self.packer, apply_steer, self.steering_direction, enabled, frame))

    # fake gps
#    if (frame % 2 == 0) :
#      pm = messaging.PubMaster(['gpsLocationExternal'])
#      dat = messaging.new_message('gpsLocationExternal')
#      pm.send('gpsLocationExternal', dat)

    return can_sends
