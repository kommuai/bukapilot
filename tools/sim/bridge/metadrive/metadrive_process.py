import math
import time
import numpy as np

from collections import namedtuple
from panda3d.core import Vec3
from multiprocessing.connection import Connection

from metadrive.engine.core.engine_core import EngineCore
from metadrive.engine.core.image_buffer import ImageBuffer
from metadrive.envs.metadrive_env import MetaDriveEnv
from metadrive.obs.image_obs import ImageObservation

from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0,0)


metadrive_simulation_state = namedtuple("metadrive_simulation_state", ["running", "done", "done_info"])
metadrive_vehicle_state = namedtuple(
  "metadrive_vehicle_state",
  ["velocity", "position", "bearing", "steering_angle", "acceleration", "gyroscope"],
)


def metadrive_action_from_wheel_angle(steering_angle, max_steering, steer_ratio):
  """Convert OpenPilot's steering-wheel angle to MetaDrive action units."""
  return np.clip(steering_angle / (max_steering * steer_ratio), -1, 1)


def wheel_angle_from_metadrive_action(action, max_steering, steer_ratio):
  """Convert MetaDrive's normalized tire-angle action to wheel-angle feedback."""
  return action * max_steering * steer_ratio

def apply_metadrive_patches(arrive_dest_done=True):
  # By default, metadrive won't try to use cuda images unless it's used as a sensor for vehicles, so patch that in
  def add_image_sensor_patched(self, name: str, cls, args):
    if self.global_config["image_on_cuda"]:# and name == self.global_config["vehicle_config"]["image_source"]:
        sensor = cls(*args, self, cuda=True)
    else:
        sensor = cls(*args, self, cuda=False)
    assert isinstance(sensor, ImageBuffer), "This API is for adding image sensor"
    self.sensors[name] = sensor

  EngineCore.add_image_sensor = add_image_sensor_patched

  # we aren't going to use the built-in observation stack, so disable it to save time
  def observe_patched(self, *args, **kwargs):
    return self.state

  ImageObservation.observe = observe_patched

  # disable destination, we want to loop forever
  def arrive_destination_patch(self, *args, **kwargs):
    return False

  if not arrive_dest_done:
    MetaDriveEnv._is_arrive_destination = arrive_destination_patch

def metadrive_process(dual_camera: bool, config: dict, camera_array, wide_camera_array, image_lock,
                      controls_recv: Connection, simulation_state_send: Connection, vehicle_state_send: Connection,
                      exit_event, op_engaged, test_duration, test_run):
  arrive_dest_done = config.pop("arrive_dest_done", True)
  steer_ratio = config.pop("steer_ratio", 16.82)
  apply_metadrive_patches(arrive_dest_done)

  road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))
  if dual_camera:
    assert wide_camera_array is not None
    wide_road_image = np.frombuffer(wide_camera_array.get_obj(), dtype=np.uint8).reshape((H, W, 3))

  env = MetaDriveEnv(config)

  def is_on_lane(vehicle):
    _, _, on_lane = vehicle.navigation._get_current_lane(vehicle)
    return on_lane

  def reset():
    env.reset()
    env.vehicle.config["max_speed_km_h"] = 1000

    simulation_state = metadrive_simulation_state(
      running=True,
      done=False,
      done_info=None,
    )
    simulation_state_send.send(simulation_state)

  reset()
  start_time = None

  def get_cam_as_rgb(cam):
    cam = env.engine.sensors[cam]
    cam.get_cam().reparentTo(env.vehicle.origin)
    cam.get_cam().setPos(C3_POSITION)
    cam.get_cam().setHpr(C3_HPR)
    img = cam.perceive(to_float=False)
    if not isinstance(img, np.ndarray):
      img = img.get() # convert cupy array to numpy
    return img

  rk = Ratekeeper(100, None)

  # MetaDrive's action is a normalized tire/front-wheel angle. The simulated
  # Honda exposes a steering-wheel angle to OpenPilot, so both directions must
  # use the same effective steering ratio.
  max_steering = getattr(env.vehicle, "max_steering", env.vehicle.MAX_STEERING)
  vc = [0,0]
  physics_dt = config.get("physics_world_step_size", 0.05)
  previous_step_velocity = np.asarray(env.vehicle.velocity, dtype=float)
  acceleration_forward = acceleration_left = 0.0

  while not exit_event.is_set():
    velocity = vec3(x=float(env.vehicle.velocity[0]), y=float(env.vehicle.velocity[1]), z=0)
    heading = float(env.vehicle.heading_theta)
    # Read angular velocity from Bullet instead of differentiating heading at
    # the 100 Hz bridge rate. MetaDrive advances physics every 50 ms, so a
    # wall-clock derivative at this loop rate otherwise becomes four zero
    # samples followed by one large spike.
    yaw_rate = float(env.vehicle._body.getAngularVelocity()[2])

    vehicle_state = metadrive_vehicle_state(
      velocity=velocity,
      position=env.vehicle.position,
      bearing=float(math.degrees(heading)),
      steering_angle=wheel_angle_from_metadrive_action(
        env.vehicle.steering, max_steering, steer_ratio
      ),
      # locationd maps raw sensor [x, y, z] to device [x, y, z] as
      # [-z, -y, -x]. MetaDrive uses +y-left/+z-up, while the device frame
      # uses +y-right/+z-down.
      acceleration=vec3(x=0, y=acceleration_left, z=-acceleration_forward),
      gyroscope=vec3(x=yaw_rate, y=0, z=0),
    )
    vehicle_state_send.send(vehicle_state)

    if controls_recv.poll(0):
      while controls_recv.poll(0):
        steer_angle, gas, should_reset = controls_recv.recv()

      steer_metadrive = metadrive_action_from_wheel_angle(
        steer_angle, max_steering, steer_ratio
      )

      vc = [steer_metadrive, gas]

      if should_reset:
        reset()
        start_time = None
        previous_step_velocity = np.asarray(env.vehicle.velocity, dtype=float)
        acceleration_forward = acceleration_left = 0.0

    is_engaged = op_engaged.is_set()
    if is_engaged and start_time is None:
      start_time = time.monotonic()

    if rk.frame % 5 == 0:
      _, _, terminated, _, _ = env.step(vc)
      stepped_velocity = np.asarray(env.vehicle.velocity, dtype=float)
      velocity_delta = (stepped_velocity - previous_step_velocity) / physics_dt
      stepped_heading = float(env.vehicle.heading_theta)
      forward = np.array([math.cos(stepped_heading), math.sin(stepped_heading)])
      left = np.array([-math.sin(stepped_heading), math.cos(stepped_heading)])
      acceleration_forward = float(np.dot(velocity_delta, forward))
      acceleration_left = float(np.dot(velocity_delta, left))
      previous_step_velocity = stepped_velocity
      timeout = True if start_time is not None and time.monotonic() - start_time >= test_duration else False
      out_of_lane = not is_on_lane(env.vehicle)

      if terminated or ((out_of_lane or timeout) and test_run):
        if terminated:
          done_result = env.done_function("default_agent")
        elif out_of_lane:
          done_result = (True, {"out_of_lane" : True})
        elif timeout:
          done_result = (True, {"timeout" : True})

        simulation_state = metadrive_simulation_state(
          running=False,
          done=done_result[0],
          done_info=done_result[1],
        )
        simulation_state_send.send(simulation_state)

      if dual_camera:
        wide_road_image[...] = get_cam_as_rgb("rgb_wide")
      road_image[...] = get_cam_as_rgb("rgb_road")
      image_lock.release()

    rk.keep_time()
