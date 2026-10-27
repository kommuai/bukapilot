import math
from multiprocessing import Queue

from metadrive.component.sensors.base_camera import _cuda_enable
from metadrive.component.map.pg_map import MapGenerateMethod

from openpilot.tools.sim.bridge.common import SimulatorBridge
from openpilot.tools.sim.bridge.metadrive.metadrive_common import RGBCameraRoad, RGBCameraWide
from openpilot.tools.sim.bridge.metadrive.metadrive_world import MetaDriveWorld
from openpilot.tools.sim.lib.camerad import W, H


def straight_block(length):
  return {
    "id": "S",
    "pre_block_socket_index": 0,
    "length": length
  }

def curve_block(length, angle=45, direction=0, radius=None):
  return {
    "id": "C",
    "pre_block_socket_index": 0,
    "length": length,
    "radius": length if radius is None else radius,
    "angle": angle,
    "dir": direction
  }

def create_map(track_size=60, curve_radius=None):
  # The original 120 m bends exceed the curvature envelope represented by the
  # driving model at the simulator's normal speed. Keep the tight route
  # selectable for stress tests, but use a broader default for normal runs.
  curve_radius = track_size * 5 if curve_radius is None else curve_radius
  curve_len = track_size * 2
  return dict(
    type=MapGenerateMethod.PG_MAP_FILE,
    lane_num=2,
    lane_width=4.5,
    config=[
      None,
      straight_block(track_size),
      curve_block(curve_len, 90, radius=curve_radius),
      straight_block(track_size),
      curve_block(curve_len, 90, radius=curve_radius),
      straight_block(track_size),
      curve_block(curve_len, 90, radius=curve_radius),
      straight_block(track_size),
      curve_block(curve_len, 90, radius=curve_radius),
    ]
  )


class MetaDriveBridge(SimulatorBridge):
  TICKS_PER_FRAME = 5

  def __init__(self, dual_camera, high_quality, test_duration=math.inf, test_run=False, render=False):
    super().__init__(dual_camera, high_quality)

    self.should_render = render
    self.test_run = test_run
    self.test_duration = test_duration if self.test_run else math.inf

  def spawn_world(self, queue: Queue):
    sensors = {
      "rgb_road": (RGBCameraRoad, W, H, )
    }

    if self.dual_camera:
      sensors["rgb_wide"] = (RGBCameraWide, W, H)

    config = dict(
      use_render=self.should_render,
      vehicle_config=dict(
        enable_reverse=False,
        render_vehicle=False,
        image_source="rgb_road",
      ),
      sensors=sensors,
      image_on_cuda=_cuda_enable,
      image_observation=True,
      interface_panel=[],
      out_of_route_done=False,
      on_continuous_line_done=False,
      crash_vehicle_done=False,
      crash_object_done=False,
      arrive_dest_done=False,
      # Scale the Honda Civic ratio for MetaDrive's default vehicle
      # wheelbase (2.46894 m versus the Civic's 2.7 m).
      steer_ratio=16.82,
      traffic_density=0.0, # traffic is incredibly expensive
      map_config=create_map(),
      decision_repeat=1,
      physics_world_step_size=self.TICKS_PER_FRAME/100,
      preload_models=False,
      show_logo=False,
      anisotropic_filtering=False
    )

    return MetaDriveWorld(queue, config, self.test_duration, self.test_run, self.dual_camera)
