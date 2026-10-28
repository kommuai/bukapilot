"""Closed, at-grade figure-eight road network for the MetaDrive bridge."""

import math

import numpy as np
from panda3d.core import NodePath

from metadrive.component.lane.circular_lane import CircularLane
from metadrive.component.lane.straight_lane import StraightLane
from metadrive.component.map.pg_map import PGMap
from metadrive.component.pgblock.create_pg_block_utils import CreateAdverseRoad, CreateRoadFrom
from metadrive.component.pgblock.first_block import FirstPGBlock
from metadrive.component.pgblock.pg_block import PGBlock, PGBlockSocket
from metadrive.component.pg_space import ParameterSpace
from metadrive.component.road_network import Road
from metadrive.component.road_network.node_road_network import NodeRoadNetwork
from metadrive.constants import PGLineType
from metadrive.manager.pg_map_manager import PGMapManager


class FigureEightLoopBlock(PGBlock):
  """The additional roads that close the two lobes through one crossing."""

  ID = "8"
  SOCKET_NUM = 1
  PARAMETER_SPACE = ParameterSpace({})

  def __init__(self, block_index, pre_block_socket, global_network, random_seed, junction,
               track_size, curve_radius):
    self.junction = np.asarray(junction, dtype=float)
    self.track_size = float(track_size)
    self.curve_radius = float(curve_radius)
    super().__init__(block_index, pre_block_socket, global_network, random_seed,
                     ignore_intersection_checking=True)

  def construct_loop(self, root_render_np, physics_world):
    # Merge adjacency lists instead of NodeRoadNetwork.add(), whose normal
    # PGMap contract rejects or replaces the shared crossing node.
    self._block_objects = []
    if not isinstance(self.origin, NodePath):
      self.origin = NodePath(self.name)
    self._clear_topology()
    self._sample_topology()
    for start_node, outgoing in self.block_network.graph.items():
      for end_node, lanes in outgoing.items():
        self._global_network.add_road(Road(start_node, end_node), lanes)
    self._create_in_world()
    self.attach_to_world(root_render_np, physics_world)

  def _try_plug_into_previous_block(self):
    jx, jy = self.junction
    radius = self.curve_radius
    east = np.array([jx + radius, jy])
    north = np.array([jx, jy + radius])
    south = np.array([jx, jy - radius])
    west = np.array([jx - radius, jy])

    east_node, north_node = "8E", "8N"
    south_node, west_node = "8S", "8W"

    def add_road(road, lane):
      CreateRoadFrom(
        lane,
        self.positive_lane_num,
        road,
        self.block_network,
        self._global_network,
        ignore_intersection_checking=True,
        center_line_type=PGLineType.BROKEN,
        side_lane_line_type=PGLineType.SIDE,
      )
      CreateAdverseRoad(
        road,
        self.block_network,
        self._global_network,
        ignore_intersection_checking=True,
        center_line_type=PGLineType.BROKEN,
        side_lane_line_type=PGLineType.SIDE,
      )
      self.add_respawn_roads(road)

    def add_straight(start_node, end_node, start, end):
      lane = StraightLane(start, end, self.lane_width, (PGLineType.BROKEN, PGLineType.SIDE))
      add_road(Road(start_node, end_node), lane)

    def add_quarter_arcs(start_node, intermediate_nodes, end_node, center, start_phase, clockwise):
      nodes = [start_node, *intermediate_nodes, end_node]
      phase_step = -math.pi / 2 if clockwise else math.pi / 2
      for segment_start, segment_end in zip(nodes[:-1], nodes[1:]):
        add_road(
          Road(segment_start, segment_end),
          CircularLane(
            center=center,
            radius=radius,
            start_phase=start_phase,
            angle=math.pi / 2,
            clockwise=clockwise,
            width=self.lane_width,
            line_types=(PGLineType.BROKEN, PGLineType.SIDE),
          ),
        )
        start_phase += phase_step

    # Continue the existing eastbound FirstPGBlock arm to the outer tangent.
    east_arm_start = self.positive_basic_lane.position(self.positive_basic_lane.length, 0)
    add_straight(FirstPGBlock.NODE_3, east_node, east_arm_start, east)

    # Three quarter arcs preserve R while keeping each navigation segment
    # within MetaDrive's normal Curve block angle envelope.
    add_quarter_arcs(
      east_node,
      ("8E1", "8E2"),
      north_node,
      east + np.array([0, radius]),
      -math.pi / 2,
      clockwise=False,
    )

    # The center node is shared by north→south and west→east through roads.
    # These straight passages cross at grade, rather than touching tangentially.
    add_straight(north_node, FirstPGBlock.NODE_1, north, self.junction)
    add_straight(FirstPGBlock.NODE_1, south_node, self.junction, south)

    # The SW lobe mirrors the first. MetaDrive's normal lane ordering keeps
    # both clockwise arc lanes aligned with the adjoining straight approaches.
    add_quarter_arcs(
      south_node,
      ("8S1", "8S2"),
      west_node,
      west + np.array([0, -radius]),
      0,
      clockwise=True,
    )
    add_straight(west_node, FirstPGBlock.NODE_1, west, self.junction)

    self.add_sockets(PGBlockSocket(Road(west_node, FirstPGBlock.NODE_1),
                                   -Road(west_node, FirstPGBlock.NODE_1)))
    return True

  def get_intermediate_spawn_lanes(self):
    return self.block_network.get_positive_lanes()


class FigureEightPGMap(PGMap):
  """PGMap with an explicit closed figure-eight cycle and shared crossing."""

  def _generate(self):
    config = self._config
    track_size = float(config.get("exit_length", 60))
    # create_map stores track_size in exit_length; preserve its five-to-one
    # formula (60 m straight arm → 300 m curve radius).
    curve_radius = track_size * 5
    lane_width = float(config.get(self.LANE_WIDTH, 4.5))
    lane_num = int(config.get(self.LANE_NUM, 2))
    start_position = config.get("start_position", [0, 0])

    first = FirstPGBlock(
      global_network=self.road_network,
      lane_width=lane_width,
      lane_num=lane_num,
      render_root_np=self.engine.worldNP,
      physics_world=self.engine.physics_world,
      length=track_size,
      start_point=start_position,
      ignore_intersection_checking=True,
    )
    loop = FigureEightLoopBlock(
      1,
      first.get_socket(0),
      self.road_network,
      random_seed=self.engine.global_random_seed,
      junction=start_position,
      track_size=track_size,
      curve_radius=curve_radius,
    )
    loop.construct_loop(self.engine.worldNP, self.engine.physics_world)
    self.blocks = [first, loop]
    self.loop_cycle_nodes = (
      FirstPGBlock.NODE_1,
      FirstPGBlock.NODE_2,
      FirstPGBlock.NODE_3,
      "8E",
      "8E1",
      "8E2",
      "8N",
      FirstPGBlock.NODE_1,
      "8S",
      "8S1",
      "8S2",
      "8W",
      FirstPGBlock.NODE_1,
    )
    self.road_network.after_init()
    self._validate_loop_topology()

  def _validate_loop_topology(self):
    cycle = self.loop_cycle_nodes
    graph = self.road_network.graph
    if cycle[0] != cycle[-1]:
      raise RuntimeError("Figure-eight route does not close at its starting node")

    for start, end in zip(cycle[:-1], cycle[1:]):
      if end not in graph.get(start, {}):
        raise RuntimeError(f"Figure-eight route is missing road edge {start!r} -> {end!r}")

    # Verify lane index continuity around every edge, including both visits to
    # the shared junction and the final edge back to the first FirstPGBlock lane.
    for index, node in enumerate(cycle[:-1]):
      previous = cycle[index - 1] if index else cycle[-2]
      following = cycle[index + 1]
      incoming = graph[previous][node]
      outgoing = graph[node][following]
      if len(incoming) != len(outgoing):
        raise RuntimeError(f"Figure-eight lane count changes at node {node!r}")
      for lane_index, (incoming_lane, outgoing_lane) in enumerate(zip(incoming, outgoing)):
        endpoint_gap = np.linalg.norm(
          incoming_lane.position(incoming_lane.length, 0) - outgoing_lane.position(0, 0)
        )
        heading_error = math.atan2(
          math.sin(incoming_lane.heading_theta_at(incoming_lane.length) - outgoing_lane.heading_theta_at(0)),
          math.cos(incoming_lane.heading_theta_at(incoming_lane.length) - outgoing_lane.heading_theta_at(0)),
        )
        if endpoint_gap > 1e-5 or abs(heading_error) > 1e-5:
          raise RuntimeError(
            f"Figure-eight lane {lane_index} is discontinuous at {node!r}: "
            f"gap={endpoint_gap:.6f}m heading_error={heading_error:.6f}rad"
          )


class FigureEightPGMapManager(PGMapManager):
  """Use the custom loop map while retaining MetaDrive's normal map lifecycle."""

  def reset(self):
    config = self.engine.global_config.copy()
    current_seed = self.engine.global_seed
    if self.maps[current_seed] is None:
      map_config = config["map_config"]
      map_config.update({"seed": current_seed})
      map_config = self.add_random_to_map(map_config)
      map = self.spawn_object(FigureEightPGMap, map_config=map_config, random_seed=None)
      self.current_map = map
      if self.engine.global_config["store_map"]:
        self.maps[current_seed] = map
    else:
      self.load_map(self.maps[current_seed])


def install_figure_eight_navigation():
  """Keep MetaDrive's navigation checkpoints rolling around the closed cycle."""
  from metadrive.component.navigation_module.node_network_navigation import NodeNetworkNavigation

  if getattr(NodeNetworkNavigation, "_figure_eight_patch_installed", False):
    return

  original_update_localization = NodeNetworkNavigation.update_localization

  def set_loop_route(self, current_lane_index, destination):
    self.spawn_road = current_lane_index[:-1]
    positive_cycle = list(self.map.loop_cycle_nodes)
    cycles = [positive_cycle]
    cycles.append(["-" + node for node in reversed(positive_cycle)])
    current_edge = tuple(current_lane_index[:2])
    selected = None
    for cycle in cycles:
      edges = list(zip(cycle[:-1], cycle[1:]))
      if current_edge in edges:
        offset = edges.index(current_edge)
        ordered_edges = edges[offset:] + edges[:offset]
        selected = [ordered_edges[0][0]] + [end for _, end in ordered_edges]
        break

    if selected is None:
      return NodeNetworkNavigation._figure_eight_original_set_route(self, current_lane_index, destination)

    self._figure_eight_cycle_nodes = selected
    self.checkpoints = selected + selected[1:]
    self._target_checkpoints_index = [0, 1]
    self.final_road = Road(self.checkpoints[-2], self.checkpoints[-1])
    final_lanes = self.final_road.get_lanes(self.map.road_network)
    self.final_lane = final_lanes[-1]
    self._navi_info.fill(0.0)
    self.current_ref_lanes = self.map.road_network.graph[self.checkpoints[0]][self.checkpoints[1]]
    self.next_ref_lanes = self.map.road_network.graph[self.checkpoints[1]][self.checkpoints[2]]
    self.current_road = Road(self.checkpoints[0], self.checkpoints[1])
    self.next_road = Road(self.checkpoints[1], self.checkpoints[2])
    self.travelled_length = 0.0
    self._last_long_in_ref_lane = 0.0
    self.total_length = sum(
      self.map.road_network.graph[start][end][0].length
      for start, end in zip(self.checkpoints[:-1], self.checkpoints[1:])
    )

    if self._dest_node_path is not None:
      ref_lane = final_lanes[0]
      later_middle = (float(self.get_current_lane_num()) / 2 - 0.5) * self.get_current_lane_width()
      check_point = ref_lane.position(ref_lane.length, later_middle)
      self._dest_node_path.setPos(check_point[0], check_point[1], self.MARK_HEIGHT)

  def update_loop_localization(self, ego_vehicle):
    if getattr(self, "_figure_eight_cycle_nodes", None):
      if self._target_checkpoints_index[1] >= len(self.checkpoints) - 3:
        cycle = self._figure_eight_cycle_nodes
        extension = cycle[1:]
        self.checkpoints.extend(extension)
        self.total_length += sum(
          self.map.road_network.graph[start][end][0].length
          for start, end in zip(cycle[:-1], cycle[1:])
        )
        self.final_road = Road(self.checkpoints[-2], self.checkpoints[-1])
        self.final_lane = self.final_road.get_lanes(self.map.road_network)[-1]
    return original_update_localization(self, ego_vehicle)

  NodeNetworkNavigation._figure_eight_original_set_route = NodeNetworkNavigation.set_route
  NodeNetworkNavigation.set_route = set_loop_route
  NodeNetworkNavigation.update_localization = update_loop_localization
  NodeNetworkNavigation._figure_eight_patch_installed = True
