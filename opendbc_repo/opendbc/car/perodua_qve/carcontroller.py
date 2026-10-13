#!/usr/bin/env python3
from cereal import car

from opendbc.car.interfaces import CarControllerBase
from opendbc.car.perodua_qve.peroduaqvecan import create_cam_lane_spoof
from opendbc.car.perodua_qve.values import CAM_LANE_SPOOF_PERIOD, ENABLE_CAM_LANE_SPOOF


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self._cam_line_mux_hi = False

  def update(self, CC, CS, now_nanos):
    del CC, now_nanos

    new_actuators = car.CarControl.Actuators.new_message()
    new_actuators.steeringAngleDeg = CS.out.steeringAngleDeg

    can_sends = []
    if ENABLE_CAM_LANE_SPOOF and (self.frame % CAM_LANE_SPOOF_PERIOD) == 0:
      # Idle/placeholder geometry; counter synced to stock cam session phase.
      line_mux = 0x80 if self._cam_line_mux_hi else 0x00
      self._cam_line_mux_hi = not self._cam_line_mux_hi
      can_sends.extend(create_cam_lane_spoof(CS.cam_session_counter, line_mux=line_mux))

    self.frame += 1
    return new_actuators, can_sends
