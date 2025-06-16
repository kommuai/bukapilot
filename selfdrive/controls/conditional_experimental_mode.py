#!/usr/bin/env python3

import numpy as np
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.conversions import Conversions as CV

THRESHOLD = 0.63
CRUISING_SPEED = 5  # m/s (~18kmh)
LOW_SPEED_LIMIT = 9  # m/s (~40kmh)
TURN_SIGNAL_SPEED = 55 * CV.MPH_TO_MS  # 24.6 m/s

params = Params()


class ConditionalExperimentalMode:
  def __init__(self):
    self.curvature_filter = FirstOrderFilter(0, 1, DT_MDL)
    self.slow_lead_filter = FirstOrderFilter(0, 1, DT_MDL)
    self.stop_light_filter = FirstOrderFilter(0, 1, DT_MDL)

    self.experimental_mode = False

  def update(self, car_state, lead, model_data):
    v_ego = car_state.vEgo

    # --- Road curvature detection ---
    curvature = self.calculate_curvature(model_data, v_ego)
    road_curve = (0.9 / abs(curvature))**0.5 < v_ego > CRUISING_SPEED
    self.curvature_filter.update(road_curve)
    curve_detected = self.curvature_filter.x >= THRESHOLD

    # --- Slow/stopped lead detection ---
    if lead.status:
      # slow lead that is less than 30kmh or relative velocity of -2.88m/ss
      slow_lead = lead.vLead < 8.33 or lead.vRel < -2.88
      self.slow_lead_filter.update(slow_lead)
      lead_detected = self.slow_lead_filter.x >= THRESHOLD
    else:
      self.slow_lead_filter.x = 0
      lead_detected = False

    # --- Stop light/sign detection ---
    x_pos = model_data.position.x
    if x_pos and len(x_pos) > 0:
      stop_light = x_pos[-1] < CRUISING_SPEED * len(x_pos) * DT_MDL
    else:
      stop_light = False
    self.stop_light_filter.update(stop_light)
    stop_light_detected = self.stop_light_filter.x >= THRESHOLD and not lead.status

    # --- Signal below speed detection ---
    signal_for_turn = v_ego < TURN_SIGNAL_SPEED and (car_state.leftBlinker or car_state.rightBlinker)

    # --- Low speed cruising ---
    below_low_speed = v_ego < LOW_SPEED_LIMIT

    # --- Final CEM trigger condition ---
    should_enable = (
      curve_detected or
      lead_detected or
      stop_light_detected or
      signal_for_turn or
      below_low_speed
    )

    # --- Handle ExperimentalMode param state ---
    if should_enable and not self.experimental_mode:
      params.put_bool("ExperimentalMode", True)
      self.experimental_mode = True
    elif not should_enable and self.experimental_mode:
      params.put_bool("ExperimentalMode", False)
      self.experimental_mode = False

  @staticmethod
  def calculate_curvature(model_data, v_ego):
    orientation_rate = np.array(model_data.orientationRate.z)
    velocity = np.array(model_data.velocity.x)

    if orientation_rate.size == 0 or velocity.size == 0:
      return 0.0

    lat_acc = orientation_rate * velocity
    max_pred_lat_acc = max(np.max(lat_acc), np.min(lat_acc), key=abs)
    return float(max_pred_lat_acc / max(v_ego, 1)**2)
