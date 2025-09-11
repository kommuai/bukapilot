#!/usr/bin/env python3

import numpy as np
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.conversions import Conversions as CV

"""
  Conditional Experimental Mode
  Speed 70kmh above: Completely radar policy
  Speed 30kmh - 70kmh: Mix mode
  Speed 0kmh - 30kmh: Experimental Mode
"""

LOW_THRESHOLD = 0.55 #0.63
THRESHOLD = 0.63
CRUISING_SPEED = 18 * CV.KPH_TO_MS
LOW_SPEED_LIMIT = 30 * CV.KPH_TO_MS
TURN_OFF_CEM_SPEED = 70 * CV.KPH_TO_MS

class ConditionalExperimentalMode:
  def __init__(self):
    self.curvature_filter = FirstOrderFilter(0, 1, DT_MDL)
    self.slow_lead_filter = FirstOrderFilter(0, 1, DT_MDL)
    self.model_stop_filter = FirstOrderFilter(0, 1, DT_MDL)

    self.experimental_mode = False
    self.params = Params()

  def update(self, car_state, lead, model_data):
    v_ego = car_state.vEgo

    # --- Road curvature detection ---
    curvature = self.calculate_curvature(model_data, v_ego)
    road_curve = (0.9 / abs(curvature))**0.5 < v_ego > CRUISING_SPEED
    road_curve &= v_ego < TURN_OFF_CEM_SPEED
    self.curvature_filter.update(road_curve)
    curve_detected = self.curvature_filter.x >= THRESHOLD

    # --- Slow/stopped lead detection ---
    if lead.status:
      # slow lead that is less than 30kmh or relative velocity of -2.68m/ss
      slow_lead = (lead.vLead < 8.3 or lead.vRel < -2.68) and v_ego < TURN_OFF_CEM_SPEED
      self.slow_lead_filter.update(slow_lead)
      lead_detected = self.slow_lead_filter.x >= LOW_THRESHOLD
    else:
      self.slow_lead_filter.x = 0
      lead_detected = False

    # --- Model stopping ---
    x_pos = model_data.position.x
    x_vel = model_data.velocity.x
    if x_pos and len(x_pos) > 0:
      model_stopping = (x_pos[-1] < CRUISING_SPEED * len(x_pos) * DT_MDL) or x_vel[-1] < 2.8
    else:
      model_stopping = False
    self.model_stop_filter.update(model_stopping)
    model_stopping_detected = self.model_stop_filter.x >= THRESHOLD and not lead.status

    # --- Low speed cruising ---
    below_low_speed = v_ego < LOW_SPEED_LIMIT

    should_enable = (
      curve_detected or
      lead_detected
    )
    personality_type = int(self.params.get("LongitudinalPersonality"))

    if (personality_type == 0):
      should_enable = False
    elif (personality_type == 1):
      should_enable |= model_stopping or below_low_speed
    else:
      should_enable |= True

    if should_enable and not self.experimental_mode:
      self.params.put_bool("ExperimentalMode", True)
      self.experimental_mode = True
    elif not should_enable and self.experimental_mode:
      self.params.put_bool("ExperimentalMode", False)
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
