#!/usr/bin/env python3

import numpy as np
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.constants import CV

LOW_THRESHOLD = 0.55 #0.63
THRESHOLD = 0.63
MODEL_STOP_THRESHOLD = 0.5  # Lower threshold for faster model stopping detection
CRUISING_SPEED = 18 * CV.KPH_TO_MS
LOW_SPEED_LIMIT = 30 * CV.KPH_TO_MS
TURN_OFF_CEM_SPEED = 110 * CV.KPH_TO_MS

class ConditionalExperimentalMode:
  def __init__(self):
    self.curvature_filter = FirstOrderFilter(0, 1, DT_MDL)
    self.slow_lead_filter = FirstOrderFilter(0, 1, DT_MDL)
    # Faster filter for model stopping (0.3s time constant instead of 1.0s for quicker response)
    self.model_stop_filter = FirstOrderFilter(0, 0.3, DT_MDL)

    self.params = Params()
    self.cem_enabled = False
    self.frame = 0

  def update(self, car_state, lead, model_data, controls_state):
    # model_data is modelV2; curvature can be 0 when orientationRate.z/velocity.x are empty or zero (e.g. RKNN policy output or first frames)
    if self.frame % 10 == 0:
      self.cem_enabled = self.params.get_bool("ConditionalExperimentalMode")

    v_ego = car_state.vEgo

    # --- Model stopping detection (runs every frame for faster response) ---
    x_pos = model_data.position.x
    x_vel = model_data.velocity.x
    model_stopping_detected = False

    if x_pos and len(x_pos) > 0 and len(x_vel) > 0:
      # Check multiple time horizons for earlier detection
      # Model has 33 indices, predicting up to 10s ahead
      # Check indices: 14 (~2s), 20 (~4s), 25 (~6s), 32 (10s)
      idx_2s = min(14, len(x_vel) - 1)
      idx_4s = min(20, len(x_vel) - 1)
      idx_6s = min(25, len(x_vel) - 1)
      idx_10s = len(x_vel) - 1

      v_2s = x_vel[idx_2s]
      v_4s = x_vel[idx_4s]
      v_6s = x_vel[idx_6s]
      v_10s = x_vel[idx_10s]

      # Check acceleration for strong deceleration signals
      accel = 0
      if hasattr(model_data, 'acceleration') and model_data.acceleration.x and len(model_data.acceleration.x) > 0:
        accel = model_data.acceleration.x[-1]

      # Multiple conditions for faster detection
      model_stopping = (
        v_2s < 3.0 or      # Very slow at ~2s (< 10.8 km/h)
        v_4s < 2.5 or      # Slow at ~4s (< 9 km/h)
        v_6s < 2.0 or      # Very slow at ~6s (< 7.2 km/h)
        v_10s < 2.8 or     # Original check at 10s (< 10 km/h)
        accel < -1.5       # Strong deceleration (> 1.5 m/s²)
      )

      # Check for very strong stopping signal (immediate enable, bypass filter)
      very_slow = v_10s < 1.5  # Very slow (< 5.4 km/h)
      strong_decel = accel < -2.0  # Very strong deceleration (> 2.0 m/s²)

      if (very_slow or strong_decel) and not lead.status:
        # Immediate enable for clear stopping signals (only when no lead)
        model_stopping_detected = True
      else:
        # Use filtered detection for borderline cases
        self.model_stop_filter.update(model_stopping)
        model_stopping_detected = self.model_stop_filter.x >= MODEL_STOP_THRESHOLD and not lead.status
    else:
      self.model_stop_filter.update(False)

    # --- Other detections (run every 4 frames to limit param writes) ---
    if self.frame % 4 == 0:
      # --- Road curvature detection ---
      curvature = self.calculate_curvature(model_data, v_ego)
      # When curvature is 0 (straight road or model not outputting plan yet), use action.desiredCurvature as fallback so we don't divide by zero
      if curvature == 0.0 and hasattr(model_data, 'action') and model_data.action is not None:
        curvature = abs(model_data.action.desiredCurvature) or 0.0
      if curvature == 0.0:
        # Straight road or no valid curvature: do not divide by zero; road_curve = False
        road_curve = False
      else:
        road_curve = (1.2 / abs(curvature))**0.5 < v_ego > CRUISING_SPEED
      road_curve &= v_ego < TURN_OFF_CEM_SPEED
      self.curvature_filter.update(road_curve)
      curve_detected = self.curvature_filter.x >= THRESHOLD

      # --- Slow/stopped lead detection ---
      if lead.status:
        # slow lead that is less than 30kmh or relative velocity of -2.68m/ss
        slow_lead = (lead.vLead < 8.3 or lead.vRel < -2.68) and v_ego < TURN_OFF_CEM_SPEED
        self.slow_lead_filter.update(slow_lead)
        slow_lead_detected = self.slow_lead_filter.x >= LOW_THRESHOLD
      else:
        self.slow_lead_filter.x = 0
        slow_lead_detected = False

      # --- Low speed cruising ---
      below_low_speed = v_ego < LOW_SPEED_LIMIT

      should_enable = curve_detected or slow_lead_detected
      personality_type = int(self.params.get("LongitudinalPersonality"))

      if (personality_type == 0):
        if not self.cem_enabled:
          should_enable = False
      elif (personality_type == 1):
        should_enable |= model_stopping_detected
      else:
        should_enable |= model_stopping_detected or below_low_speed

      if should_enable != controls_state.experimentalMode:
        self.params.put_bool("ExperimentalMode", should_enable)

    self.frame += 1

  @staticmethod
  def calculate_curvature(model_data, v_ego):
    orientation_rate = np.array(model_data.orientationRate.z, dtype=np.float64)
    velocity = np.array(model_data.velocity.x, dtype=np.float64)

    if orientation_rate.size == 0 or velocity.size == 0:
      return 0.0

    lat_acc = orientation_rate * velocity
    if lat_acc.size == 0:
      return 0.0

    max_pred_lat_acc = max(float(np.max(lat_acc)), float(np.min(lat_acc)), key=abs)
    denom = max(float(v_ego), 1.0) ** 2
    if denom <= 0:
      return 0.0
    return float(max_pred_lat_acc / denom)
