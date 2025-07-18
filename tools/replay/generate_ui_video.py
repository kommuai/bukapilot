#!/usr/bin/env python3
import argparse
import os
import sys

import cv2
import numpy as np
import pygame

import cereal.messaging as messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.transformations.camera import tici_fcam_intrinsics
from openpilot.tools.replay.lib.ui_helpers import (UP,
                                         BLACK, GREEN,
                                         YELLOW, Calibration,
                                         get_blank_lid_overlay, init_plots,
                                         maybe_update_radar_points, plot_lead,
                                         plot_model,
                                         pygame_modules_have_loaded, _INTRINSICS)
from cereal.visionipc import VisionIpcClient, VisionStreamType

os.environ['BASEDIR'] = BASEDIR

ANGLE_SCALE = 5.0
OUTPUT_FPS = 20.0
OVERLAY_SIZE = (1080, 1920)
ACCEL_PLOT_LEN = 100

accel_hist = {
  'a_ego': np.zeros(ACCEL_PLOT_LEN, dtype=np.float32),
  'a_target': np.zeros(ACCEL_PLOT_LEN, dtype=np.float32),
  'v_ego': np.zeros(ACCEL_PLOT_LEN, dtype=np.float32),
  'v_target': np.zeros(ACCEL_PLOT_LEN, dtype=np.float32),
  'user_gas': np.zeros(ACCEL_PLOT_LEN, dtype=np.float32),
  'computer_gas': np.zeros(ACCEL_PLOT_LEN, dtype=np.float32),
  'user_brake': np.zeros(ACCEL_PLOT_LEN, dtype=np.float32),
  'computer_brake': np.zeros(ACCEL_PLOT_LEN, dtype=np.float32),
}

def draw_longitudinal_plan_plot(surface):
  h, w = surface.shape[:2]
  bg_color = (32, 32, 32)
  cv2.rectangle(surface, (0, 0), (w, h), bg_color, -1)

  def scale(val, vmin, vmax):
    return int(h * (1 - np.clip((val - vmin) / (vmax - vmin), 0.0, 1.0)))

  colors = {
    'a_ego': (0, 255, 0),
    'a_target': (255, 255, 0),
  }

  for i in range(1, ACCEL_PLOT_LEN):
    x0 = (i - 1) * w // ACCEL_PLOT_LEN
    x1 = i * w // ACCEL_PLOT_LEN

    for key, color in colors.items():
      y0 = scale(accel_hist[key][i - 1], -3.0, 3.0)
      y1 = scale(accel_hist[key][i], -3.0, 3.0)
      cv2.line(surface, (x0, y0), (x1, y1), color, 2)

  cv2.putText(surface, "Longitudinal Plan", (10, 30),
              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

  # Legend
  for i, (label, color) in enumerate(colors.items()):
    y = h - 40 + i * 20
    cv2.rectangle(surface, (10, y - 10), (20, y), color, -1)
    cv2.putText(surface, label, (25, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


def draw_gas_brake_plot(surface):
  h, w = surface.shape[:2]
  bg_color = (32, 32, 32)
  cv2.rectangle(surface, (0, 0), (w, h), bg_color, -1)

  def scale(val, vmin, vmax):
    return int(h * (1 - np.clip((val - vmin) / (vmax - vmin), 0.0, 1.0)))

  colors = {
    'user_brake': (255, 0, 0),
    'computer_brake': (128, 0, 0),
    'user_gas': (0, 255, 255),
    'computer_gas': (0, 128, 255)
  }

  for i in range(1, ACCEL_PLOT_LEN):
    x0 = (i - 1) * w // ACCEL_PLOT_LEN
    x1 = i * w // ACCEL_PLOT_LEN

    for key, color in colors.items():
      y0 = scale(accel_hist[key][i - 1], -3.0, 3.0)
      y1 = scale(accel_hist[key][i], -3.0, 3.0)
      cv2.line(surface, (x0, y0), (x1, y1), color, 2)

  cv2.putText(surface, "Gas, Brakes", (10, 30),
              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

  # Horizontal Legend
  legend_y = h - 30
  x_cursor = 10
  for label, color in colors.items():
    cv2.rectangle(surface, (x_cursor, legend_y - 10), (x_cursor + 15, legend_y), color, -1)
    cv2.putText(surface, label, (x_cursor + 20, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    x_cursor += 20 + cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0] + 15

def decode_yuv_to_rgb(yuv_data, width, height, stride):
  imgff = np.frombuffer(yuv_data, dtype=np.uint8).reshape((len(yuv_data) // stride, stride))
  return cv2.cvtColor(imgff[:height * 3 // 2, :width], cv2.COLOR_YUV2RGB_NV12)

def crop_center_80_percent(img):
  h, w = img.shape[:2]
  crop_h = int(h * 0.8)
  y1 = (h - crop_h) // 2
  y2 = y1 + crop_h
  return img[y1:y2, :]

def update_overlay_frame(img, top_down, wide_img):
  overlay = np.zeros((1920, 1080, 3), dtype=np.uint8)
  cropped_img = crop_center_80_percent(img)
  cropped_wide = crop_center_80_percent(wide_img)
  td_np = pygame.surfarray.array3d(top_down[0]).swapaxes(0, 1)

  td_np = cv2.resize(td_np, (1080, 640), interpolation=cv2.INTER_LINEAR)
  overlay[0:640, :] = cv2.resize(cropped_img, (1080, 640), interpolation=cv2.INTER_LINEAR)
  overlay[640:1280, :] = td_np
  overlay[1280:1920, :] = cv2.resize(cropped_wide, (1080, 640), interpolation=cv2.INTER_LINEAR)

  # Label the camera views
  cv2.putText(overlay, "Telescopic Camera", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
  cv2.putText(overlay, "In-Cabin Camera", (30, 1280 + 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

  return overlay

def draw_text_info(overlay, sm):
  road_overlay = overlay[0:640, :]
  text_color = (255, 255, 255)

  alert1 = sm['controlsState'].alertText1
  alert2 = sm['controlsState'].alertText2

  if alert1:
    size1 = cv2.getTextSize(alert1, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 2)[0]
    x1 = (road_overlay.shape[1] - size1[0]) // 2
    y1 = (road_overlay.shape[0] // 2) - 10
    cv2.putText(road_overlay, alert1, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.5, text_color, 2)

  if alert2:
    size2 = cv2.getTextSize(alert2, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 2)[0]
    x2 = (road_overlay.shape[1] - size2[0]) // 2
    y2 = (road_overlay.shape[0] // 2) + size2[1] + 10
    cv2.putText(road_overlay, alert2, (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, 1.2, text_color, 2)

  lines = [
    f"ENGAGED: {'YES' if sm['controlsState'].enabled else 'NO'}",
    f"SPEED: {round(sm['carState'].vEgo * 3.6, 1)} km/h",
    f"SET SPEED: {round(sm['carState'].cruiseState.speed * 3.6, 1)} km/h",
    f"LONG MPC SOURCE: {sm['longitudinalPlan'].longitudinalPlanSource}",
    f"STEER RATIO: {round(sm['liveParameters'].steerRatio, 2)}",
    f"STEERING ANGLE: {round(sm['carState'].steeringAngleDeg, 2)} deg"
  ]
  for i, text in enumerate(lines):
    cv2.putText(road_overlay, text, (20, 80 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)

def draw_accel_plot(radar_overlay):
  h, w = radar_overlay.shape[:2]
  plot_h = h // 2
  plot_w = w // 2

  bg_color = (32, 32, 32)
  cv2.rectangle(radar_overlay, (0, 0), (plot_w, plot_h), bg_color, -1)

  # normalize values
  def scale(val, vmin, vmax):
    return int(plot_h * (1 - np.clip((val - vmin) / (vmax - vmin), 0.0, 1.0)))

  colors = {
    'a_ego': (0, 255, 0),
    'a_target': (255, 255, 0),
    'user_brake': (255, 0, 0),
    'computer_brake': (128, 0, 0),
    'user_gas': (0, 255, 255),
    'computer_gas': (0, 128, 255)
  }

  for i in range(1, ACCEL_PLOT_LEN):
    x0 = (i - 1) * plot_w // ACCEL_PLOT_LEN
    x1 = i * plot_w // ACCEL_PLOT_LEN

    for key, color in colors.items():
      y0 = scale(accel_hist[key][i - 1], -3.0, 3.0)
      y1 = scale(accel_hist[key][i], -3.0, 3.0)
      cv2.line(radar_overlay, (x0, y0), (x1, y1), color, 2)

  # Title
  cv2.putText(radar_overlay, "Longitudinal Planner (Accel/Brake)", (10, 30),
              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

  # Legend box
  legend_x, legend_y = 10, plot_h + 10
  font_scale = 0.5
  spacing = 18

  for i, (label, color) in enumerate(colors.items()):
    y = legend_y + i * spacing
    cv2.rectangle(radar_overlay, (legend_x, y - 10), (legend_x + 10, y), color, -1)
    cv2.putText(radar_overlay, label, (legend_x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 1)

def ui_thread(addr):
  cv2.setNumThreads(1)
  pygame.init()
  pygame.font.init()
  assert pygame_modules_have_loaded()

  sm = messaging.SubMaster([
    'carState', 'longitudinalPlan', 'carControl', 'radarState', 'liveCalibration', 'controlsState', 'liveTracks', 'modelV2', 'liveParameters', 'roadCameraState', 'wideRoadCameraState'
  ], addr=addr)

  calibration = None
  lid_overlay_blank = get_blank_lid_overlay(UP)
  top_down_surface = pygame.surface.Surface((int(UP.lidar_x * 1.5), int(UP.lidar_y * 1.5)), 0, 8)

  vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
  vipc_wide_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, True)

  img = np.zeros((1200, 1920, 3), dtype='uint8')
  wide_img = np.zeros((1200, 1920, 3), dtype='uint8')

  fourcc = cv2.VideoWriter_fourcc(*'mp4v')
  video_writer = cv2.VideoWriter("generated_visuals.mp4", fourcc, OUTPUT_FPS, OVERLAY_SIZE)

  while True:
    lid_overlay = lid_overlay_blank.copy()
    top_down = top_down_surface, lid_overlay

    if not vipc_client.is_connected():
      vipc_client.connect(True)
    if not vipc_wide_client.is_connected():
      vipc_wide_client.connect(True)

    try:
      yuv_img_raw = vipc_client.recv()
      if yuv_img_raw is None or not yuv_img_raw.data.any():
        continue
    except Exception as e:
      print(f"[VisionIPC Error] road: {e}")
      continue

    try:
      yuv_wide_img_raw = vipc_wide_client.recv()
    except Exception as e:
      print(f"[VisionIPC Error] wide: {e}")
      yuv_wide_img_raw = None

    sm.update(0)

    # Update accel_hist
    for key in accel_hist:
      accel_hist[key][:-1] = accel_hist[key][1:]

    accel_hist['a_ego'][-1] = sm['carState'].aEgo
    accel_hist['v_ego'][-1] = sm['carState'].vEgo
    if len(sm['longitudinalPlan'].accels):
      accel_hist['a_target'][-1] = sm['longitudinalPlan'].accels[0]
    if len(sm['longitudinalPlan'].speeds):
      accel_hist['v_target'][-1] = sm['longitudinalPlan'].speeds[0]
    accel_hist['user_gas'][-1] = sm['carState'].gas
    accel_hist['computer_gas'][-1] = max(sm['carControl'].actuators.accel, 0.0)
    accel_hist['user_brake'][-1] = sm['carState'].brake
    accel_hist['computer_brake'][-1] = -min(sm['carControl'].actuators.accel, 0.0)

    rgb = decode_yuv_to_rgb(yuv_img_raw.data, vipc_client.width, vipc_client.height, vipc_client.stride)
    img = rgb

    if yuv_wide_img_raw is not None and yuv_wide_img_raw.data.any():
      wide_rgb = decode_yuv_to_rgb(yuv_wide_img_raw.data, vipc_wide_client.width, vipc_wide_client.height, vipc_wide_client.stride)
      wide_img = wide_rgb

    if sm.recv_frame['modelV2']:
      plot_model(sm['modelV2'], img, calibration, top_down)
    if sm.recv_frame['radarState']:
      plot_lead(sm['radarState'], top_down)

    maybe_update_radar_points(sm['liveTracks'], top_down[1])

    resized_overlay = cv2.resize(top_down[1], (top_down[0].get_width(), top_down[0].get_height()))
    pygame.surfarray.blit_array(top_down[0], resized_overlay.swapaxes(0, 1))

    if sm.updated['liveCalibration']:
      rpyCalib = np.asarray(sm['liveCalibration'].rpyCalib)
      calib_scale = 1.0
      calibration = Calibration(img.shape[0] * img.shape[1], rpyCalib, tici_fcam_intrinsics, calib_scale)

    overlay_frame = update_overlay_frame(img, top_down, wide_img)

    # Split radar view and longitudinal plots into two rows
    radar_overlay = overlay_frame[640:960, :]
    planner_overlay = overlay_frame[960:1280, :]

    # Radar points visualization (top-down)
    #radar_overlay[:, :] = cv2.resize(pygame.surfarray.array3d(top_down[0]).swapaxes(0, 1), (1080, 320))
    radar_overlay[:, :] = cv2.flip(
      cv2.resize(pygame.surfarray.array3d(top_down[0]).swapaxes(0, 1), (1080, 320)),
      0  # flip vertically
    )
    # Longitudinal planner and gas/brake plots
    # Split planner_overlay into top and bottom halves
    h, w = planner_overlay.shape[:2]
    longitudinal_plot = planner_overlay[0:h//2, :]
    gas_brake_plot = planner_overlay[h//2:h, :]

    draw_longitudinal_plan_plot(longitudinal_plot)
    draw_gas_brake_plot(gas_brake_plot)

    draw_text_info(overlay_frame, sm)
    video_writer.write(cv2.cvtColor(overlay_frame, cv2.COLOR_RGB2BGR))

def get_arg_parser():
  parser = argparse.ArgumentParser(description="Show replay data in a UI.")
  parser.add_argument("ip_address", nargs="?", default="127.0.0.1", help="The ip address on which to receive zmq messages.")
  parser.add_argument("--frame-address", default=None, help="The frame address (fully qualified ZMQ endpoint for frames) on which to receive zmq messages.")
  return parser

if __name__ == "__main__":
  args = get_arg_parser().parse_args(sys.argv[1:])
  if args.ip_address != "127.0.0.1":
    os.environ["ZMQ"] = "1"
    messaging.reset_context()
  ui_thread(args.ip_address)
