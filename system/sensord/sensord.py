#!/usr/bin/env python3
import time
import threading

import cereal.messaging as messaging
from cereal.services import SERVICE_LIST
from openpilot.common.realtime import config_realtime_process, Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.system.sensord.sensors.i2c_sensor import Sensor
from openpilot.system.sensord.sensors.icm42670_accel import ICM42670_Accel
from openpilot.system.sensord.sensors.icm42670_gyro import ICM42670_Gyro
from openpilot.system.sensord.sensors.icm42670_temp import ICM42670_Temp
from openpilot.system.sensord.sensors.lis2mdl_magn import LIS2MDL_Magn

I2C_BUS_IMU = 3


def polling_loop(sensor: Sensor, service: str, event: threading.Event) -> None:
  pm = messaging.PubMaster([service])
  rk = Ratekeeper(SERVICE_LIST[service].frequency, print_delay_threshold=None)
  while not event.is_set():
    try:
      evt = sensor.get_event()
      if sensor.is_data_valid():
        msg = messaging.new_message(service, valid=True)
        setattr(msg, service, evt)
        pm.send(service, msg)
    except Exception:
      cloudlog.exception(f"Error in {service} polling loop")
    rk.keep_time()

def main() -> None:
  config_realtime_process([1, ], 1)

  sensors_cfg = [
    (ICM42670_Accel(I2C_BUS_IMU), "accelerometer"),
    (ICM42670_Gyro(I2C_BUS_IMU), "gyroscope"),
    (ICM42670_Temp(I2C_BUS_IMU), "temperatureSensor"),
    (LIS2MDL_Magn(I2C_BUS_IMU), "magnetometer"),
  ]

  # Reset sensors
  for sensor, _ in sensors_cfg:
    try:
      sensor.reset()
    except Exception:
      cloudlog.exception(f"Error initializing {sensor} sensor")

  # Initialize sensors
  exit_event = threading.Event()
  threads: list[threading.Thread] = []
  for sensor, service in sensors_cfg:
    try:
      sensor.init()
      threads.append(threading.Thread(
        target=polling_loop,
        args=(sensor, service, exit_event),
        daemon=True
      ))
    except Exception:
      cloudlog.exception(f"Error initializing {service} sensor")

  try:
    for t in threads:
      t.start()
    while any(t.is_alive() for t in threads):
      time.sleep(1)
  except KeyboardInterrupt:
    pass
  finally:
    exit_event.set()
    for t in threads:
      if t.is_alive():
        t.join()

    for sensor, _ in sensors_cfg:
      try:
        sensor.shutdown()
      except Exception:
        cloudlog.exception("Error shutting down sensor")

if __name__ == "__main__":
  main()
