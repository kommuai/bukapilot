import math
import numpy as np
import time
import wave
import subprocess
import os

from cereal import car, messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import Ratekeeper
from openpilot.common.retry import retry
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params

from openpilot.system import micd

params = Params()
SAMPLE_RATE = 48000
SAMPLE_BUFFER = 4096  # (approx 100ms)
MAX_VOLUME = 1.0
MAX_VOLUME_QUIET_MODE = 0.4
MIN_VOLUME = 1.0
CONTROLS_TIMEOUT = 5  # 5 seconds
FILTER_DT = 1. / (micd.SAMPLE_RATE / micd.FFT_SAMPLES)

AMBIENT_DB = 30  # DB where MIN_VOLUME is applied
DB_SCALE = 30   # AMBIENT_DB + DB_SCALE is where MAX_VOLUME is applied

AudibleAlert = car.CarControl.HUDControl.AudibleAlert

# Static mapping for all alerts except refuse
sound_list: dict[int, tuple[str, int | None, float]] = {
  AudibleAlert.engage: ("engage.wav", 1, MAX_VOLUME),
  AudibleAlert.disengage: ("disengage.wav", 1, MAX_VOLUME),
  # AudibleAlert.refuse handled dynamically
  AudibleAlert.prompt: ("prompt.wav", 1, MAX_VOLUME),
  AudibleAlert.promptRepeat: ("prompt.wav", None, MAX_VOLUME),
  AudibleAlert.promptDistracted: ("prompt_distracted.wav", None, MAX_VOLUME),
  AudibleAlert.warningSoft: ("warning_soft.wav", None, MAX_VOLUME),
  AudibleAlert.warningImmediate: ("warning_immediate.wav", None, MAX_VOLUME),
}


def check_controls_timeout_alert(sm):
  controls_missing = time.monotonic() - sm.recv_time['controlsState']
  if controls_missing > CONTROLS_TIMEOUT:
    if sm['controlsState'].enabled and (controls_missing - CONTROLS_TIMEOUT) < 10:
      return True
  return False


class Soundd:
  def __init__(self):
    self.load_sounds()

    self.current_alert = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.controls_timeout_alert = False
    self.last_alert_type_name = None

    # cache now keyed by (alert, type_name)
    self.loaded_sounds: dict[tuple[int, str | None], np.ndarray]

    self.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)

  def load_sounds(self):
    self.loaded_sounds = {}
    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]
      self.loaded_sounds[(sound, None)] = self._load_wav(filename)

  def _load_wav(self, filename: str) -> np.ndarray:
    path = os.path.join(BASEDIR, "selfdrive/assets/sounds", filename)
    try:
      wavefile = wave.open(path, 'r')
    except FileNotFoundError:
      cloudlog.error(f"Missing sound file: {path}")
      return np.zeros(1, dtype=np.float32)

    if wavefile.getnchannels() != 1 or wavefile.getsampwidth() != 2:
      cloudlog.error(f"Incompatible WAV format: {filename}")
      return np.zeros(1, dtype=np.float32)

    if wavefile.getframerate() != SAMPLE_RATE:
      cloudlog.error(f"Wrong sample rate in {filename}: got {wavefile.getframerate()}, expected {SAMPLE_RATE}")
      return np.zeros(1, dtype=np.float32)

    length = wavefile.getnframes()
    return np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**15)

  def get_sound_data(self, frames):
    ret = np.zeros(frames, dtype=np.float32)

    if self.current_alert != AudibleAlert.none:
      # determine cache key
      key = (self.current_alert, self.last_alert_type_name if self.current_alert == AudibleAlert.refuse else None)

      if key not in self.loaded_sounds and self.current_alert == AudibleAlert.refuse:
        if self.last_alert_type_name:
          # take the LEFT side of "A/B"
          short_name = self.last_alert_type_name.split("/")[0]
          filename = f"/data/openpilot/selfdrive/assets/sounds/events/{short_name}.wav"
          cloudlog.info(f"Loading dynamic refuse sound: {filename}")
          sound_data = self._load_wav(filename)

          if sound_data.shape[0] <= 1:
            cloudlog.warning(f"Falling back to refuse.wav for {filename}")
            sound_data = self._load_wav("refuse.wav")

          self.loaded_sounds[key] = sound_data
        else:
          self.loaded_sounds[key] = self._load_wav("refuse.wav")

      num_loops = sound_list.get(self.current_alert, ("", 1, MAX_VOLUME))[1]
      sound_data = self.loaded_sounds.get(key, np.zeros(1, dtype=np.float32))
      written_frames = 0

      current_sound_frame = self.current_sound_frame % len(sound_data)
      loops = self.current_sound_frame // len(sound_data)

      while written_frames < frames and (num_loops is None or loops < num_loops):
        available_frames = sound_data.shape[0] - current_sound_frame
        frames_to_write = min(available_frames, frames - written_frames)
        ret[written_frames:written_frames+frames_to_write] = sound_data[current_sound_frame:current_sound_frame+frames_to_write]
        written_frames += frames_to_write
        self.current_sound_frame += frames_to_write

    return ret * self.current_volume

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    if status:
      cloudlog.warning(f"soundd stream over/underflow: {status}")
    data_out[:frames, 0] = self.get_sound_data(frames)

  def update_alert(self, new_alert, quiet_mode, alert_type_name=None):
    if quiet_mode:
      allowed_alerts = new_alert in {
          AudibleAlert.none,
          AudibleAlert.promptRepeat,
          AudibleAlert.promptDistracted,
          AudibleAlert.prompt,
          AudibleAlert.warningSoft,
          AudibleAlert.warningImmediate,
      }
    else:
      allowed_alerts = True

    if allowed_alerts:
      current_alert_played_once = (
        self.current_alert == AudibleAlert.none or
        self.current_sound_frame > len(self.loaded_sounds.get((self.current_alert, self.last_alert_type_name), []))
      )
      if self.current_alert != new_alert and (new_alert != AudibleAlert.none or current_alert_played_once):
        self.current_alert = new_alert
        self.current_sound_frame = 0

        if new_alert == AudibleAlert.refuse:
          self.last_alert_type_name = alert_type_name
        else:
          self.last_alert_type_name = None

  def get_audible_alert(self, sm, quiet_mode):
    if sm.updated['controlsState']:
      new_alert = sm['controlsState'].alertSound.raw
      alert_type_name = sm['controlsState'].alertType  # e.g. "A/noEntry"
      self.update_alert(new_alert, quiet_mode, alert_type_name)
    elif check_controls_timeout_alert(sm):
      self.update_alert(AudibleAlert.warningImmediate, quiet_mode)
      self.controls_timeout_alert = True
    elif self.controls_timeout_alert:
      self.update_alert(AudibleAlert.none, quiet_mode)
      self.controls_timeout_alert = False

  def calculate_volume(self, weighted_db, quiet_mode):
    max_vol = MAX_VOLUME_QUIET_MODE if quiet_mode else MAX_VOLUME
    volume = ((weighted_db - AMBIENT_DB) / DB_SCALE) * (max_vol - MIN_VOLUME) + MIN_VOLUME
    return math.pow(10, (np.clip(volume, MIN_VOLUME, MAX_VOLUME) - 1))

  @retry(attempts=7, delay=3)
  def get_stream(self, sd):
    sd._terminate()
    sd._initialize()
    return sd.OutputStream(channels=1, samplerate=SAMPLE_RATE,
                           callback=self.callback, blocksize=SAMPLE_BUFFER)

  def soundd_thread(self):
    import sounddevice as sd
    sm = messaging.SubMaster(['controlsState', 'microphone'])

    with self.get_stream(sd) as stream:
      rk = Ratekeeper(20)
      cloudlog.info(f"soundd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")

      while True:
        sm.update(0)

        quiet_mode = params.get_bool("QuietMode")
        if sm.updated['microphone'] and self.current_alert == AudibleAlert.none:
          self.spl_filter_weighted.update(sm["microphone"].soundPressureWeightedDb)
          self.current_volume = self.calculate_volume(float(self.spl_filter_weighted.x), quiet_mode)

        self.get_audible_alert(sm, quiet_mode)
        rk.keep_time()
        assert stream.active


def main():
  s = Soundd()
  subprocess.run(["amixer", "sset", "PCM", "100%"], check=True)
  subprocess.run(["amixer", "-c", "0", "sset", "\"Speaker\"", "on"], check=True)
  s.soundd_thread()


if __name__ == "__main__":
  main()

