#!/usr/bin/env python3
import numpy as np
from functools import cache
import threading

from cereal import messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.utils import retry
from openpilot.common.swaglog import cloudlog

RATE = 10
FFT_SAMPLES = 1600  # 100ms at 16 kHz
REFERENCE_SPL = 2e-5  # newtons/m^2
SAMPLE_RATE = 16000
SAMPLE_BUFFER = 800  # 50ms at 16 kHz

# Fallback sample rates if device doesn't support 16 kHz (ALSA -22)
FALLBACK_RATES = [
  (16000, 800),   # (rate, blocksize for 50ms)
  (48000, 2400),
  (44100, 2205),
  (8000, 400),
]


@cache
def get_a_weighting_filter():
  # Calculate the A-weighting filter
  # https://en.wikipedia.org/wiki/A-weighting
  freqs = np.fft.fftfreq(FFT_SAMPLES, d=1 / SAMPLE_RATE)
  A = 12194 ** 2 * freqs ** 4 / ((freqs ** 2 + 20.6 ** 2) * (freqs ** 2 + 12194 ** 2) * np.sqrt((freqs ** 2 + 107.7 ** 2) * (freqs ** 2 + 737.9 ** 2)))
  return A / np.max(A)


def calculate_spl(measurements):
  # https://www.engineeringtoolbox.com/sound-pressure-d_711.html
  sound_pressure = np.sqrt(np.mean(measurements ** 2))  # RMS of amplitudes
  if sound_pressure > 0:
    sound_pressure_level = 20 * np.log10(sound_pressure / REFERENCE_SPL)  # dB
  else:
    sound_pressure_level = 0
  return sound_pressure, sound_pressure_level


def apply_a_weighting(measurements: np.ndarray) -> np.ndarray:
  # Generate a Hanning window of the same length as the audio measurements
  measurements_windowed = measurements * np.hanning(len(measurements))

  # Apply the A-weighting filter to the signal
  return np.abs(np.fft.ifft(np.fft.fft(measurements_windowed) * get_a_weighting_filter()))


def resample_to_16k(samples: np.ndarray, from_rate: int) -> np.ndarray:
  """Resample to 16000 Hz for pipeline. Simple integer ratio or linear interpolation."""
  if from_rate == SAMPLE_RATE:
    return samples
  if from_rate == 48000:  # 3:1
    return samples[::3].copy()
  if from_rate == 8000:   # 1:2
    return np.repeat(samples, 2)
  # 44100 or other: linear interpolation
  n = len(samples)
  m = int(round(n * SAMPLE_RATE / from_rate))
  return np.interp(np.linspace(0, n - 1, m, dtype=np.float32), np.arange(n), samples).astype(np.float32)


class Mic:
  def __init__(self):
    self.rk = Ratekeeper(RATE)
    self.pm = messaging.PubMaster(['soundPressure', 'rawAudioData'])

    self.measurements = np.empty(0)
    self.stream_samplerate = SAMPLE_RATE  # actual device rate; callback resamples to SAMPLE_RATE

    self.sound_pressure = 0
    self.sound_pressure_weighted = 0
    self.sound_pressure_level_weighted = 0

    self.lock = threading.Lock()

  def update(self):
    with self.lock:
      sound_pressure = self.sound_pressure
      sound_pressure_weighted = self.sound_pressure_weighted
      sound_pressure_level_weighted = self.sound_pressure_level_weighted

    msg = messaging.new_message('soundPressure', valid=True)
    msg.soundPressure.soundPressure = float(sound_pressure)
    msg.soundPressure.soundPressureWeighted = float(sound_pressure_weighted)
    msg.soundPressure.soundPressureWeightedDb = float(sound_pressure_level_weighted)

    self.pm.send('soundPressure', msg)
    self.rk.keep_time()

  def callback(self, indata, frames, time, status):
    """
    Using amplitude measurements, calculate an uncalibrated sound pressure and sound pressure level.
    Then apply A-weighting to the raw amplitudes and run the same calculations again.

    Logged A-weighted equivalents are rough approximations of the human-perceived loudness.
    """
    # Resample to 16 kHz if device opened at a fallback rate
    mono = indata[:, 0]
    if self.stream_samplerate != SAMPLE_RATE:
      mono = resample_to_16k(mono, self.stream_samplerate)

    msg = messaging.new_message('rawAudioData', valid=True)
    audio_data_int_16 = (mono * 32767).astype(np.int16)
    msg.rawAudioData.data = audio_data_int_16.tobytes()
    msg.rawAudioData.sampleRate = SAMPLE_RATE
    self.pm.send('rawAudioData', msg)

    with self.lock:
      self.measurements = np.concatenate((self.measurements, mono))

      while self.measurements.size >= FFT_SAMPLES:
        measurements = self.measurements[:FFT_SAMPLES]

        self.sound_pressure, _ = calculate_spl(measurements)
        measurements_weighted = apply_a_weighting(measurements)
        self.sound_pressure_weighted, self.sound_pressure_level_weighted = calculate_spl(measurements_weighted)

        self.measurements = self.measurements[FFT_SAMPLES:]

  @retry(attempts=10, delay=3)
  def get_stream(self, sd):
    # Reload sounddevice to reinitialize portaudio
    sd._terminate()
    sd._initialize()
    last_err = None
    for rate, blocksize in FALLBACK_RATES:
      try:
        self.stream_samplerate = rate
        stream = sd.InputStream(
          channels=1,
          samplerate=rate,
          callback=self.callback,
          blocksize=blocksize,
        )
        if rate != SAMPLE_RATE:
          cloudlog.info(f"micd: opened at {rate} Hz (resampling to {SAMPLE_RATE} Hz)")
        return stream
      except sd.PortAudioError as e:
        last_err = e
        cloudlog.debug(f"micd: rate {rate} Hz failed: {e}")
        continue
    raise (last_err or sd.PortAudioError("No supported sample rate", -9999, None))

  def micd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd
    try:
      with self.get_stream(sd) as stream:
        cloudlog.info(f"micd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")
        while True:
          self.update()
    except sd.PortAudioError as e:
      cloudlog.warning(f"micd: no microphone available or ALSA error, running without input: {e}")
      while True:
        self.update()


def main():
  mic = Mic()
  mic.micd_thread()


if __name__ == "__main__":
  main()
