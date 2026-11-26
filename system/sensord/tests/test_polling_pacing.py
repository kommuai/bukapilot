from types import SimpleNamespace

from openpilot.system.sensord import sensord


class StopAfterChecks:
  def __init__(self, iterations: int):
    self.iterations = iterations
    self.checks = 0

  def is_set(self) -> bool:
    self.checks += 1
    return self.checks > self.iterations


class RecordingRatekeeper:
  instances = []

  def __init__(self, *_args, **_kwargs):
    self.keep_time_calls = 0
    self.instances.append(self)

  def keep_time(self):
    self.keep_time_calls += 1


class RecordingPublisher:
  instances = []

  def __init__(self, _services):
    self.sent = []
    self.instances.append(self)

  def send(self, service, message):
    self.sent.append((service, message))


class SequenceSensor:
  def __init__(self, validity):
    self.validity = iter(validity)
    self.events = 0

  def get_event(self):
    self.events += 1
    return f"event-{self.events}"

  def is_data_valid(self):
    return next(self.validity)


def setup_polling_fakes(monkeypatch):
  RecordingRatekeeper.instances.clear()
  RecordingPublisher.instances.clear()
  monkeypatch.setattr(sensord, "Ratekeeper", RecordingRatekeeper)
  monkeypatch.setattr(sensord.messaging, "PubMaster", RecordingPublisher)
  monkeypatch.setattr(sensord.messaging, "new_message", lambda _service, valid: SimpleNamespace(valid=valid))
  monkeypatch.setattr(sensord.cloudlog, "exception", lambda *_args, **_kwargs: None)


def test_invalid_iterations_are_paced_without_publishing(monkeypatch):
  setup_polling_fakes(monkeypatch)
  sensor = SequenceSensor([False, True])
  stop = StopAfterChecks(iterations=2)

  sensord.polling_loop(sensor, "accelerometer", stop)

  ratekeeper, = RecordingRatekeeper.instances
  publisher, = RecordingPublisher.instances
  assert sensor.events == 2
  assert ratekeeper.keep_time_calls == 2
  assert len(publisher.sent) == 1
  service, message = publisher.sent[0]
  assert service == "accelerometer"
  assert message.valid
  assert message.accelerometer == "event-2"


def test_sensor_read_errors_are_logged_and_still_paced(monkeypatch):
  setup_polling_fakes(monkeypatch)
  stop = StopAfterChecks(iterations=2)
  logged = []

  class ErrorThenValidSensor:
    def __init__(self):
      self.events = 0

    def get_event(self):
      self.events += 1
      if self.events == 1:
        raise OSError("offline fixture")
      return "event-2"

    def is_data_valid(self):
      return True

  monkeypatch.setattr(sensord.cloudlog, "exception", lambda message: logged.append(message))
  sensor = ErrorThenValidSensor()

  sensord.polling_loop(sensor, "accelerometer", stop)

  ratekeeper, = RecordingRatekeeper.instances
  publisher, = RecordingPublisher.instances
  assert ratekeeper.keep_time_calls == 2
  assert logged == ["Error in accelerometer polling loop"]
  assert len(publisher.sent) == 1
  assert publisher.sent[0][1].accelerometer == "event-2"
