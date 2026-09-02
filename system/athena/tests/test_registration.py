from openpilot.common.params import Params
from openpilot.system.athena.registration import register, is_registered_device, UNREGISTERED_DONGLE_ID


class TestRegistration:

  def setup_method(self):
    self.params = Params()
    self.params.remove("DongleId")
    self.params.remove("IMEI")
    self.params.remove("HardwareSerial")

  def test_cached_dongle_skips_registration(self, mocker):
    dongle = "DONGLE_ID_123"
    self.params.put("DongleId", dongle)
    m = mocker.patch("openpilot.system.athena.registration.runescapej.register_user", autospec=True)
    assert register() == dongle
    m.assert_not_called()

  def test_register_success(self, mocker):
    dongle = "DONGLE_ID_123"
    self.params.put("DongleId", UNREGISTERED_DONGLE_ID)
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_serial", return_value="SERIAL123")
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_imei", side_effect=lambda slot: "IMEI0" if slot == 0 else "IMEI1")
    m = mocker.patch("openpilot.system.athena.registration.runescapej.register_user", return_value=dongle, autospec=True)

    assert register() == dongle
    m.assert_called_once_with("IMEI1", "SERIAL123")
    assert self.params.get("DongleId") == dongle
    assert self.params.get("IMEI") == "IMEI0"
    assert self.params.get("HardwareSerial") == "SERIAL123"

  def test_retry_then_success(self, mocker):
    dongle = "DONGLE_ID_123"
    self.params.put("DongleId", UNREGISTERED_DONGLE_ID)
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_serial", return_value="SERIAL123")
    mocker.patch("openpilot.system.athena.registration.HARDWARE.get_imei", side_effect=lambda slot: "IMEI0" if slot == 0 else "IMEI1")
    mocker.patch("openpilot.system.athena.registration.time.sleep")
    m = mocker.patch("openpilot.system.athena.registration.runescapej.register_user",
                     side_effect=[Exception("temporary"), dongle],
                     autospec=True)

    assert register() == dongle
    assert m.call_count == 2
    assert self.params.get("DongleId") == dongle
    assert self.params.get("IMEI") == "IMEI0"
    assert self.params.get("HardwareSerial") == "SERIAL123"

  def test_is_registered_device_helper(self):
    self.params.put("DongleId", UNREGISTERED_DONGLE_ID)
    assert not is_registered_device()
    self.params.put("DongleId", "DONGLE_ID_123")
    assert is_registered_device()
