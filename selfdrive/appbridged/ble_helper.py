#!/usr/bin/env python3
import logging
import threading
import os
from time import monotonic, sleep
from queue import SimpleQueue
from bluezero import adapter, peripheral
from openpilot.common.swaglog import cloudlog

# --- MALLOC SAFETY INITIALISATION ---
import dbus
import dbus.mainloop.glib

# Initialise threads to ensure the underlying C library uses locks, preventing
# heap corruption and subsequent malloc crashes during concurrent D-Bus calls.
try:
  dbus.mainloop.glib.threads_init()
except Exception:
  pass
# -------------------------------------

# bluezero is noisy at INFO and also uses bare print() in its advertisement
# callbacks. Silence its logger and suppress stdout during publish().
logging.getLogger("bluezero").setLevel(logging.WARNING)
logging.getLogger("bluezero.GATT").setLevel(logging.WARNING)

# BLE Nordic UART UUIDs
UART_SERVICE      = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
RX_CHARACTERISTIC = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'  # Write from phone
TX_CHARACTERISTIC = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'  # Notify to phone

CHUNK_TIMEOUT = 1.0  # seconds before dropping incomplete message

class BLEBridge:
  """Threaded BLE Nordic UART bridge with RX and TX."""
  def __init__(self, local_name=None):
    # Capture process identifier at start-up. If a fork occurs,
    # the connection becomes unsafe and must be terminated to avoid corruption.
    self._initial_pid = os.getpid()

    self.ad = list(adapter.Adapter.available())[0]
    self.dev = peripheral.Peripheral(self.ad.address, local_name=local_name, appearance=963)

    self.rx_queue = SimpleQueue()
    self.tx_char = None
    self._counters = {}
    self._tx_accepts_bytes = None

    # Add UART service
    self.dev.add_service(srv_id=1, uuid=UART_SERVICE, primary=True)

    # RX: phone -> device (write)
    self.dev.add_characteristic(
      srv_id=1, chr_id=1, uuid=RX_CHARACTERISTIC,
      value=[], notifying=False,
      flags=['write', 'write-without-response'],
      write_callback=self.on_write
    )

    # TX: device -> phone (notify)
    self.dev.add_characteristic(
      srv_id=1, chr_id=2, uuid=TX_CHARACTERISTIC,
      value=[], notifying=False,
      flags=['notify'],
      notify_callback=self.notify_state
    )

    self.dev.on_connect = self.on_connect
    self.dev.on_disconnect = self.on_disconnect
    self.connected = False

  def _check_fork(self):
    """Verifies that the process identifier remains unchanged."""
    if os.getpid() != self._initial_pid:
      cloudlog.error("BLEBridge: Process forked. Terminating connection to prevent memory corruption.")
      raise RuntimeError("Unsafe D-Bus usage detected after fork.")

  def on_connect(self, dev):
    self.connected = True
    print(f"BLE Connected: {dev.address}")

  def on_disconnect(self, adapter_addr, dev_addr):
    self.connected = False
    print(f"BLE Disconnected: {dev_addr}")

  def notify_state(self, notifying, characteristic):
    self.tx_char = characteristic if notifying else None
    self._tx_accepts_bytes = None

  def on_write(self, value, options):
    """Receive bytes from phone and store in queue."""
    self.rx_queue.put(bytes(value))

  def send(self, payload: bytes):
    """Send bytes to phone via BLE."""
    self._check_fork()
    if not (tx_char := self.tx_char):
      return

    if (accepts_bytes := self._tx_accepts_bytes) is False:
      tx_char.set_value(list(payload))
      return

    if accepts_bytes is True:
      tx_char.set_value(payload)
      return

    # Probe once, then cache capability for the fast path.
    try:
      tx_char.set_value(payload)
      self._tx_accepts_bytes = True
    except TypeError:
      self._tx_accepts_bytes = False
      tx_char.set_value(list(payload))

  def read(self):
    """Pop next received BLE packet if available."""
    return q.get_nowait() if not (q := self.rx_queue).empty() else None

  def start(self):
    """Start BLE peripheral loop."""
    self._check_fork()
    self.dev.publish()
    while True:
      sleep(0.1) # keep thread running

  def chunk_and_send(self, channel: int, payload: bytes, CHUNK_SIZE=240):
    """Split payload into BLE chunks and send."""
    if CHUNK_SIZE <= 0:
      return
    if not self.tx_char:
      return

    # Message ID cycles from 1 to 255
    cnts = self._counters
    msg_id = (cnts.get(channel, 0) % 255) + 1
    cnts[channel] = msg_id

    if not (payload_len := len(payload)):
      return

    if channel > 255 or (total_segments := (payload_len + CHUNK_SIZE - 1) // CHUNK_SIZE) > 255:
      raise ValueError("BLE chunk header overflow")

    view = memoryview(payload)
    header = bytearray(4)
    header[0] = channel
    header[1] = msg_id
    header[2] = total_segments
    send = self.send

    for seg_idx in range(total_segments):
      offset = seg_idx * CHUNK_SIZE
      header[3] = seg_idx
      send(header + view[offset:offset + CHUNK_SIZE])

class ChunkReceiver:
  """Assemble incoming BLE chunks into full messages."""
  def __init__(self, ble):
    self.ble = ble
    # { (channel, msg_id): [chunks_list, missing_count, last_time] }
    self.active_messages = {}
    self.completed_messages = SimpleQueue()
    self._next_cleanup_time = monotonic() + (CHUNK_TIMEOUT * 0.5)
    threading.Thread(target=self._receive_loop, daemon=True).start()

  def _receive_loop(self):
    """Continuously read BLE packets, assemble chunks, drop timed-out messages."""
    while True:
      processed_packet = False
      if connected := self.ble.connected:
        ble_read = self.ble.read
        active_messages = self.active_messages
        completed_messages_put = self.completed_messages.put
        while (pkt := ble_read()) is not None:
          processed_packet = True
          if len(pkt) < 4:
            continue

          channel = pkt[0]
          msg_id = pkt[1]
          total_segments = pkt[2]
          seg_idx = pkt[3]
          if total_segments == 0 or seg_idx >= total_segments:
            continue

          chunk = pkt[4:]
          key = (channel << 8) | msg_id
          now = monotonic()  # assign once per packet
          if (entry := active_messages.get(key)) is None:
            chunks_list = [None] * total_segments
            entry = [chunks_list, total_segments, now]
            active_messages[key] = entry
          else:
            chunks_list, _, _ = entry
            if len(chunks_list) != total_segments:
              chunks_list = [None] * total_segments
              entry[0] = chunks_list
              entry[1] = total_segments

          chunks_list, missing_count, _ = entry
          if chunks_list[seg_idx] is None:
            missing_count -= 1
          chunks_list[seg_idx] = chunk
          entry[1] = missing_count
          entry[2] = now

          if missing_count == 0:
            completed_messages_put((channel, b"".join(chunks_list)))
            del active_messages[key]

          if now >= self._next_cleanup_time:
            self._cleanup_stale_messages(now)
            self._next_cleanup_time = now + (CHUNK_TIMEOUT * 0.5)

      if not processed_packet:
        sleep(0.002 if connected else 0.05) # Small sleep only when no packets are ready

  def _cleanup_stale_messages(self, now):
    if stale_keys := [k for k, (_, _, last_time) in self.active_messages.items() if now - last_time > CHUNK_TIMEOUT]:
      for key in stale_keys:
        cloudlog.info("Dropping incomplete BLE message")
        del self.active_messages[key]

  def get_message(self):
    """Return the next completed message if available."""
    return q.get_nowait() if not (q := self.completed_messages).empty() else None
