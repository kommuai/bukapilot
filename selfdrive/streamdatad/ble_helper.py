#!/usr/bin/env python3
import threading
from time import monotonic, sleep
from queue import SimpleQueue
from bluezero import adapter, peripheral

# BLE Nordic UART UUIDs
UART_SERVICE      = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
RX_CHARACTERISTIC = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'  # Write from phone
TX_CHARACTERISTIC = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'  # Notify to phone

CHUNK_TIMEOUT = 1.0  # seconds before dropping incomplete message

class BLEBridge:
  """Threaded BLE Nordic UART bridge with RX and TX."""
  def __init__(self, local_name=None):
    self.ad = list(adapter.Adapter.available())[0]
    self.dev = peripheral.Peripheral(self.ad.address, local_name=local_name, appearance=963)

    self.rx_queue = SimpleQueue()
    self.tx_char = None

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

  def on_connect(self, dev):
    self.connected = True
    print(f"BLE Connected: {dev.address}")

  def on_disconnect(self, adapter_addr, dev_addr):
    self.connected = False
    print(f"BLE Disconnected: {dev_addr}")

  def notify_state(self, notifying, characteristic):
    self.tx_char = characteristic if notifying else None

  def on_write(self, value, options):
    """Receive bytes from phone and store in queue."""
    self.rx_queue.put(bytes(value))

  def send(self, payload: bytes):
    """Send bytes to phone via BLE."""
    if self.tx_char:
      self.tx_char.set_value(list(payload))

  def read(self):
    """Pop next received BLE packet if available."""
    if not self.rx_queue.empty():
      return self.rx_queue.get()
    return None

  def start(self):
    """Start BLE peripheral loop."""
    self.dev.publish()
    while True:
      sleep(0.1) # keep thread running

  def chunk_and_send(self, channel: int, payload: bytes, CHUNK_SIZE=240):
    """Split payload into BLE chunks and send."""
    # Track message IDs per channel
    cnts = getattr(self, "_counters", setattr(self, "_counters", {}) or self._counters)
    cnts[channel] = msg_id = cnts.get(channel, 0) % 255 + 1 # Message ID cycles from 1 to 255
    view = memoryview(payload)
    for seg_idx in range(total_segments := -(-len(payload) // CHUNK_SIZE)):
      offset = seg_idx * CHUNK_SIZE
      self.send(bytes([channel, msg_id, total_segments, seg_idx]) + view[offset:offset + CHUNK_SIZE])

class ChunkReceiver:
  """Assemble incoming BLE chunks into full messages."""
  def __init__(self, ble):
    self.ble = ble
    self.lock = threading.Lock()
    self.active_messages = {}  # { (channel, msg_id): [received_chunks, total_segments, last_time] }
    self.completed_messages = SimpleQueue()
    threading.Thread(target=self._receive_loop, daemon=True).start()

  def _receive_loop(self):
    """Continuously read BLE packets, assemble chunks, drop timed-out messages."""
    while True:
      processed_packet = False
      if self.ble.connected:
        while pkt := self.ble.read():
          processed_packet = True
          channel, msg_id, total_segments, seg_idx = pkt[:4]
          chunk = pkt[4:]
          key = (channel, msg_id)
          now = monotonic() # assign once per packet
          with self.lock:
            entry = self.active_messages.get(key)
            if entry is None:
              entry = [[None]*total_segments, total_segments, now]
              self.active_messages[key] = entry
            chunks_list, total, _ = entry
            chunks_list[seg_idx] = chunk
            entry[2] = now

            if None not in chunks_list:
              msg = b''.join(chunks_list)
              self.completed_messages.put((channel, msg))
              del self.active_messages[key]

            for key2, (chunks, total, last_time) in list(self.active_messages.items()):
              if now - last_time > CHUNK_TIMEOUT: # Drop timed-out messages
                print("Dropping incomplete message")
                del self.active_messages[key2]

      if not processed_packet:
        sleep(0.01) # Small sleep only if no packets to process

  def get_message(self):
    """Return the next completed message if available."""
    return q.get() if not (q := self.completed_messages).empty() else None
