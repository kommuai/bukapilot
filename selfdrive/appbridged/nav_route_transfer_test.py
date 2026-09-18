import base64
import unittest

from openpilot.selfdrive.appbridged import nav_route_transfer
from openpilot.selfdrive.appbridged.nav_route_transfer import NavRouteTransfer


def transfer_message(**overrides):
  message = {
    "requestId": "nav-session:4:123",
    "transferId": "transfer-1",
    "sessionId": "nav-session",
    "routeGeneration": 4,
    "destKey": "3.13500|101.72300",
    "totalBytes": 9,
    "partCount": 3,
  }
  message.update(overrides)
  return message


class NavRouteTransferTest(unittest.TestCase):
  def test_out_of_order_and_duplicate_parts_reconstruct_exact_bytes(self):
    transfer = NavRouteTransfer()
    self.assertTrue(transfer.start(transfer_message())["ok"])
    self.assertEqual(transfer.add_part(transfer_message(partIndex=2, routePart=b"ghi"))["receivedParts"], 1)
    self.assertTrue(transfer.add_part(transfer_message(partIndex=2, routePart=b"ghi"))["duplicate"])
    transfer.add_part(transfer_message(partIndex=0, routePart=b"abc"))
    result = transfer.add_part(transfer_message(partIndex=1, routePart=b"def"))
    self.assertTrue(result["complete"])
    self.assertEqual(result["receivedParts"], 3)
    self.assertEqual(result["routeBytes"], b"abcdefghi")
    self.assertFalse(transfer.progress()["ok"])

  def test_base64_parts_reconstruct_exact_bytes(self):
    transfer = NavRouteTransfer()
    transfer.start(transfer_message())
    result = None
    for part_index, payload in enumerate((b"abc", b"def", b"ghi")):
      result = transfer.add_part(transfer_message(
        partIndex=part_index,
        routePartBase64=base64.b64encode(payload).decode("ascii"),
      ))
    self.assertTrue(result["complete"])
    self.assertEqual(result["routeBytes"], b"abcdefghi")

  def test_integer_array_parts_reconstruct_exact_bytes(self):
    transfer = NavRouteTransfer()
    transfer.start(transfer_message())
    result = None
    for part_index, payload in enumerate((b"abc", b"def", b"ghi")):
      result = transfer.add_part(transfer_message(partIndex=part_index, routePart=list(payload)))
    self.assertTrue(result["complete"])
    self.assertEqual(result["routeBytes"], b"abcdefghi")

  def test_invalid_integer_array_discards_transfer(self):
    for payload in ([True], [-1], [256], [1.5], []):
      transfer = NavRouteTransfer()
      transfer.start(transfer_message())
      result = transfer.add_part(transfer_message(partIndex=0, routePart=payload))
      self.assertEqual(result["reason"], "invalid_part_payload")
      self.assertFalse(transfer.progress()["ok"])

  def test_invalid_base64_discards_transfer(self):
    transfer = NavRouteTransfer()
    transfer.start(transfer_message())
    result = transfer.add_part(transfer_message(partIndex=0, routePartBase64="%%%"))
    self.assertEqual(result["reason"], "invalid_part_encoding")
    self.assertFalse(transfer.progress()["ok"])

  def test_part_identity_rejects_boolean_generation(self):
    transfer = NavRouteTransfer()
    transfer.start(transfer_message(routeGeneration=1))
    result = transfer.add_part(transfer_message(routeGeneration=True, partIndex=0, routePart=b"abc"))
    self.assertEqual(result["reason"], "part_metadata_mismatch")
    self.assertFalse(transfer.progress()["ok"])

  def test_stale_transfer_part_does_not_discard_current_transfer(self):
    transfer = NavRouteTransfer()
    transfer.start(transfer_message())
    stale = transfer.add_part(transfer_message(transferId="older", partIndex=0, routePart=b"abc"))
    self.assertEqual(stale["reason"], "stale_transfer")
    self.assertTrue(transfer.add_part(transfer_message(partIndex=0, routePart=b"abc"))["ok"])

  def test_conflicting_duplicate_discards_partial_route(self):
    transfer = NavRouteTransfer()
    transfer.start(transfer_message())
    transfer.add_part(transfer_message(partIndex=0, routePart=b"abc"))
    result = transfer.add_part(transfer_message(partIndex=0, routePart=b"xyz"))
    self.assertEqual(result["reason"], "conflicting_duplicate")
    self.assertFalse(transfer.progress()["ok"])

  def test_large_route_and_more_than_64_parts_reconstruct(self):
    total_bytes = 120 * 1024
    part_count = 300
    route = bytes(index % 256 for index in range(total_bytes))
    transfer = NavRouteTransfer()
    self.assertTrue(transfer.start(transfer_message(totalBytes=total_bytes, partCount=part_count))["ok"])
    result = None
    for part_index in range(part_count):
      start = total_bytes * part_index // part_count
      end = total_bytes * (part_index + 1) // part_count
      result = transfer.add_part(transfer_message(
        totalBytes=total_bytes, partCount=part_count, partIndex=part_index, routePart=route[start:end],
      ))
    self.assertTrue(result["complete"])
    self.assertEqual(result["routeBytes"], route)

  def test_idle_partial_transfer_expires_from_last_part_and_releases_buffer(self):
    transfer = NavRouteTransfer()
    original_monotonic = nav_route_transfer.monotonic
    times = iter((100.0, 115.0))
    nav_route_transfer.monotonic = lambda: next(times)
    try:
      self.assertTrue(transfer.start(transfer_message(totalBytes=60_000, partCount=2))["ok"])
      part = transfer.add_part(transfer_message(
        totalBytes=60_000, partCount=2, partIndex=0, routePart=b"x" * 30_000,
      ))
    finally:
      nav_route_transfer.monotonic = original_monotonic
    self.assertTrue(part["ok"])
    last_part_at = part["updatedAt"]
    self.assertIsNone(transfer.expire(last_part_at + nav_route_transfer.ROUTE_TRANSFER_IDLE_TIMEOUT_SECONDS - 0.01))
    expired = transfer.expire(last_part_at + nav_route_transfer.ROUTE_TRANSFER_IDLE_TIMEOUT_SECONDS)
    self.assertEqual(expired["receivedParts"], 1)
    self.assertEqual(expired["receivedBytes"], 30_000)
    self.assertFalse(transfer.progress()["ok"])

  def test_slow_progress_remains_active_until_idle_timeout(self):
    transfer = NavRouteTransfer()
    original_monotonic = nav_route_transfer.monotonic
    times = iter((0.0, 15.0))
    nav_route_transfer.monotonic = lambda: next(times)
    try:
      self.assertTrue(transfer.start(transfer_message())["ok"])
      result = transfer.add_part(transfer_message(partIndex=0, routePart=b"a"))
    finally:
      nav_route_transfer.monotonic = original_monotonic
    self.assertTrue(result["ok"])
    self.assertEqual(result["receivedParts"], 1)
    self.assertTrue(transfer.progress()["ok"])


  def test_invalid_limits_and_part_metadata_are_rejected(self):
    transfer = NavRouteTransfer()
    invalid = transfer.start(transfer_message(totalBytes=3, partCount=4))
    self.assertEqual(invalid["reason"], "invalid_limits")
    transfer.start(transfer_message())
    result = transfer.add_part(transfer_message(partCount=2, partIndex=0, routePart=b"abc"))
    self.assertEqual(result["reason"], "part_metadata_mismatch")
    self.assertFalse(transfer.progress()["ok"])


if __name__ == "__main__":
  unittest.main()
