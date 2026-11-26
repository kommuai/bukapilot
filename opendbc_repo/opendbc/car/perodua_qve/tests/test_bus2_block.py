#!/usr/bin/env python3
import unittest

from opendbc.car.perodua_qve.values import (
  BUS2_ALWAYS_FORWARD,
  BUS2_FORWARD_RANGE,
  BUS2_POOL,
  qve_bus2_split_addrs,
)


class TestQveBus2Block(unittest.TestCase):
  def test_allow_0x50_and_0x80_to_0x88(self):
    blocked, forwarded = qve_bus2_split_addrs()
    self.assertEqual(forwarded, list(BUS2_FORWARD_RANGE))
    self.assertEqual(BUS2_FORWARD_RANGE, tuple(range(0x80, 0x89)))
    self.assertEqual(blocked, [a for a in BUS2_POOL if a not in BUS2_FORWARD_RANGE])
    self.assertNotIn(BUS2_ALWAYS_FORWARD, BUS2_POOL)
    self.assertNotIn(BUS2_ALWAYS_FORWARD, blocked)
    for addr in BUS2_FORWARD_RANGE:
      self.assertIn(addr, BUS2_POOL)
      self.assertNotIn(addr, blocked)


if __name__ == "__main__":
  unittest.main()
