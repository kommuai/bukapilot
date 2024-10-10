#!/usr/bin/env python3
import unittest

from opendbc.can.parser import CANParser
from opendbc.can.packer import CANPacker
from opendbc.can.tests.test_packer_parser import can_list_to_can_capnp


class TestCanChecksums(unittest.TestCase):

  def test_honda_checksum(self):
    """Test checksums for Honda standard and extended CAN ids"""
    dbc_file = "byd_general_pt"
    msgs = [("LKAS_HUD_ADAS", 0)]
    parser = CANParser(dbc_file, msgs, 0)
    packer = CANPacker(dbc_file)

    values = {
      'SET_ME_XFF': 255,
      'SET_ME_X5F': 95,
      'SET_ME_1_2': 1,
      'SETTINGS': 12,
      'HMA': 2,
      'COUNTER': 11,
    }

    # known correct checksums according to the above values
    counters = [11, 6, 1, 8]
    answers = [226, 50, 130, 18]

    for counter, answer in zip(counters, answers):
      values["COUNTER"] = counter
      msgs = [
        packer.make_can_msg("LKAS_HUD_ADAS", 0, values),
      ]
      can_strings = [can_list_to_can_capnp(msgs), ]
      parser.update_strings(can_strings)

      self.assertEqual(parser.vl['LKAS_HUD_ADAS']['CHECKSUM'], answer)


if __name__ == "__main__":
  unittest.main()
