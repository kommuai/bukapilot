import unittest
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.car.byd.radar_interface import RadarInterface, SEAL6_RADAR_TIMEOUT_NS
from opendbc.car.structs import RadarData


def make_interface():
  ri = object.__new__(RadarInterface)
  ri.seal6_radar = True
  ri.radar_slot_count = 2
  ri.radar_msg_count = 1
  ri.updated_messages = set()
  ri.track_id = 100
  ri.slot_track_id = {}
  ri.pts = {}
  ri.valid_cnt = {i: 0 for i in range(ri.radar_slot_count)}
  ri.miss_cnt = {i: 0 for i in range(ri.radar_slot_count)}
  ri.track_history = {}
  ri.MAX_HISTORY_FRAMES = 10
  ri.seal6_prev_drel = {}
  ri.seal6_last_trigger_monotonic_ns = None
  ri.seal6_last_trigger_event_ns = None
  ri.seal6_freshness_expired = False
  ri.speed_cp = None
  ri.rcp = None
  ri.trigger_msg = 0x670
  return ri


class TestSeal6RadarInterface(unittest.TestCase):
  def test_stable_co_moving_lead_is_not_rejected(self):
    ri = make_interface()
    for _ in range(10):
      self.assertFalse(ri._seal6_low_speed_untrusted(0, 12.0, 0.1, 0.0, 2.0, 12.0))

  def test_low_speed_range_snap_is_rejected(self):
    ri = make_interface()
    self.assertTrue(ri._seal6_low_speed_untrusted(0, 14.0, 0.1, 0.0, 2.0, 12.0))

  def test_seal6_uses_record_valid_flags_and_dbc_scaled_yrel(self):
    ri = make_interface()
    msg = {
      "VALID": 1,
      "VALID_2": 0,
      "DREL": 10.0,
      "YREL": -0.60,
      "VREL": 0.0,
      "AREL": 0.0,
      "DREL_2": 12.0,
      "YREL_2": 0.30,
      "VREL_2": 0.0,
      "AREL_2": 0.0,
    }

    _, record1_measured, _, y_rel, _, _ = ri._read_track(msg, 0.0, 0.0, record_index=0)
    _, record2_measured, _, y_rel_2, _, _ = ri._read_track(msg, 0.0, 0.0, record_index=1)

    self.assertTrue(record1_measured)
    self.assertFalse(record2_measured)
    self.assertAlmostEqual(y_rel, -0.60)
    self.assertAlmostEqual(y_rel_2, 0.30)

    msg["VALID"] = 0
    msg["VALID_2"] = 1
    self.assertFalse(ri._read_track(msg, 0.0, 0.0, record_index=0)[1])
    self.assertTrue(ri._read_track(msg, 0.0, 0.0, record_index=1)[1])

  def test_closer_side_return_dropped_if_confirmed_centered_radar_target_exists(self):
    ri = make_interface()
    ri.pts = {
      0: SimpleNamespace(trackId=10, dRel=3.75, yRel=-0.60, vRel=0.0, measured=True),
      1: SimpleNamespace(trackId=11, dRel=14.8, yRel=-0.04, vRel=0.3, measured=True),
    }
    ri.valid_cnt = {0: 10, 1: 10}
    ri.slot_track_id = {0: 10, 1: 11}

    ri._seal6_suppress_low_speed_side_returns(0.0)

    self.assertNotIn(0, ri.pts)
    self.assertIn(1, ri.pts)

  def test_radar_only_side_return_remains_when_no_centered_competitor_exists(self):
    ri = make_interface()
    ri.pts = {0: SimpleNamespace(trackId=10, dRel=3.75, yRel=-0.60, vRel=0.0, measured=True)}
    ri.valid_cnt[0] = 10
    ri.slot_track_id[0] = 10

    ri._seal6_suppress_low_speed_side_returns(0.0)

    self.assertIn(0, ri.pts)

  def test_unconfirmed_centered_return_does_not_suppress_side_return(self):
    ri = make_interface()
    ri.pts = {
      0: SimpleNamespace(trackId=10, dRel=3.75, yRel=-0.60, vRel=0.0, measured=True),
      1: SimpleNamespace(trackId=11, dRel=14.8, yRel=-0.04, vRel=0.3, measured=True),
    }
    ri.valid_cnt = {0: 10, 1: 9}

    ri._seal6_suppress_low_speed_side_returns(0.0)

    self.assertIn(0, ri.pts)

  def test_side_return_filter_is_inactive_above_low_speed(self):
    ri = make_interface()
    ri.pts = {
      0: SimpleNamespace(trackId=10, dRel=3.75, yRel=-0.60, vRel=0.0, measured=True),
      1: SimpleNamespace(trackId=11, dRel=14.8, yRel=-0.04, vRel=0.3, measured=True),
    }
    ri.valid_cnt = {0: 10, 1: 10}

    ri._seal6_suppress_low_speed_side_returns(4.0)

    self.assertEqual(len(ri.pts), 2)

  def test_software_track_id_follows_object_to_new_slot(self):
    ri = make_interface()
    ri.pts = {1: SimpleNamespace(trackId=99, dRel=11.9, yRel=0.2, vRel=-2.0)}

    ri._seal6_reassociate_track_ids({0: (7, 12.0, 0.2, -2.0)})

    self.assertEqual(ri.pts[1].trackId, 7)
    self.assertEqual(ri.slot_track_id, {1: 7})
    self.assertNotIn(7, ri.track_history)

  def test_recycled_slot_gets_new_id_when_motion_does_not_match(self):
    ri = make_interface()
    ri.track_id = 20
    ri.pts = {0: SimpleNamespace(trackId=8, dRel=30.0, yRel=0.0, vRel=0.0)}

    ri._seal6_reassociate_track_ids({0: (8, 10.0, 0.0, 0.0)})

    self.assertEqual(ri.pts[0].trackId, 20)
    self.assertEqual(ri.slot_track_id[0], 20)
    self.assertIn(8, ri.track_history)

  def test_freshness_gap_returns_empty_radar_data_once(self):
    ri = make_interface()
    ri.radar_slot_count = 1
    ri.valid_cnt = {0: 1}
    ri.miss_cnt = {0: 0}
    ri.slot_track_id = {0: 5}
    point = RadarData.RadarPoint()
    point.trackId = 5
    point.dRel = 10.0
    point.yRel = 0.0
    point.vRel = 0.0
    ri.pts = {0: point}
    ri.seal6_last_trigger_monotonic_ns = 1_000_000_000
    ri.seal6_last_trigger_event_ns = 10_000_000_000

    class FakeParser:
      _last_update_nanos = 0

      def update(self, _can_strings):
        return set()

    ri.rcp = FakeParser()
    with patch("opendbc.car.byd.radar_interface.time.monotonic_ns", return_value=1_000_000_001 + SEAL6_RADAR_TIMEOUT_NS):
      expired = ri.update([])
    with patch("opendbc.car.byd.radar_interface.time.monotonic_ns", return_value=1_000_000_002 + SEAL6_RADAR_TIMEOUT_NS):
      no_repeat = ri.update([])

    self.assertIsNotNone(expired)
    self.assertEqual(len(expired.points), 0)
    self.assertTrue(expired.errors.canError)
    self.assertIsNone(no_repeat)
    self.assertEqual(ri.pts, {})

  def test_replay_uses_log_clock_instead_of_file_read_wall_time(self):
    ri = make_interface()
    ri.seal6_last_trigger_monotonic_ns = 1_000_000_000
    ri.seal6_last_trigger_event_ns = 10_000_000_000

    still_fresh = ri._seal6_check_freshness(
      1_000_000_000 + 10 * SEAL6_RADAR_TIMEOUT_NS,
      10_000_000_000 + 100_000_000,
      False,
      None,
    )
    self.assertFalse(still_fresh)

    log_gap = ri._seal6_check_freshness(
      1_000_000_000 + 11 * SEAL6_RADAR_TIMEOUT_NS,
      10_000_000_000 + SEAL6_RADAR_TIMEOUT_NS + 1,
      False,
      None,
    )
    self.assertTrue(log_gap)

  def test_event_clock_rewind_clears_tracks(self):
    ri = make_interface()
    point = RadarData.RadarPoint()
    point.trackId = 5
    point.dRel = 10.0
    ri.pts = {0: point}
    ri.slot_track_id = {0: 5}
    ri.valid_cnt[0] = 8
    ri.seal6_last_trigger_monotonic_ns = 1_000_000_000
    ri.seal6_last_trigger_event_ns = 10_000_000_000

    rewound = ri._seal6_check_freshness(
      1_000_000_001,
      9_000_000_000,
      False,
      None,
    )

    self.assertTrue(rewound)
    self.assertEqual(ri.pts, {})
    self.assertEqual(ri.slot_track_id, {})
    self.assertTrue(ri.seal6_freshness_expired)


if __name__ == "__main__":
  unittest.main()
