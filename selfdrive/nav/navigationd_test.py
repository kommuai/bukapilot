import unittest

from openpilot.selfdrive.nav.navigationd_policy import (
  has_persistent_nav_stops, route_failure_retry_ready,
  same_route_reroute_blocked, nav_location_sources_match,
)


class Params:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    value = self.values.get(key)
    return value is True or value in (b'1', b'true', '1', 'true')


def stop(stop_id='stop-a'):
  return {
    'id': stop_id, 'name': 'A', 'place_name': 'A',
    'latitude': 3.1, 'longitude': 101.5,
  }


class NavigationRecoveryTest(unittest.TestCase):
  def test_route_source_must_match_and_be_supported(self):
    self.assertTrue(nav_location_sources_match('phone', 'phone'))
    self.assertTrue(nav_location_sources_match('combined', 'combined'))
    self.assertFalse(nav_location_sources_match('device', 'phone'))
    self.assertFalse(nav_location_sources_match(None, None))
    self.assertFalse(nav_location_sources_match('invalid', 'invalid'))

  def test_navigationd_starts_for_persisted_stops(self):
    params = Params({'NavStopList': [stop()]})

    self.assertTrue(has_persistent_nav_stops(params))
    self.assertFalse(has_persistent_nav_stops(Params({'NavStopList': '[]'})))

  def test_route_failure_retries_after_cooldown(self):
    self.assertFalse(route_failure_retry_ready(129.0, 130.0, 3))
    self.assertTrue(route_failure_retry_ready(130.0, 130.0, 3))
    self.assertFalse(route_failure_retry_ready(130.0, 130.0, 2))

  def test_same_route_guard_allows_reroute_after_movement(self):
    accepted = type('Position', (), {'distance_to': lambda self, other: 0.0})()
    current = type('Position', (), {'distance_to': lambda self, other: 0.0})()

    self.assertTrue(same_route_reroute_blocked(100.0, 200.0, current, accepted))

    moved = type('Position', (), {'distance_to': lambda self, other: 50.0})()
    self.assertFalse(same_route_reroute_blocked(100.0, 200.0, moved, accepted))


if __name__ == '__main__':
  unittest.main()
