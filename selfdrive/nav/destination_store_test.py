import unittest

from openpilot.selfdrive.nav.destination_store import apply_stop_operation, write_stop_list


def stop(stop_id, name, latitude, longitude):
  return {"id": stop_id, "name": name, "place_name": name, "latitude": latitude, "longitude": longitude}


class DestinationStoreTest(unittest.TestCase):
  class Params:
    def __init__(self):
      self.values = {"NavStopListVersion": 0}

    def get(self, key):
      return self.values.get(key)

    def put(self, key, value):
      self.values[key] = value

    def remove(self, key):
      self.values.pop(key, None)

  def test_stop_metadata_uses_integer_param_values(self):
    params = self.Params()
    normalized = write_stop_list(params, [stop("a", "A", 3.1, 101.5)], index=0)

    self.assertEqual(len(normalized), 1)
    self.assertIsInstance(params.values["NavStopIndex"], int)
    self.assertIsInstance(params.values["NavStopListVersion"], int)
    self.assertEqual(params.values["NavStopIndex"], 0)
    self.assertEqual(params.values["NavStopListVersion"], 1)

  def test_repeated_destination_can_be_added_after_another_stop(self):
    first = stop("place-a", "A", 3.1, 101.5)
    middle = stop("place-b", "B", 3.2, 101.6)
    updated, ok, reason, changed = apply_stop_operation(
      [first, middle], "navAddStop", {"stop": first},
    )

    self.assertTrue(ok)
    self.assertEqual(reason, "added")
    self.assertTrue(changed)
    self.assertEqual([item["name"] for item in updated], ["A", "B", "A"])
    self.assertEqual(len({item["id"] for item in updated}), 3)

  def test_duplicate_of_current_tail_is_a_noop(self):
    first = stop("place-a", "A", 3.1, 101.5)
    updated, ok, reason, changed = apply_stop_operation(
      [first], "navAddStop", {"stop": stop("other-id", "A again", 3.1, 101.5)},
    )

    self.assertTrue(ok)
    self.assertEqual(reason, "already_present")
    self.assertFalse(changed)
    self.assertEqual(updated, [first])

  def test_reorder_rejects_adjacent_duplicate_destinations(self):
    stops = [
      stop("a-first", "A", 3.1, 101.5),
      stop("b", "B", 3.2, 101.6),
      stop("a-second", "A", 3.1, 101.5),
    ]
    updated, ok, reason, changed = apply_stop_operation(
      stops, "navReorderStops", {"stopIds": ["a-first", "a-second", "b"]},
    )

    self.assertFalse(ok)
    self.assertEqual(reason, "adjacent_duplicate")
    self.assertFalse(changed)
    self.assertEqual(updated, stops)

  def test_reorder_keeps_repeated_destinations_when_separated(self):
    stops = [
      stop("a-first", "A", 3.1, 101.5),
      stop("b", "B", 3.2, 101.6),
      stop("a-second", "A", 3.1, 101.5),
    ]
    updated, ok, reason, changed = apply_stop_operation(
      stops, "navReorderStops", {"stopIds": ["a-second", "b", "a-first"]},
    )

    self.assertTrue(ok)
    self.assertEqual(reason, "reordered")
    self.assertTrue(changed)
    self.assertEqual([item["id"] for item in updated], ["a-second", "b", "a-first"])


if __name__ == "__main__":
  unittest.main()
