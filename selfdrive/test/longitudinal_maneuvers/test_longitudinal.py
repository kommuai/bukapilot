#!/usr/bin/env python3
import os
import unittest

from common.params import Params
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE
from selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver

comma_maneuvers = [
  Maneuver(
    'steady state following a car at 20m/s, then lead decel to 0mph at 3+m/s^2',
    duration=40.,
    initial_speed=20.,
    lead_relevancy=True,
    initial_distance_lead=35.,
    speed_lead_values=[20., 20., 0.],
    prob_lead_values=[0., 1., 1.],
    cruise_values=[20., 20., 20.],
    breakpoints=[2., 2.01, 8.8],
  ),
  Maneuver(
    "approach stopped car at 20m/s",
    duration=30.,
    initial_speed=20.,
    lead_relevancy=True,
    initial_distance_lead=120.,
    speed_lead_values=[0.0, 0., 0.],
    prob_lead_values=[0.0, 0., 1.],
    cruise_values=[20., 20., 20.],
    breakpoints=[0.0, 2., 2.01],
  ),
  Maneuver(
    "approach slower cut-in car at 20m/s",
    duration=20.,
    initial_speed=20.,
    lead_relevancy=True,
    initial_distance_lead=50.,
    speed_lead_values=[15., 15.],
    breakpoints=[1., 11.],
    only_lead2=True,
  ),
  Maneuver(
    "stay stopped behind radar override lead",
    duration=20.,
    initial_speed=0.,
    lead_relevancy=True,
    initial_distance_lead=10.,
    speed_lead_values=[0., 0.],
    prob_lead_values=[0., 0.],
    breakpoints=[1., 11.],
    only_radar=True,
  ),
  Maneuver(
    "NaN recovery",
    duration=30.,
    initial_speed=15.,
    lead_relevancy=True,
    initial_distance_lead=60.,
    speed_lead_values=[0., 0., 0.0],
    breakpoints=[1., 1.01, 11.],
    cruise_values=[float("nan"), 15., 15.],
  ),
  # controls relies on planner commanding to move for stock-ACC resume spamming
  Maneuver(
    "resume from a stop",
    duration=20.,
    initial_speed=0.,
    lead_relevancy=True,
    initial_distance_lead=STOP_DISTANCE,
    speed_lead_values=[0., 0., 2.],
    breakpoints=[1., 10., 15.],
    ensure_start=True,
  ),
]

maneuvers = [
  Maneuver(
    "steady state converge to 90km/h",
    duration=30.,
    initial_speed=25.,
    lead_relevancy=True,
    initial_distance_lead=60.,
    speed_lead_values=[25., 25., 25.],
    breakpoints=[0., 15., 21.66],
    write_file=True,
  ),
  Maneuver(
    "steady state following a car at 72km/h, then lead decel to 0mph at -3m/s^2",
    duration=30.,
    initial_speed=20.,
    lead_relevancy=True,
    initial_distance_lead=20.,
    speed_lead_values=[20., 20., 0.],
    prob_lead_values=[0., 0.8, 1.],
    cruise_values=[20., 20., 20.],
    breakpoints=[0., 15., 21.66],
    write_file=False,
  ),
  Maneuver(
    "steady state following a car at 30km/h, then lead decel to 0mph at -3m/s^2",
    duration=30.,
    initial_speed=8.3,
    lead_relevancy=True,
    initial_distance_lead=10.,
    speed_lead_values=[8.3, 8.3, 0.],
    prob_lead_values=[0., 0.8, 1.],
    cruise_values=[8.3, 8.3, 8.3],
    breakpoints=[0., 10., 12.76],
    write_file=False,
  ),

  Maneuver(
    "approach stopped car at 100km/h, from 160m away. (usual highway to stop situation)",
    duration=30.,
    initial_speed=28.,
    lead_relevancy=True,
    initial_distance_lead=160.,
    speed_lead_values=[0.0, 0., 0.],
    prob_lead_values=[0.0, 0.8, 1.],
    cruise_values=[28., 28., 28.],
    breakpoints=[0.0, 2., 2.01],
  ),
  Maneuver(
    "approach stopped car at 70km/h, from 120m away. (usual approach to traffic light)",
    duration=30.,
    initial_speed=20.,
    lead_relevancy=True,
    initial_distance_lead=120.,
    speed_lead_values=[0.0, 0., 0.],
    prob_lead_values=[0.0, 0., 1.],
    cruise_values=[20., 20., 20.],
    breakpoints=[0.0, 2., 2.01],
    write_file=False,
  ),
]


class LongitudinalControl(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    os.environ['SIMULATION'] = "1"
    os.environ['SKIP_FW_QUERY'] = "1"
    os.environ['NO_CAN_TIMEOUT'] = "1"

    params = Params()
    params.clear_all()
    params.put_bool("Passive", bool(os.getenv("PASSIVE")))
    params.put_bool("OpenpilotEnabledToggle", True)

  # hack
  def test_longitudinal_setup(self):
    pass


def run_maneuver_worker(k):
  def run(self):
    man = maneuvers[k]
    print(man.title)
    valid, _ = man.evaluate()
    self.assertTrue(valid, msg=man.title)
  return run

for k in range(len(maneuvers)):
  setattr(LongitudinalControl, f"test_longitudinal_maneuvers_{k+1}",
          run_maneuver_worker(k))


if __name__ == "__main__":
  unittest.main(failfast=True)
