# Re-export car platform types and helpers for opendbc (proton, byd, dnga) that import from openpilot.selfdrive.car
from dataclasses import dataclass

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import apply_std_steer_angle_limits


@dataclass
class AngleRateLimit:
  """Indexable as (speed_bp, angle_v) for lateral.apply_std_steer_angle_limits."""
  speed_bp: list
  angle_v: list

  def __getitem__(self, i):
    return (self.speed_bp, self.angle_v)[i]


def dbc_dict(pt: str, radar: str | None) -> DbcDict:
  if radar is None:
    return {Bus.pt: pt}
  return {Bus.pt: pt, Bus.radar: radar}


__all__ = [
  'AngleRateLimit',
  'CarSpecs',
  'DbcDict',
  'PlatformConfig',
  'Platforms',
  'apply_std_steer_angle_limits',
  'dbc_dict',
]
