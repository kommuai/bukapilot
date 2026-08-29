#!/usr/bin/env python3
"""Verify KA2's manual road LSC table against comma's source reference."""

import argparse
import json
import re
from pathlib import Path


def values_between(text, start_pattern):
  match = re.search(start_pattern + r"\s*(?:\[[^]]+\])?\s*=\s*\{(.*?)\};", text, re.DOTALL)
  if not match:
    raise ValueError(f"missing {start_pattern}")
  return [int(value, 0) for value in re.findall(r"0x[0-9a-fA-F]+|\d+", match.group(1))]


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("comma_sensor_source", type=Path)
  parser.add_argument("--ka2-table", type=Path,
                      default=Path("system/camerad/rk/rk_road_lsc_tables.h"))
  parser.add_argument("--ka2-calibration", type=Path,
                      default=Path("system/camerad/rk/calib_embed/ox03c10_D2V11K_9420.json"))
  args = parser.parse_args()

  comma_words = values_between(args.comma_sensor_source.read_text(), r"vignetting_lut")
  if len(comma_words) != 13 * 17:
    raise ValueError(f"expected 221 comma LSC words, got {len(comma_words)}")
  low = [word & 0x1fff for word in comma_words]
  high = [(word >> 13) & 0x1fff for word in comma_words]
  if low != high:
    raise ValueError("comma vignetting_lut does not contain matching 13-bit gain fields")

  expected = []
  for y in range(17):
    position = y * 12 / 16
    y0 = int(position)
    y1 = min(y0 + 1, 12)
    fraction = position - y0
    for x in range(17):
      expected.append(round(low[y0 * 17 + x] * (1 - fraction) + low[y1 * 17 + x] * fraction))

  table = args.ka2_table.read_text()
  actual = values_between(table, r"kCommaRoadGainQ10")
  if actual != expected:
    raise ValueError("KA2 road mesh differs from the comma-derived 13-bit reference")
  if len(actual) != 17 * 17 or not all(1024 <= value <= 8191 for value in actual):
    raise ValueError("KA2 road mesh is outside the RKISP3 17x17 13-bit Q10 domain")

  sectors_x = values_between(table, r"kSectorSizeX")
  sectors_y = values_between(table, r"kSectorSizeY")
  if sectors_x != [120] * 16 or sectors_y != [75] * 16:
    raise ValueError("KA2 road mesh sectors must exactly cover 1920x1200")

  calibration = json.loads(args.ka2_calibration.read_text())
  road_lsc = calibration["main_scene"][0]["sub_scene"][0]["scene_isp30"]["lsc_v2"]
  if not road_lsc["common"]["enable"]:
    raise ValueError("KA2 road calibration leaves ALSC disabled")
  disabled = calibration["sys_static_cfg"]["algoSwitch"]["disable_algos"]
  if "DISABLE_ALSC" in disabled:
    raise ValueError("KA2 road calibration excludes the ALSC algorithm")
  for table_config in road_lsc["tbl"]["tableAll"]:
    for channel in ("lsc_samples_red", "lsc_samples_greenR",
                    "lsc_samples_greenB", "lsc_samples_blue"):
      if table_config[channel]["uCoeff"] != actual:
        raise ValueError(f"KA2 static {channel} mesh differs from the manual road mesh")

  print(f"PASS comma 13x17 -> KA2 17x17: range={min(actual)}..{max(actual)} "
        f"center={actual[8 * 17 + 8]} sectors={sum(sectors_x)}x{sum(sectors_y)} "
        f"static_meshes={len(road_lsc['tbl']['tableAll']) * 4}")


if __name__ == "__main__":
  main()
