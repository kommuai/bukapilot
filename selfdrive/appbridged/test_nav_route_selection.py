"""Focused pure-contract tests executed on the KA2 checkout."""
from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
import math
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve()
while not (ROOT / "selfdrive/appbridged/appbridged.py").is_file() and ROOT != ROOT.parent:
  ROOT = ROOT.parent
APPBRIDGED = ROOT / "selfdrive/appbridged/appbridged.py"
TRANSFER = ROOT / "selfdrive/appbridged/nav_route_transfer.py"
MAX_BYTES = 1024 * 1024
MAX_PARTS = 255
MAX_CANDIDATES = 16
MAX_SET_BYTES = 4 * 1024 * 1024


def extracted_helpers():
  tree = ast.parse(APPBRIDGED.read_text(encoding="utf-8"))
  wanted = {"_nav_validate_candidate_manifest", "_nav_validate_banner_instructions", "_nav_parse_route_payload"}
  nodes = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in wanted]
  if {node.name for node in nodes} != wanted:
    raise AssertionError("candidate validation/parser helpers not found")
  module = ast.Module(body=nodes, type_ignores=[])
  namespace = {
    "hashlib": hashlib, "json": json, "math": math, "re": re,
    "NAV_ROUTE_SELECTION_MAX_BYTES": MAX_BYTES,
    "NAV_ROUTE_SELECTION_MAX_CANDIDATES": MAX_CANDIDATES,
    "NAV_ROUTE_SELECTION_MAX_SET_BYTES": MAX_SET_BYTES,
  }
  exec(compile(module, str(APPBRIDGED), "exec"), namespace)
  return namespace


def route_bytes(route_id="set-1:1", padding=""):
  payload = {
    "schemaVersion": 1, "routeId": route_id,
    "geometry": [{"latitude": 3.1, "longitude": 101.5}, {"latitude": 3.2, "longitude": 101.6}],
    "steps": [{"maneuver": "turn", "instruction": "Turn left", "distance": 100, "duration": 10,
               "location": {"latitude": 3.2, "longitude": 101.6}}],
    "speechInstructions": [{"announcement": "Turn left", "distanceAlongGeometry": 20}],
  }
  if padding:
    payload["padding"] = padding
  return json.dumps(payload, separators=(",", ":")).encode("utf-8")


class RouteManifestAndDigestTests(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.helpers = extracted_helpers()

  def test_candidate_count_byte_length_manifest_and_aggregate_bounds(self):
    validate = self.helpers["_nav_validate_candidate_manifest"]
    digest = "a" * 64
    manifest = [{"routeId": f"set-1:{i}", "payloadSha256": digest, "payloadBytes": 256 * 1024} for i in range(16)]
    self.assertTrue(validate("set-1", "set-1:0", manifest[:1]))
    self.assertTrue(validate("set-1", "set-1:0", manifest))
    over_aggregate = list(manifest)
    over_aggregate[-1] = {**over_aggregate[-1], "payloadBytes": over_aggregate[-1]["payloadBytes"] + 1}
    self.assertFalse(validate("set-1", "set-1:0", over_aggregate))
    self.assertFalse(validate("set-1", "set-1:0", [
      {"routeId": "set-1:0", "payloadSha256": digest, "payloadBytes": 32},
      {"routeId": "set-1:0", "payloadSha256": digest, "payloadBytes": 32},
    ]))
    self.assertFalse(validate("set-1", "not-allowlisted", manifest[:1]))
    self.assertFalse(validate("set-1", "set-1:0", [{"routeId": "set-1:0", "payloadSha256": "A" * 64, "payloadBytes": 32}]))
    self.assertFalse(validate("set-1", "set-1:0", [{"routeId": "set-1:0", "payloadSha256": digest, "payloadBytes": MAX_BYTES + 1}]))
    exact_aggregate = [{"routeId": f"set-1:{i}", "payloadSha256": digest, "payloadBytes": MAX_BYTES // 4} for i in range(16)]
    self.assertTrue(validate("set-1", "set-1:0", exact_aggregate), "aggregate exactly 4 MiB is accepted")
    aggregate_over = list(exact_aggregate)
    aggregate_over[-1]["payloadBytes"] += 1
    self.assertFalse(validate("set-1", "set-1:0", aggregate_over), "aggregate overflow rejects")

  def test_exact_received_bytes_are_hashed_before_json_parsing(self):
    parse = self.helpers["_nav_parse_route_payload"]
    raw = route_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    text, parsed = parse(raw, "set-1:1", digest, len(raw))
    self.assertEqual(text.encode("utf-8"), raw)
    self.assertEqual(parsed["routeId"], "set-1:1")
    with self.assertRaisesRegex(ValueError, "invalid_route_bytes"):
      parse(raw, "set-1:1", digest, len(raw) + 1)
    with self.assertRaisesRegex(ValueError, "route_digest_mismatch"):
      parse(b"not even JSON".ljust(32, b"!"), "set-1:1", digest, 32)
    changed = raw[:-1] + (b" " if raw[-1:] != b" " else b"!")
    with self.assertRaisesRegex(ValueError, "route_digest_mismatch"):
      parse(changed, "set-1:1", digest, len(changed))
    with self.assertRaises((UnicodeDecodeError, ValueError)):
      invalid_utf8 = b"{" + b" " * 31 + b"\xff"
      parse(invalid_utf8, "set-1:1", hashlib.sha256(invalid_utf8).hexdigest(), len(invalid_utf8))

  def test_parser_accepts_inclusive_one_mib_and_rejects_oversize(self):
    parse = self.helpers["_nav_parse_route_payload"]
    base = route_bytes()
    # Padding is ignored by schema validation; calculate the exact UTF-8 byte boundary.
    low, high = 0, MAX_BYTES
    exact = None
    while low <= high:
      middle = (low + high) // 2
      candidate = route_bytes(padding="x" * middle)
      if len(candidate) == MAX_BYTES:
        exact = candidate
        break
      if len(candidate) < MAX_BYTES:
        low = middle + 1
      else:
        high = middle - 1
    self.assertIsNotNone(exact, f"base route length was {len(base)}")
    parse(exact, "set-1:1", hashlib.sha256(exact).hexdigest(), len(exact))
    over = exact + b" "
    with self.assertRaisesRegex(ValueError, "invalid_route_bytes"):
      parse(over, "set-1:1", hashlib.sha256(over).hexdigest(), len(over))

  def test_geometry_guidance_and_route_identity_rejections(self):
    parse = self.helpers["_nav_parse_route_payload"]
    invalid_cases = []
    wrong_identity = json.loads(route_bytes())
    wrong_identity["routeId"] = "other"
    invalid_cases.append((wrong_identity, "invalid_route_identity"))
    bad_geometry = json.loads(route_bytes())
    bad_geometry["geometry"][0]["latitude"] = 91
    invalid_cases.append((bad_geometry, "invalid_route_geometry"))
    bad_guidance = json.loads(route_bytes())
    bad_guidance["speechInstructions"] = [None]
    invalid_cases.append((bad_guidance, "invalid_route_guidance"))
    for payload, reason in invalid_cases:
      raw = json.dumps(payload, separators=(",", ":")).encode()
      with self.subTest(reason=reason), self.assertRaisesRegex(ValueError, reason):
        parse(raw, "set-1:1", hashlib.sha256(raw).hexdigest(), len(raw))

  def test_banner_bounds_including_groups_components_and_text_lengths(self):
    valid = [{"distanceAlongGeometry": i, "primary": {"text": "x" * 256,
             "components": [{"text": "y" * 256} for _ in range(16)]}} for i in range(8)]
    validate = self.helpers["_nav_validate_banner_instructions"]
    self.assertTrue(validate(valid), "8 banner instructions, 16 components and 256-char fields are inclusive limits")
    self.assertFalse(validate(valid + [valid[0]]))
    self.assertFalse(validate([{**valid[0], "primary": {"text": "x" * 257}}]))
    self.assertFalse(validate([{**valid[0], "primary": {"components": [{"text": "x"}] * 17}}]))


class MultipartTransferTests(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    spec = importlib.util.spec_from_file_location("nav_route_transfer", TRANSFER)
    cls.transfer_module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = cls.transfer_module
    spec.loader.exec_module(cls.transfer_module)
    cls.transfer_module._has_memory_for_route = lambda _size: True

  def start(self, transfer, total_bytes=8, part_count=2):
    return transfer.start({"requestId": "req", "transferId": "tx", "sessionId": "session",
                           "destKey": "101|3", "routeGeneration": 1,
                           "totalBytes": total_bytes, "partCount": part_count})

  def part(self, index, payload, total=8, count=2):
    return {"requestId": "req", "transferId": "tx", "sessionId": "session", "destKey": "101|3",
            "routeGeneration": 1, "totalBytes": total, "partCount": count,
            "partIndex": index, "routePart": payload}

  def test_reordered_parts_and_identical_duplicate_retry_reassemble_exact_bytes(self):
    transfer = self.transfer_module.NavRouteTransfer()
    self.assertTrue(self.start(transfer)["ok"])
    self.assertTrue(transfer.add_part(self.part(1, b"EFGH"))["ok"])
    self.assertTrue(transfer.add_part(self.part(1, b"EFGH"))["duplicate"])
    result = transfer.add_part(self.part(0, b"ABCD"))
    self.assertTrue(result["complete"])
    self.assertEqual(result["routeBytes"], b"ABCDEFGH")

  def test_conflicting_retry_malformed_part_and_cancel_preserve_no_partial_result(self):
    transfer = self.transfer_module.NavRouteTransfer()
    self.start(transfer)
    transfer.add_part(self.part(0, b"ABCD"))
    conflict = transfer.add_part(self.part(0, b"WXYZ"))
    self.assertEqual(conflict["reason"], "conflicting_duplicate")
    self.assertEqual(transfer.add_part(self.part(1, b"EFGH"))["reason"], "no_transfer")
    self.start(transfer)
    transfer.clear()
    self.assertEqual(transfer.add_part(self.part(0, b"ABCD"))["reason"], "no_transfer")

  def test_255_part_boundary_and_invalid_256_or_oversized_limits(self):
    transfer = self.transfer_module.NavRouteTransfer()
    self.assertTrue(self.start(transfer, 255, 255)["ok"])
    transfer.clear()
    self.assertTrue(self.start(transfer, 256, 256)["ok"],
                    "assembler bounds itself by message totalBytes; protocol part ceiling belongs to appbridge")
    transfer.clear()
    source = APPBRIDGED.read_text(encoding="utf-8")
    self.assertIn("total_bytes <= NAV_ROUTE_SELECTION_MAX_BYTES", source,
                  "appbridge start handlers enforce the 1 MiB application ceiling before staging")
    self.assertIn("NAV_ROUTE_SELECTION_MAX_PARTS", source)

  def test_live_and_diagnostic_selection_source_contracts_for_sequence_replay(self):
    bridge = APPBRIDGED.read_text(encoding="utf-8")
    navigationd = (ROOT / "selfdrive/nav/navigationd.py").read_text(encoding="utf-8")
    # These are source-order contract checks; daemon-level state activation needs the device runtime.
    live_stage = bridge[bridge.index("def _stage_nav_route_selection"):bridge.index("def _apply_nav_route_select(")]
    diag_commit = bridge[bridge.index("def _commit_nav_ble_diag_route_selection"):bridge.index("def _apply_nav_ble_diag_select_route(")]
    live_consumer = navigationd[navigationd.index("def _apply_route_selection_locked"):navigationd.index("def _load_pushed_route(")]
    for label, source in (("live staging", live_stage), ("diagnostic selection", diag_commit)):
      self.assertIn("selectionSequence", source, label)
      self.assertIn("stale_selection", source, label)
      self.assertIn("payloadSha256", source, label)
    self.assertIn("NavRouteSelectAcceptedAck", live_stage)
    self.assertIn("lastSelectionId", diag_commit)
    self.assertIn("lastSelectedRouteId", diag_commit)
    self.assertIn("lastPayloadSha256", diag_commit)
    self.assertLess(live_consumer.index("self._route = route"), live_consumer.index("self._route_selection_ack(envelope, 'accepted')"),
                    "live acknowledgement follows route consumer activation")
    self.assertIn("self._nav_ble_diag_route = route", diag_commit)
    self.assertLess(diag_commit.index("self._nav_ble_diag_route = route"), diag_commit.rindex("self._send_nav_ble_diag_select_ack(settings, 'accepted')"),
                    "test acknowledgement follows test route activation")


class LiveSelectionReplayTests(unittest.TestCase):
  def test_exact_latest_duplicate_is_ack_only_and_stale_conflicts_reject(self):
    navigationd_path = ROOT / "selfdrive/nav/navigationd.py"
    tree = ast.parse(navigationd_path.read_text(encoding="utf-8"))
    node = next((item for item in tree.body if isinstance(item, ast.FunctionDef)
                 and item.name == "_is_exact_route_selection_retry"), None)
    self.assertIsNotNone(node)
    namespace = {}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(navigationd_path), "exec"), namespace)
    is_retry = namespace["_is_exact_route_selection_retry"]
    accepted = {
      "selectionSequence": 2, "selectionId": "id-B", "candidateSetId": "set-1",
      "selectedRouteId": "route-B", "payloadSha256": "b" * 64,
    }
    args = (2, 2, accepted, "id-B", "set-1", "route-B", "b" * 64)
    self.assertTrue(is_retry(*args), "exact duplicate returns cached accepted ACK")
    self.assertFalse(is_retry(1, 2, accepted, "id-A", "set-1", "route-A", "a" * 64),
                     "replay of prior A after B is stale")
    self.assertFalse(is_retry(2, 2, accepted, "id-conflict", "set-1", "route-B", "b" * 64),
                     "conflicting reuse of latest sequence is rejected")
    self.assertFalse(is_retry(2, 2, accepted, "id-B", "set-1", "route-A", "a" * 64),
                     "route identity mismatch is rejected")
    self.assertFalse(is_retry(2, 2, accepted, "id-B", "set-1", "route-B", "c" * 64),
                     "digest mismatch is rejected")
    self.assertFalse(is_retry(3, 2, accepted, "id-C", "set-1", "route-C", "c" * 64),
                     "new sequence is not a duplicate")

    method = navigationd_path.read_text(encoding="utf-8")
    apply = method[method.index("  def _apply_route_selection_locked"):method.index("  def _load_pushed_route(")]
    self.assertLess(apply.index("if selection_sequence <= highest_sequence:"),
                    apply.index("if route_id == context.get('activeRouteId'):"),
                    "duplicate replay is handled before rejecting the already active route")
    self.assertLess(apply.index("self.params.put('NavRouteSelectAck', json.dumps(previous_accepted_ack"),
                    apply.index("self._route = route"),
                    "cached ACK path precedes any route activation")



if __name__ == "__main__":
  unittest.main(verbosity=2)
