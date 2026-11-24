# Stack Lock Applicability — UBLOXD/PIGEOND removal

**Revision**: r8  
**Decision**: `no new runtime or framework - existing stack unchanged`  
**Decided by**: commander

## Applicability

This design removes the `kaitaistruct` dependency and UBLOXD-only parser/build wiring. It adds no runtime, framework, or dependency. The existing project stack remains unchanged, so no registered overlay or version lock is introduced for this change.

## Detector evidence and limitation

Command: `python /data/kommu/.codex/skills/scripts/check_runtime.py --project-root /data/openpilot --detect-project`

The read-only detector reported host Python 3.12.3 as below the harness minimum of 3.13. Its project walk exceeded the 10 MB total-byte limit and stopped early; it found no supported runtime classification signal and explicitly marked inspection incomplete. This is not a passing stack scan and does not establish a detected OpenPilot/Python/C++ stack slug. The registry has no such project slug. The sanctioned fallback applies because this change adds no runtime or framework; the incomplete scan remains a limitation.

## Locked scope

- No new runtime or framework is introduced.
- `kaitaistruct` and UBLOXD-only generation support are removed; `pyserial` remains for KA2 qcomgpsd.
- No project runtime or framework version changes are specified.
- The stack detector limitation must remain visible in downstream build planning; no classification or successful detector result is claimed.
