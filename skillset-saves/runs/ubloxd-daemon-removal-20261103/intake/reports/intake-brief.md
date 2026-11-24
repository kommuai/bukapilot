# Intake brief: remove UBLOXD and PIGEOND

## Request

Remove `ubloxd` and `pigeond` completely as runtime daemons and remove code and repository wiring that exists only to build, launch, test, debug, or document those daemons.

## Scope

- Delete `system/ubloxd/` and its generated parsers, Kaitai schemas, daemon tests, and build script.
- Remove process registrations and predicates, the Kaitai build option and dependency, and daemon-only test and debug references.
- Remove this subsystem's TICI manager path, including the UBLOX daemons and qcomgpsd TICI enablement; do not add a TICI GNSS replacement.
- Keep qcomgpsd enabled and operating on KA2 as it does today, including its GNSS assistance behavior.
- Keep cereal UBLOX log schemas and replay compatibility needed to read historical drives.
- Preserve all unrelated working tree changes and do not commit.

## Acceptance

- Neither daemon is registered or launchable through manager configuration.
- The deleted `system/ubloxd/` code and daemon-only Kaitai support are absent.
- `qcomgpsd` remains enabled on KA2 and keeps the same onroad GNSS/XTRA behavior.
- The GNSS manager no longer enables TICI daemons or qcomgpsd; no TICI replacement or XTRA changes are introduced.
- Historical UBLOX logs remain parseable through retained schemas and replay compatibility.
- Existing unrelated dirty files remain untouched and unstaged.
- No tests are run unless the user asks to test or verify this change; the required gate/artifact validators are still run as part of Admiral's process.

## Constraints and decisions

- Clear persistent `UbloxAvailable` at KA2 manager startup so KA2 consumers select `gpsLocation`; preserve PC simulation's external-GPS selection and historical replay's route-derived value.
- Keep `gpsLocationExternal` schema and historical tooling consumers where they support simulation or old logs; remove the daemon-specific UBLOX debug layout.
- This cleanup removes TICI-specific manager support for this GNSS subsystem; it does not remove the wider TICI hardware implementation.
- Keep qcomgpsd's KA2 GNSS source, XTRA behavior, and shared GNSS power GPIO dependency unchanged.
- Current harness diagnostics report missing Codex hooks and Python 3.12.3 where this harness requires Python >=3.13. Persistence works; record these as readiness limitations and keep gate checks explicit.
