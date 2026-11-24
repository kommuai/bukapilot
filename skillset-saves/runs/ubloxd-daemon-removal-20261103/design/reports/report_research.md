# Requirements Brief — UBLOXD and PIGEOND removal

**Revision**: 2  
**Sources mined**: updated intake decision register; `system/manager/process_config.py`, `system/manager/manager.py`, `system/qcomgpsd/qcomgpsd.py`, `common/gps.py`, `common/params_keys.h`, `system/timed.py`, `selfdrive/selfdrived/selfdrived.py`, `selfdrive/test/process_replay/process_replay.py`, `selfdrive/test/test_ka2_onroad.py`, `system/hardware/hardwared.py`, `selfdrive/ui/ui_state.py`, `tools/sim/lib/simulated_sensors.py`, `cereal/log.capnp`, `cereal/services.py`, `SConstruct`, `pyproject.toml`, `uv.lock`, `tools/lib/log_time_series.py`, and `system/hardware/tici/pins.py`.  
**Prepared for**: `design/architect` (next stage per `pipelines.yaml`)

## Actors and jobs

| Actor | Job to be done | Success looks like | Frequency / volume |
| ----- | -------------- | ------------------ | ------------------ |
| KA2 vehicle operator | Continue receiving live location from the existing qcomgpsd path after UBLOXD/PIGEOND and TICI GNSS manager support are removed. | qcomgpsd remains enabled for KA2 with its current onroad GNSS, XTRA assistance, and transport behavior; live location consumers select `gpsLocation`. | One manager-controlled onroad lifecycle; `gpsLocation` is listed at 1 Hz in `cereal/services.py:55`, and KA2 on-road checks expect qcom GNSS messages at `selfdrive/test/test_ka2_onroad.py:821-825`. |
| System maintainer | Remove this subsystem's TICI manager path and UBLOX-specific daemon runtime. | No `ubloxd` or `pigeond` entries remain; qcomgpsd is no longer enabled for TICI; daemon-only source/build/test/dependency paths are removed. | Checked at each build/release; target count is zero for UBLOXD/PIGEOND and TICI qcomgpsd enablement. |
| Historical log reader and replay user | Read old drive segments that contain UBLOX service messages. | Cereal schemas and historical replay selection continue to decode those messages. | Existing logs; old `ubloxGnss` and `ubloxRaw` service rates are declared as 10 Hz and 20 Hz in `cereal/services.py:56,60`. |

## Requirements

| # | Requirement | Source | Confidence | Affects | Open assumption |
| - | ----------- | ------ | ---------- | ------- | --------------- |
| R-1 | The manager must no longer register or launch `ubloxd` or `pigeond`. | User request; current entries are `system/manager/process_config.py:114-115`. | reported | Process inventory and startup lifecycle. | A repository-wide post-change search can establish there are no in-repository launch references; no external deployment launcher was inspected. |
| R-2 | Keep qcomgpsd enabled on KA2 with its existing provider, effective onroad-only start/stop behavior, XTRA assistance, transport, and GPIO use unchanged. The current predicate implementation is not itself a required behavior: on KA2 `ublox_available()` is false, so `started and not ublox_available()` is equivalent to `only_onroad`. | Intake D-8; current predicate and platform gate are `system/manager/process_config.py:26-27,50-51,110`; KA2 runtime check during intake found `/dev/ttyHS0` absent; provider and power setup are `system/qcomgpsd/qcomgpsd.py:260-265,341-364`; KA2 GNSS expectation is `selfdrive/test/test_ka2_onroad.py:821-825`. | inferred | KA2 manager lifecycle and GNSS data path; allows removing the UBLOX-specific arbitration helper without changing KA2 process lifecycle. | A KA2 runtime owner can confirm the output stream on a later drive; no tests or hardware checks were run for this brief. |
| R-3 | Do not enable qcomgpsd for TICI; remove its TICI manager enablement along with the TICI UBLOX daemon registrations. | Intake D-2; current qcomgpsd registration is `system/manager/process_config.py:110`, while TICI-only UBLOX registrations are `process_config.py:114-115`. | reported | TICI manager process inventory for this GNSS subsystem. | TICI GNSS support is intentionally removed from this scope; no replacement provider is required. |
| R-4 | Set persistent `UbloxAvailable` false only during KA2 manager startup, before any managed process starts, so KA2 consumers choose `gpsLocation`. Do not reset it globally: preserve PC simulation's `gpsLocationExternal` selection and process replay's route-derived value. | Intake D-10; parameter is persistent at `common/params_keys.h:173`; `manager.py:192-201` runs init before manager thread and first process ensure at `manager.py:128-129`; `timed` starts before qcomgpsd (`process_config.py:90,110`) and snapshots its service at `system/timed.py:36-40`; selection is `common/gps.py:4-8`; simulation publishes `gpsLocationExternal` at `tools/sim/lib/simulated_sensors.py:18,54-71`; replay sets the parameter from historical `ubloxGnss` at `process_replay.py:724-726`. | reported | KA2 startup ordering and live GPS service selection, with PC simulation and historical replay compatibility. | Keep the reset scoped to KA2 manager startup; do not alter process replay's route-derived parameter or PC simulation's external-service configuration. |
| R-5 | Retain historical UBLOX message decoding and replay compatibility. | Intake D-3/D-4; `cereal/log.capnp:2550-2554`, `cereal/services.py:54-60`, `process_replay.py:409-413,724-726`, `tools/lib/log_time_series.py:19-24`. | reported | Log schema stability and replay of old routes. | This request removes daemons and TICI manager support, not the historical log format; deleting schemas or replay state needs a separate migration decision. |
| R-6 | Remove only UBLOX reset/safeboot pins, while retaining `GNSS_PWR_EN` used by KA2 qcomgpsd. | Intake scope; `system/hardware/tici/pins.py:5-7`; qcomgpsd imports and uses `GPIO.GNSS_PWR_EN` at `system/qcomgpsd/qcomgpsd.py:262-263`. | reported | Hardware GPIO definitions used by the retained KA2 path. | `UBLOX_RST_N` and `UBLOX_SAFEBOOT_N` currently have no other repository references; keep the shared GNSS power-enable symbol. |
| R-7 | Remove files and configuration that exist solely for `ubloxd`/`pigeond`, Kaitai generation, and qcomgpsd's TICI manager enablement. | `SConstruct:21,217-220`; `system/ubloxd/SConscript`; `pyproject.toml:39-40,167`; `uv.lock` Kaitai entries; `system/ubloxd/` tree; current qcomgpsd TICI/KA2 gate in `process_config.py:110`. | observed | Build graph, lockfile, tests, TICI manager inventory, and repository maintenance. | Keep `pyserial` for KA2 qcomgpsd; narrow its comment in `pyproject.toml:14` rather than removing the dependency. |

## Constraints

| # | Constraint | Kind | Source | Confidence | Affects | Open assumption |
| - | ---------- | ---- | ------ | ---------- | ------- | --------------- |
| C-1 | qcomgpsd currently uses `started and not ublox_available()` and is enabled on TICI/KA2. On the KA2 device, `ublox_available()` evaluates false because `/dev/ttyHS0` is absent, making its current start condition equivalent to `only_onroad`; replacing the predicate removes this UBLOX-specific arbitration and TICI-specific path while preserving KA2 onroad lifecycle. | operational | `system/manager/process_config.py:26-27,50-51,110`; KA2 runtime check during intake found `/dev/ttyHS0` absent. | inferred | Per-platform manager inventory and KA2 startup lifecycle. | If a future KA2 build adds `/dev/ttyHS0`, re-evaluate the equivalence; no such device is present on the scoped KA2. |
| C-2 | `UbloxAvailable` is persistent and consumed by live GPS service selection, PC simulation configuration, and historical replay. | technical | `common/gps.py:4-8`; `common/params_keys.h:173`; manager order in `system/manager/manager.py:192-201,128-129`; simulation at `tools/sim/lib/simulated_sensors.py:18,54-71`; replay config at `process_replay.py:409-413,724-726`. | observed | KA2 live parameter lifecycle plus PC simulation and historical replay behavior. | Reset false only during KA2 manager startup; preserve PC simulation's external GPS selection and replay's route-derived value. |
| C-3 | The retained qcomgpsd path uses `GNSS_PWR_EN`; UBLOX reset/safeboot symbols are separate declarations. | technical | `system/hardware/tici/pins.py:5-7`; `system/qcomgpsd/qcomgpsd.py:262-263`. | observed | KA2 hardware GPIO definitions. | Remove only the two UBLOX-specific pins; retain `GNSS_PWR_EN`. |
| C-4 | `kaitaistruct` and Kaitai generation support are referenced only by the UBLOX parser tree and root dependency wiring. | technical | `system/ubloxd/SConscript`, `system/ubloxd/generated/*.py`, `pyproject.toml:39-40`, `uv.lock:781,1308,1405`. | observed | Build options and lockfile. | Regenerate the lockfile without changing unrelated dependency versions. |
| C-5 | `hardwared` subscribes to `gpsLocationExternal`, while KA2 qcomgpsd publishes `gpsLocation`; keeping these consumers unchanged is the scope-preserving default because the request keeps KA2 qcomgpsd behavior as it is today. | operational | `system/hardware/hardwared.py:169,441`; `system/qcomgpsd/qcomgpsd.py:265,341-364`; `selfdrive/ui/ui_state.py:53`; intake D-8. | inferred | Hardware status packet and UI subscription. | The status packet's external-only location field is already not populated by the KA2 qcomgpsd service. Changing consumers to use `gpsLocation` would be a separate behavior change. |

## Non-functional targets

| Target | Number | Source | Confidence | Why this number |
| ------ | ------ | ------ | ---------- | --------------- |
| UBLOXD/PIGEOND manager registrations after removal | 0 | User request and current registration locations `system/manager/process_config.py:114-115`. | reported | “Completely remove” is objectively satisfied only when neither daemon is registered. |
| qcomgpsd TICI manager enablement after removal | 0 | Intake D-2 and current process gate `system/manager/process_config.py:110`. | reported | This user-specified subsystem scope removes TICI GNSS manager support without adding a replacement. |

No CPU or memory reduction target was specified. This scope does not require a benchmark; any numeric resource target should be set by the user or maintainer in a separate measurement task.

## Non-goals

- Keep qcomgpsd's provider, effective onroad start/stop behavior, XTRA, transport, and GPIO use on KA2. Replacing the `qcomgps` predicate with `only_onroad` is an implementation choice supported by the current KA2 device check, not a requirement to retain the predicate code.
- Do not add a TICI GNSS replacement; this request intentionally removes this subsystem's TICI manager enablement while the user gradually retires TICI devices.
- Do not remove `ubloxGnss`, `ubloxRaw`, `gpsLocationExternal`, or their cereal schema/service definitions used for old logs and simulation; reopen for a separate log-format migration.
- Preserve PC simulation's `gpsLocationExternal` selection and process replay's route-derived `UbloxAvailable` value.
- Leave `hardwared` and UI GPS subscriptions unchanged as the scope-preserving default; the request is to retain KA2 qcomgpsd behavior as it is today. Changing `hardwared` to consume `gpsLocation` would be a separate behavior change. Keep qcomgpsd source/XTRA transport unchanged and do not add HTTPS/disable-XTRA controls.
- Do not remove `pyserial`, which remains needed by KA2 qcomgpsd.
- Do not alter unrelated working-tree changes or commit as part of this request.

## Prior art and comparable flows

| Source | What it shows | Why it transfers (or does not) |
| ------ | -------------- | ------------------------------ |
| `system/qcomgpsd/qcomgpsd.py:265,341-364` | Existing modem GNSS daemon publishes `qcomGnss` and `gpsLocation`, sets source to `qcomdiag`, maps position, accuracy, and fix fields. | This is the retained KA2 provider; its code path and transport stay unchanged. |
| `system/manager/process_config.py:26-27,50-51,110,114-115` plus KA2 runtime check | qcomgpsd is gated by `started and not ublox_available()` and currently enabled on TICI/KA2; on KA2 the UBLOX check is false, so the effective condition is `started`. | `only_onroad` is behavior-equivalent on KA2; removing the UBLOX arbitration helper and narrowing qcomgpsd enablement to KA2 removes the TICI-specific path without changing qcomgpsd's provider or lifecycle there. |
| `selfdrive/test/test_ka2_onroad.py:258,389,821-825` | KA2 on-road suite includes qcomgpsd in its process accounting and expects a stream of qcom GNSS messages. | Existing KA2 runtime expectation; it is not run for this design revision. |
| `cereal/log.capnp:2550-2554` and `selfdrive/test/process_replay/process_replay.py:724-726` | Logs can contain UBLOX service messages, and replay derives `UbloxAvailable` from whether `ubloxGnss` appears. | Preserve log decoding and per-route selection even after removing the live daemons. |

## Conflicts preserved

| Conflict | Position A (source, confidence) | Position B (source, confidence) | Who decides |
| -------- | ------------------------------- | ------------------------------- | ----------- |
| Existing manager configuration enables qcomgpsd on TICI as well as KA2, but updated intake removes TICI GNSS manager support and requires no replacement there. | `system/manager/process_config.py:110,114-115` (observed). | Intake D-2 explicitly scopes the device to KA2 and removes TICI manager enablement (reported). | User decision recorded at intake; no unresolved conflict remains. |

## Open questions

| Question | Blocks | Owner | Latest safe decision point |
| -------- | ------ | ----- | -------------------------- |
| Does qcomgpsd continue producing the expected `qcomGnss`/`gpsLocation` stream on KA2 after the repository change? | Runtime confirmation only; the code path and KA2 checks already exist. | KA2 vehicle/runtime owner | On-device drive after the change is installed. |
| What measurable CPU or memory change is expected from removing these daemons? | No current implementation decision; no benchmark is authorized or required by this request. | User / maintainer | Before a separate performance benchmark. |

## Limitations

- No KA2 hardware run was performed for this report; the existing qcomgpsd source and on-road expectations were inspected only.
- The TICI GNSS manager path is intentionally removed by user decision; this brief makes no claim that TICI GNSS will continue to work through this subsystem.
- The local runtime diagnostic is incomplete because the host runs Python 3.12.3 where the harness requires Python 3.13, and the project walk exceeded its total-byte limit. No runtime classification is claimed from that check.
- Historical compatibility was inferred from source schemas and replay logic; no old route was replayed.
