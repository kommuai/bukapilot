# Architecture: remove UBLOXD and PIGEOND

## Decision and boundaries

Remove the TICI UBLOX receiver pipeline (`pigeond` serial capture, `ubloxd` message decoding, and generated Kaitai parsers) and remove this GNSS subsystem's TICI manager enablement. Do not add a TICI GNSS replacement. Retain qcomgpsd's KA2 onroad behavior, modem GNSS source, transport, and XTRA assistance behavior. Replace its UBLOX arbitration predicate with `only_onroad`; on KA2 this is behavior-equivalent because `ublox_available()` is false there. Retain UBLOX Cap'n Proto services and route-derived replay selection for historical logs.

The change affects the manager's platform process inventory, live GPS parameter initialization, and UBLOX-only build/dependency/test/debug/documentation wiring. It does not add a service, schema, network endpoint, user-facing screen, storage format, runtime, or framework. It does not change qcomgpsd code or its KA2 XTRA path.

## Components and responsibilities

| Component | Responsibility after the change | Data flow |
| --- | --- | --- |
| `system/manager/manager.py` | On KA2 only, clear persistent `UbloxAvailable` at manager initialization before starting any managed process. Leave the value unchanged on PC and other platforms. | KA2 manager startup → `Params[UbloxAvailable]=false` → offroad process launch. |
| `system/manager/process_config.py` | Register qcomgpsd for KA2 only with `only_onroad`; register neither UBLOX daemon nor UBLOX availability arbitration. | Onroad KA2 state → qcomgpsd → `gpsLocation`, `qcomGnss`. |
| `common/gps.py` | Keep the shared GPS service selector. KA2 manager startup sets the value false; PC simulation and process replay retain their existing parameter-driven selection. | `Params` → `gpsLocation` or `gpsLocationExternal`, according to existing platform/simulation/replay state. |
| `system/timed.py` and `selfdrive/selfdrived/selfdrived.py` | Continue selecting a GPS service once during process initialization. | On KA2, `UbloxAvailable=false` → subscription to `gpsLocation`; PC simulation can retain its external-GPS selection. |
| `system/qcomgpsd/qcomgpsd.py` | Keep current KA2 behavior, including modem setup, GNSS power control, message transport, assistance download/injection, and `gpsLocation`/`qcomGnss` publication. | Existing Qualcomm modem GNSS path → existing GPS and GNSS services on KA2. |
| Cereal and process replay | Preserve old UBLOX message types and route-derived `UbloxAvailable` behavior. | Historical `ubloxGnss` records → legacy decoding and original replay service selection. |
| `system/hardware/hardwared.py` and `selfdrive/ui/ui_state.py` | Keep current `gpsLocationExternal` subscriptions unchanged. | The status/UI service behavior remains as it is today; on KA2 the qcom path does not populate the external GPS service. |

The manager startup write is an invariant on KA2: `UbloxAvailable` is persistent, `timed` starts before qcomgpsd in the process list, and `timed` chooses its GPS service once. Therefore `manager_init()` must set the parameter false on KA2 before any process starts; setting it later would leave `timed` subscribed to `gpsLocationExternal`. This ensures only `timed` and `selfdrived`, which call `common.gps.get_gps_location_service`, select the existing qcom `gpsLocation` service. As specified in IF-5, `hardwared` and UI remain subscribed to `gpsLocationExternal`. PC simulation retains its existing parameter-driven external-GPS selection, and process replay keeps its route-derived value unchanged. On TICI there is intentionally no live GNSS publisher provided by this subsystem after removal.

## Internal service interfaces

There is no public API endpoint surface. The internal contracts are:

| ID | Kind | Name | Contract |
| --- | --- | --- | --- |
| IF-1 | process lifecycle | GNSS manager platform policy | Remove `ubloxd`, `pigeond`, and the `ublox_available()` arbitration helper. Set qcomgpsd's platform enablement to KA2 only and use `only_onroad`. Do not start qcomgpsd through this subsystem on TICI. |
| IF-2 | parameter and message service | Live GPS service selection | In `manager_init()`, set persistent `UbloxAvailable=false` before any managed process starts only when running on KA2. The `timed` and `selfdrived` consumers that call `common.gps.get_gps_location_service` then select `gpsLocation`; `hardwared` and UI remain on `gpsLocationExternal` under IF-5. Leave PC simulation's parameter-driven external-GPS selection and process replay's route-derived value unchanged. |
| IF-3 | message and assistance services | KA2 qcomgpsd behavior | Keep the existing KA2 `gpsLocation` and `qcomGnss` messages, modem source, onroad lifecycle, XTRA download/injection behavior, transport, and GPIO use unchanged. Replace the process predicate `started and not ublox_available()` with `only_onroad`; on KA2 `ublox_available()` is false, so both predicates reduce to `started` and preserve current behavior. qcomgpsd implementation and its XTRA/GPIO paths are unchanged. |
| IF-4 | historical log format | UBLOX replay compatibility | Keep `ubloxGnss`, `ubloxRaw`, `gpsLocationExternal`, their cereal schema/service definitions, generic legacy consumers, and route-derived replay behavior so old drives remain readable. Remove only the `ubloxd` process replay target. |
| IF-5 | status/UI services | Existing external GPS subscriptions | Leave `hardwared` and UI subscriptions to `gpsLocationExternal` unchanged. Their KA2 behavior remains unchanged; changing them to consume `gpsLocation` is outside this request. |
| IF-6 | TICI subsystem availability | Removed TICI GNSS manager path | Do not register qcomgpsd for TICI and do not retain UBLOX daemons there. No location continuity is promised from this subsystem on TICI; broader TICI hardware support remains in place. |

The `gpsLocation` and `gpsLocationExternal` variants share `GpsLocationData`. The existing KA2 qcomgpsd source reports `SensorSource.qcomdiag`; this request preserves that behavior and does not claim qcom accuracy or availability beyond the current KA2 path. No TICI provider-quality claim is made because TICI GNSS manager support is intentionally removed.

## Failure behavior and recovery

- If qcomgpsd is unavailable or fails on KA2, GNSS location from this subsystem is absent or invalid. No UBLOX fallback remains.
- KA2 manager initialization clears `UbloxAvailable`, preventing stale persistent state from routing `timed` and `selfdrived` to the unserved external GPS topic. PC simulation keeps its existing parameter-driven selection; `hardwared` and UI keep their external-service subscriptions under IF-5.
- On TICI, the absence of a GNSS provider from this subsystem is intentional and follows the user decision; this architecture does not add a fallback or claim equivalent TICI location behavior.
- Historical replay remains a separate configuration context. Route contents may set `UbloxAvailable` true and supply `gpsLocationExternal` and UBLOX telemetry from recorded messages.

## Build and maintenance boundaries

Remove the `system/ubloxd/` tree, its SCons inclusion and Kaitai generator option, the `kaitaistruct` direct dependency and lock entry, and `system/ubloxd` from pytest discovery. Keep `pyserial`, which qcomgpsd still uses. Remove only unused `UBLOX_RST_N` and `UBLOX_SAFEBOOT_N` pin definitions; retain `GNSS_PWR_EN`, which qcomgpsd uses on KA2.

Remove daemon-only references in manager tests, process replay's ubloxd process configuration and special-case, replay README examples, the raw-stream capture script, and the UBLOX-only PlotJuggler layout. Preserve generic GPS/simulation layouts, external GPS service definitions, old log schemas, and old-log parser handling. In `process_config.py`, remove `ublox_available()`, the daemon-only `ublox()` predicate, and both daemon process entries. Set qcomgpsd's `enabled` condition to KA2 and its predicate to `only_onroad`.

## Trust boundary implications

On KA2, the modem diagnostic and XTRA assistance paths already exist and remain unchanged; this change does not activate or broaden them. XTRA assistance is still external input in the existing qcomgpsd path and remains part of the security assessment, but changing transport or validation is outside this architecture. On TICI, qcomgpsd and UBLOX manager registrations for this subsystem are removed, so the prior subsystem GNSS path is no longer enabled. No new remote endpoint, credential, file upload, or process privilege is added.

## User-facing scope

No user-facing surface changes. This request changes manager support for the GNSS subsystem while preserving KA2 qcomgpsd behavior and historical log reading. UI design-system evidence is out of scope.

## Known limitations

No KA2 hardware run or historical route replay was performed for this architecture revision. The local project-stack detector previously stopped at its 10 MB inspection budget and reported Python 3.12.3 below the harness's Python 3.13 requirement; no runtime classification is claimed here.
