# Implementation Spec — remove UBLOXD/PIGEOND and TICI GNSS manager support

**Revision**: 1  
**Design package**: `ubloxd-removal-design-20261103-v2` (run revision 23)  
**Architecture**: `reports/architecture.md`@`9b51e014d4626fdefc9ccc48c9f0b0b3ce8d99e0089320399bcf21d3f5e006e`  
**Plan**: `reports/plan.md`@`862ec7a07df5564560772ab66a690dd631387791a145238a6efb5d429aa2d85a`  
**Security seed**: `reports/security-seed.json`@`0b9cc411b07301f923eccc7ef7a0279d20fe1a68639d7acb989794b7a948a3e3`  
**Detected stack**: Existing stack unchanged; no runtime or framework is introduced. Runtime classification is unverified because the host Python is 3.12.3 (harness minimum 3.13) and the repository scan stopped at its 10 MB limit.

## Slice order

| # | Slice id | Depends on | Ships behind | Acceptance owner |
| - | -------- | ---------- | ------------ | ---------------- |
| 1 | `slice.gnss.ublox-removal` | Approved architecture, interfaces, plan, security assessment, and stack applicability; no preceding implementation slice | No flag; one atomic repository change | Repository maintainer |

## Slices

### `slice.gnss.ublox-removal`

**Modules owned**: `system/manager/manager.py`; `system/manager/process_config.py`; `system/manager/test/test_manager.py`; deletion of `system/ubloxd/`; `SConstruct`; `pyproject.toml`; `uv.lock`; daemon-specific portions of `selfdrive/test/process_replay/process_replay.py`, `selfdrive/test/process_replay/test_processes.py`, and `selfdrive/test/process_replay/README.md`; daemon-only capture/debug artifacts such as `tools/scripts/save_ubloxraw_stream.py` and `tools/plotjuggler/layouts/ublox-debug.xml`; UBLOX reset/safeboot pin definitions in `system/hardware/tici/pins.py`. Preserve unrelated working-tree files and the historical/service files called out below.

**Depends on**: No other implementation slice. The manager parameter policy and process inventory land together because a KA2 process starting before the stale persistent selector is cleared could choose an unserved location topic. Build, dependency, test-discovery, and debug cleanup is part of this same removal so no partial release can launch or compile the retired daemon.

**Data / migration changes**: No schema migration or data rewrite. Keep the persistent `UbloxAvailable` key for PC simulation and replay. In `manager_init()`, set it to false only on KA2, before any managed process starts. Leave the parameter unchanged on PC and other platforms; keep process replay's route-derived value unchanged. Remove `kaitaistruct` from `pyproject.toml` and regenerate `uv.lock`, reviewing the lock diff for unrelated version churn.

**Contract tests**: No public endpoint or cereal service contract changes. The internal manager contract is IF-1 through IF-6 in the architecture. No product test is authorized or run in this request. If test execution is separately authorized later, focused manager coverage should prove the KA2-only parameter reset, unchanged non-KA2 parameter behavior, absence of UBLOXD/PIGEOND registrations, and KA2 qcomgpsd `only_onroad` policy. Existing modem/XTRA integration tests are not required to validate this unchanged qcomgpsd implementation and must not be represented as run.

**Proof plan**: The build phase should first establish the above focused manager expectations as failing assertions if testing is authorized; the smallest implementation is the KA2-guarded parameter write and the process-config reduction. Then delete the daemon tree and its exclusive build, dependency, test, replay, capture, and layout wiring in the same slice. The refactor boundary is limited to dead UBLOX manager plumbing and daemon-only artifacts; do not restructure qcomgpsd or its XTRA/transport/GPIO code. Under the current scope, use the plan's static source and diff review criteria only; no test command, device run, or historical replay has been executed or is claimed.

**Rollback**: Withdraw the single change set as a unit if KA2 startup selection or retained historical parsing is shown to regress. Restore only files changed by this slice; preserve unrelated dirty worktree changes. There is no parameter migration to reverse: a future rollback restores the prior manager code and process configuration. Do not add TICI GNSS support as an implicit rollback path.

**Acceptance evidence**: Repository maintainer reviews the final diff and static search results against plan AC-1 through AC-9. Confirm `manager_init()` clears the parameter only on KA2 before process startup; qcomgpsd is KA2-only with `only_onroad`; the old predicate was equivalent on KA2; neither daemon remains registered or in the source tree; the Kaitai dependency/build wiring is gone; `pyserial` and `GNSS_PWR_EN` remain; the PC simulation and historical replay branches are intact; `hardwared` and UI subscriptions are unchanged; and no unrelated dirty file was modified or staged. No KA2 hardware run, test suite, or route replay is part of this evidence.

## Operational constraints

| Constraint | Slice affected | Owner | Removal criterion |
| ---------- | -------------- | ----- | ----------------- |
| KA2-only `UbloxAvailable=false` write must precede all managed process starts; `timed` chooses its service once and starts before onroad qcomgpsd. | `slice.gnss.ublox-removal` | Systems maintainer | Static review confirms the KA2 guard and startup ordering; non-KA2 manager and replay/simulation selection remain unchanged. |
| qcomgpsd process policy changes from UBLOX arbitration to `only_onroad`; on KA2 the existing predicate evaluates to `started` because `ublox_available()` is false there. qcomgpsd's source, XTRA, transport, and GPIO behavior stay unchanged. | `slice.gnss.ublox-removal` | Systems maintainer | Static review confirms KA2-only enablement and no qcomgpsd implementation changes. |
| Kaitai lock regeneration must not rewrite unrelated dependency versions. | `slice.gnss.ublox-removal` | Build maintainer | Review `uv.lock` diff after removing only the daemon-only dependency. |
| There is no feature flag, cutover, backfill, or schema migration. Existing manager state and `qcomGnss`/location messages provide the current operational signal; add no new telemetry. | `slice.gnss.ublox-removal` | Repository maintainer | No temporary controls or migrations are introduced. |
| Tests, device checks, and historical replay are not run under this request. The host stack diagnostic is incomplete. | `slice.gnss.ublox-removal` | Commander / repository maintainer | Keep the limitation explicit; do not report unrun checks as passing. |

## Non-goals

- Do not add a TICI GNSS replacement or remove wider TICI hardware support.
- Do not change qcomgpsd source, GNSS provider, KA2 onroad lifecycle, XTRA behavior, transport, or `GNSS_PWR_EN` use.
- Do not remove `pyserial`, `UbloxAvailable`, `ubloxGnss`, `ubloxRaw`, `gpsLocationExternal`, their cereal schema/service definitions, or historical replay and simulation support.
- Do not change `hardwared` or UI subscriptions from `gpsLocationExternal` to `gpsLocation`.
- Do not benchmark CPU/memory, run product tests, run a KA2 hardware check, replay a historical route, stage changes, commit, or alter unrelated dirty files.

## Blocking decisions

| Decision | Owner | Blocks slices | Latest safe decision point |
| -------- | ----- | ------------- | -------------------------- |
| None. The KA2 target, TICI manager removal, startup parameter scope, PC/replay preservation, and XTRA behavior are resolved in the intake register. | — | — | — |

## Residual risks

- KA2 behavior equivalence depends on the scoped KA2 having no `/dev/ttyHS0`; this is why the old `qcomgps` predicate reduces to `started`. The design retains this assumption and does not claim a hardware check was run.
- TICI GNSS from this subsystem is intentionally unavailable after removal. This is accepted scope, not an implementation fallback.
- Historical compatibility is supported by retained schemas and route-derived replay logic; no old drive was replayed during design.
- The runtime detector did not complete, so the stack record will use the sanctioned no-new-runtime/framework applicability only because this change adds no runtime or framework.
