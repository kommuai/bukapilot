# Delivery Plan — remove UBLOXD/PIGEOND and TICI GNSS manager support

**Revision**: 2  
**Architecture**: `reports/architecture.md`@`9b51e014d4626fdefc9ccc48c9f0b0b3ce8d99e0089320399bcf21d3f5e006e3`  
**Requirements**: `reports/report_research.md`@`5a176afdae8d658d9ada87432c8fae40997b3a2729f2ccd76470320052fab1f3`  
**Decision register**: `../intake/report_grilling.md`@`b9a771f641732d80f5f247648e207b92a8feb529bbc3c67f4a02ee773e8de8f5`  
**Intake brief**: `../intake/reports/intake-brief.md`@`a20b350b2b2ed74ba3a61e08183cd7b707c856c566eeb547015e30e67bd4fc92`  
**Taste snapshot**: `artifacts/taste-snapshot.json`@`f8f03c10342a4dd3544cce957c71ded99583ae0691c9c8434afd62a971f18845`

## Milestone path

| # | Milestone | Delivery slice | Depends on | Decision gate before it | Target |
| - | --- | --- | --- | --- | --- |
| 1 | Remove the TICI UBLOX manager path and daemon-only repository wiring; retain KA2 qcomgpsd behavior and old-log compatibility. | `slice.gnss.ublox-removal` | Approved architecture, typed security seed, implementation spec, and commander-owned stack applicability | Static review confirms the manager platform inventory, KA2 parameter initialization, build graph, and historical replay scope match intake. | This change request |

## Delivery slice and dependency order

Keep the code change as one atomic slice. The manager parameter invariant and process inventory must land together, while the rest of the slice removes only dead daemon-specific wiring.

1. **Set KA2 startup selection.** In `manager_init()`, set persistent `UbloxAvailable=false` only when `KA2` is true, before any managed process starts. Leave PC simulation and other platforms unchanged. This ensures KA2 `timed` and `selfdrived` select `gpsLocation` while their shared selector remains unchanged.
2. **Narrow the manager process inventory.** Remove `ublox_available()`, the UBLOX-only predicate, and the `ubloxd` and `pigeond` entries. Set qcomgpsd to `enabled=KA2` and `only_onroad`. On the scoped KA2, the current `started and not ublox_available()` predicate is equivalent to `only_onroad` because UBLOX hardware is absent. Remove qcomgpsd's TICI enablement; do not add a TICI GNSS replacement.
3. **Delete daemon-only code and wiring.** Remove `system/ubloxd/`, Kaitai generation/build wiring and dependency, pytest discovery and daemon tests, manager test blacklist entry, ubloxd process replay target and its exception, replay documentation examples, raw UBLOX capture script, UBLOX-only PlotJuggler layout, daemon-only debug references, and UBLOX reset/safeboot pin definitions.
4. **Preserve compatibility and KA2 implementation.** Keep qcomgpsd's KA2 source, XTRA assistance, transport, onroad lifecycle, and `GNSS_PWR_EN` unchanged. Keep `pyserial`, cereal UBLOX schemas/services, historical replay parameter selection, PC simulation external-GPS selection, generic GPS layouts, and `hardwared`/UI `gpsLocationExternal` subscriptions unchanged. Remove `kaitaistruct` from `pyproject.toml` and regenerate `uv.lock` without unrelated dependency changes.

## Acceptance criteria

| ID | Slice | Observable condition | Check that proves it | Acceptor |
| --- | --- | --- | --- | --- |
| AC-1 | `slice.gnss.ublox-removal` | Manager configuration registers neither `ubloxd` nor `pigeond`; qcomgpsd is enabled for KA2 only and uses `only_onroad`. | Static review of `process_config.py` confirms no UBLOX availability arbitration, no daemon entries, and qcomgpsd has the KA2-only enablement and onroad predicate. | Repository maintainer |
| AC-2 | `slice.gnss.ublox-removal` | KA2 manager startup clears persistent `UbloxAvailable` before any managed process starts; non-KA2 manager startup does not reset this parameter. | Static review confirms the `KA2`-scoped write is in `manager_init()` before the initial `ensure_running()` path; confirm PC/simulation code paths have no new reset. | Repository maintainer |
| AC-3 | `slice.gnss.ublox-removal` | On KA2, the unchanged shared selector resolves `gpsLocation` for `timed` and `selfdrived`; PC simulation and historical replay retain their existing parameter-driven service selection. | Review `common/gps.py`, `system/timed.py`, `selfdrive/selfdrived/selfdrived.py`, simulation setup, and route-derived replay configuration against the KA2-only manager write. | Repository maintainer |
| AC-4 | `slice.gnss.ublox-removal` | qcomgpsd implementation and KA2 provider behavior are unchanged, including XTRA assistance, modem transport, GPIO power control, and onroad start/stop behavior. | Final diff contains no qcomgpsd implementation or assistance/transport/GPIO changes; process configuration alone uses the KA2-equivalent `only_onroad` predicate. | Repository maintainer |
| AC-5 | `slice.gnss.ublox-removal` | `hardwared` and UI continue subscribing to `gpsLocationExternal`; no qcom location consumer migration is included. | Final diff leaves their service subscriptions and status/UI handling unchanged. | Repository maintainer |
| AC-6 | `slice.gnss.ublox-removal` | Historical drives containing `ubloxGnss`/`ubloxRaw` remain decodable, and replay keeps deriving `UbloxAvailable` from recorded messages while the `ubloxd` process replay target is removed. | Static review confirms cereal schema/service definitions and log-derived replay setup remain, and only the daemon process target is removed. No route replay is claimed as run. | Log/replay maintainer |
| AC-7 | `slice.gnss.ublox-removal` | The daemon source tree, Kaitai generator option/dependency, daemon-only tests/debug tooling, and daemon documentation examples are absent; `pyserial` and `GNSS_PWR_EN` remain. | Inspect the final diff and use scoped source searches across manager/build/test/tooling paths, allowing only documented historical UBLOX log and simulation references. Review `pyproject.toml` and `uv.lock` after lock regeneration. | Repository and build maintainers |
| AC-8 | `slice.gnss.ublox-removal` | TICI has no qcomgpsd or UBLOX manager registration from this GNSS subsystem; no TICI replacement is added. | Static review confirms qcomgpsd `enabled=KA2` and the removed UBLOX process entries. This criterion is configuration scope only; no TICI runtime check is required. | Repository maintainer |
| AC-9 | `slice.gnss.ublox-removal` | Pre-existing unrelated working-tree changes remain untouched and unstaged; no commit is created. | Compare the final status and diff against the known pre-existing dirty paths; inspect staging state. | Repository maintainer |

No product tests, hardware checks, or replay runs are performed for this change. The listed checks are static review conditions. A future ordinary KA2 drive observation may be useful, but is not claimed done or required to close this plan. The local project-stack detector is incomplete and below its Python runtime floor; stack applicability is owned by commander and is not asserted here.

## Rollout and rollback shape

| Stage | Audience | Mechanism | Rollback | Trigger to advance |
| --- | --- | --- | --- | --- |
| Code review | Maintainers | Review the atomic manager, build, and compatibility diff with the static criteria above. | Withdraw only this change set if review identifies a KA2 or historical-log regression; preserve unrelated dirty files. | Review confirms the requested platform scope and KA2 equivalence. |
| Normal use | KA2 operator | Use the accepted build with the existing qcomgpsd path onroad. | Revert this scoped change if a KA2 regression is observed; no TICI GNSS provider is included by this request. | No additional rollout phase is required. |

This plan does not authorize tests, deployment, hardware validation, a commit, or staging unrelated changes.

## Risk handling

| Risk | Kind | Severity | Consequence if it lands | Mitigation | Owner |
| --- | --- | --- | --- | --- | --- |
| Persistent `UbloxAvailable=true` on KA2 reaches `timed` before any clearing. | startup ordering | Major | `timed` subscribes to `gpsLocationExternal`, which is not published by the retained KA2 qcomgpsd path. | Clear the parameter only on KA2 inside `manager_init()` before the first managed process starts. | Build maintainer |
| Simplified qcomgpsd predicate changes KA2 process lifecycle. | technical | Minor | qcomgpsd starts/stops at a different time than before. | The scoped KA2 device has no `/dev/ttyHS0`, so the old predicate reduces to `started`; use `only_onroad` and leave qcomgpsd implementation unchanged. | Systems maintainer |
| TICI loses this subsystem's GNSS manager path. | platform scope | Info | TICI no longer receives GNSS from UBLOX or qcomgpsd through this subsystem. | This is explicit intake scope; do not add or imply a TICI replacement. Broader TICI hardware code remains out of scope. | Project maintainer |
| Removing historical message definitions breaks old logs. | compatibility | Minor | Existing drive logs fail to decode. | Retain cereal UBLOX schemas/services and route-derived replay behavior. | Log/replay maintainer |
| Kaitai lock regeneration changes unrelated dependencies. | build maintenance | Minor | Dependency drift obscures this daemon-only cleanup. | Regenerate after removing only the direct dependency and inspect lockfile diff for unrelated version changes. | Build maintainer |
| Repository search cannot inspect external launchers outside this checkout. | coverage | Info | An external deployment script might still reference removed daemon paths. | Limit the claim to this repository and have the release owner update external manifests if one is found. | Release maintainer |
| Runtime classification remains unverified on this host. | verification | Info | Stack registry applicability cannot be independently derived here. | Commander owns the sanctioned stack fallback and will record the incomplete detector limitation. | Commander |

## Decision Register

| Decision | Status | Chosen | Source | Rejected options (why) | Owner | Reopen trigger / latest safe point |
| --- | --- | --- | --- | --- | --- | --- |
| D-1 Remove both daemons and daemon-only repository wiring | resolved | Remove their manager entries, implementation, generated parsers, build/dependency/test/debug/documentation paths. | user | Keep either daemon; conflicts with complete removal. | Project maintainer | Reopen if an in-repository production consumer is found before the change lands. |
| D-2 Remove this subsystem's TICI manager path | resolved | Remove TICI qcomgpsd enablement with the UBLOX daemon registrations; do not add a TICI GNSS replacement. | user | Promote qcomgpsd on TICI; outside the confirmed KA2 target scope. | Project maintainer | Reopen only if user requests TICI GNSS support. |
| D-3 Preserve historical UBLOX log support | resolved | Retain cereal service/schema definitions and legacy replay handling. | codebase | Delete message schemas; would break stored drives. | Log/replay maintainer | Reopen for a separately approved log-format migration. |
| D-8 Preserve current KA2 qcomgpsd behavior | resolved | Keep provider, effective onroad lifecycle, XTRA assistance, transport, and GPIO behavior unchanged. | user | Change XTRA, source, or onroad behavior; not required for daemon removal. | Systems maintainer | Reopen if the user requests a separate KA2 GNSS change. |
| D-10 Clear the persistent GPS selector only on KA2 startup | resolved | Set `UbloxAvailable=false` in KA2 `manager_init()` before process launch; leave PC simulation and historical replay selection unchanged. | codebase | Reset globally or inside a later qcom predicate; would alter PC behavior or run after `timed` starts. | Systems maintainer | Reopen if another supported platform requires a different GPS source initialization. |
| D-4 Remove old UBLOX schemas/replay compatibility | deferred | Keep historical log format. | codebase | None; schema migration is separate from daemon removal. | Log-format owner | Future log-format migration, before schema removal. |
| D-5 Promote qcomgpsd to replace UBLOX on TICI | rejected | Do not add TICI GNSS support through this subsystem. | user | User confirmed KA2 scope and requested removal of the TICI manager path. | Project maintainer | Reopen only if user asks for TICI GNSS support. |
| D-6 Keep ubloxd on TICI | rejected | Remove ubloxd and pigeond entirely. | user | Conflicts with daemon removal. | Project maintainer | None for this request. |
| D-7 Remove qcomgpsd from KA2 or change XTRA behavior | yagni-deferred | Keep the existing KA2 qcomgpsd behavior. | user | Broader KA2 GNSS change is unnecessary for daemon removal. | User | Reopen on a separate user request. |
| D-9 Remove TICI hardware/platform infrastructure beyond this GNSS manager path | yagni-deferred | Leave unrelated TICI hardware implementation intact. | user | Broader hardware removal is outside the scoped subsystem change. | User | Reopen only with explicit authorization for broader TICI code removal. |

## Migration / deprecation — UBLOX runtime and TICI manager path

**Replacement readiness**: On KA2, the qcomgpsd provider and its effective onroad lifecycle are already present; `only_onroad` is behavior-equivalent to the current predicate on the scoped KA2 because UBLOX hardware is absent. TICI has no replacement by design.  
**Consumer discovery**: Repository references were inspected across manager configuration, GPS selection, time and selfdrive consumers, hardware/UI subscriptions, simulation, replay, schemas, tests, build files, tools, and layouts. This does not cover launchers outside this checkout.  
**Consumers found**: KA2 qcomgpsd — systems maintainer; `timed` and `selfdrived` GPS selection — systems maintainer; historical logs and replay — log/replay maintainer; TICI daemon manager entries — removed per user decision. `hardwared` and UI stay on `gpsLocationExternal`.  
**Posture**: Direct removal from the TICI manager configuration and atomic daemon cleanup in this requested change; no advisory window because the user requested complete daemon removal.  
**Migration path**: No stored-log migration. Existing UBLOX schemas and replay handling remain. KA2 stays on its existing qcomgpsd path; no TICI GNSS replacement is introduced.  
**Removal criteria**: Neither daemon is registered or present under `system/ubloxd`; qcomgpsd remains KA2-only and onroad; KA2 startup clears the selector before processes start; historic message definitions remain.  
**Fallback**: If the KA2 static review reveals a provider or startup regression, revert this scoped change and correct the manager wiring. TICI GNSS remains intentionally out of scope.

## Non-goals and YAGNI deferrals

| Deferred | Why it is safe to defer | Reopen trigger | Owner |
| --- | --- | --- | --- |
| Remove UBLOX schemas or historical replay handling | Runtime daemons and TICI manager entries can be removed without rewriting stored logs. | A separate log-format migration is approved. | Log-format owner |
| Add a TICI GNSS replacement | The confirmed scope removes the TICI manager path and does not ask for replacement behavior. | User asks for TICI GNSS support. | User |
| Change `hardwared`/UI external-GPS subscriptions | The updated architecture explicitly keeps those service consumers unchanged. | Separate decision to route them to `gpsLocation`. | Project maintainer |
| Benchmark CPU or memory effects | No resource target or benchmark was requested. | User requests a benchmark with a defined baseline and measurement window. | User / maintainer |
