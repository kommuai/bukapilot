---
submission_id: ubloxd-removal-design-20261103-v2
revision: r8
---

# Design Package — UBLOXD/PIGEOND removal

**Boundary**: `design-to-build`  
**Submission ID**: `ubloxd-removal-design-20261103-v2`  
**Design revision**: `r8`  
**Run**: `ubloxd-daemon-removal-20261103`

## Outcome and scope

This package specifies removal of the `ubloxd` and `pigeond` daemons, their daemon-only code and repository wiring, and this GNSS subsystem's TICI manager enablement. It does not add a TICI GNSS replacement. KA2 keeps its existing qcomgpsd provider, onroad behavior, XTRA assistance, transport, and GPIO use. Historical UBLOX schemas and replay support remain intact.

The package is ready for the `design-to-build` gate. No product code was changed, no product tests or hardware checks were run, no route was replayed, and no commit was made.

## Decisions

The hashed intake decision register is `../intake/report_grilling.md` (SHA-256 `b9a771f641732d80f5f247648e207b92a8feb529bbc3c67f4a02ee773e8de8f5`); the normalized brief is `../intake/reports/intake-brief.md` (SHA-256 `a20b350b2b2ed74ba3a61e08183cd7b707c856c566eeb547015e30e67bd4fc92`). Resolved decisions include:

- Remove the UBLOX daemon source/build/test/debug/documentation paths and both process registrations.
- Remove qcomgpsd's TICI manager enablement; keep qcomgpsd on KA2 and do not introduce TICI GNSS replacement behavior.
- On KA2 only, set persistent `UbloxAvailable=false` in `manager_init()` before any managed process starts. Leave PC simulation selection and historical process replay's route-derived parameter unchanged.
- Replace the qcomgpsd UBLOX arbitration predicate with `only_onroad`. On the scoped KA2, the old predicate evaluates to `started` because `ublox_available()` is false, so the simplified predicate preserves the existing onroad lifecycle.
- Keep qcomgpsd implementation, provider, XTRA, transport, and GPIO behavior unchanged. Retain `pyserial` and `GNSS_PWR_EN`.
- Keep cereal UBLOX schemas/services and historical replay support; remove only the ubloxd process replay target and daemon-specific tooling.

## Architecture and interfaces

The architecture report is `reports/architecture.md` (SHA-256 `9b51e014d4626fdefc9ccc48c9f0b0b3ce8d99e0089320399bcf21d3f5e006e`). It defines six internal contracts, IF-1 through IF-6, covering manager platform policy, KA2 startup parameter selection, unchanged KA2 qcomgpsd behavior, historical log compatibility, unchanged hardwared/UI external-GPS subscriptions, and removal of this subsystem's TICI path.

On KA2, `manager_init()` sets `UbloxAvailable=false` before `timed` or any other managed process can start. This matters because `timed` selects its GPS topic once at process startup and starts before onroad qcomgpsd. The reset is KA2-only: PC simulation retains its parameter-driven `gpsLocationExternal` selection, and process replay retains its route-derived value. `timed` and `selfdrived` use the shared selector and choose `gpsLocation` on KA2; hardwared and UI remain on their current `gpsLocationExternal` subscriptions.

There is no public API, new message schema, user-facing screen, or new runtime/framework. qcomgpsd is registered for KA2 only and uses `only_onroad`; the daemon-specific `ublox_available()` helper and `ublox()` predicate are removed. The wider TICI hardware implementation remains outside scope.

## Delivery plan and acceptance

The delivery plan is `reports/plan.md` (SHA-256 `862ec7a07df5564560772ab66a690dd631387791a145238a6efb5d429aa2d85a`). It defines one atomic repository slice and acceptance criteria AC-1 through AC-9. The acceptance criteria cover:

- No UBLOXD/PIGEOND registration or source tree, and no TICI qcomgpsd manager enablement.
- KA2-only startup reset before process launch; PC simulation and historical replay selection preserved.
- KA2 qcomgpsd remains onroad with equivalent `only_onroad` behavior and unchanged implementation/XTRA/transport/GPIO.
- Historical UBLOX schemas and replay selection remain; daemon-only build, dependency, test-discovery, debug, and documentation paths are removed.
- Existing unrelated dirty files remain unchanged and unstaged; no commit is created.

All current acceptance checks are static source and diff reviews. No product tests, hardware checks, or replay runs are claimed. A future ordinary KA2 drive observation may be useful but is not required to close this design package.

## Implementation specification

The implementation spec is `reports/implementation-spec.md` (SHA-256 `93ce5f5495225df742682d0df1bedac3be466d9b9742524ec6b692f753f28650`). It keeps manager startup and process inventory changes together with daemon-only repository cleanup in one slice. It specifies module ownership, the KA2 parameter ordering, dependency cleanup and lock review, rollback as withdrawing only this scoped change, and operational constraints. It does not authorize a product test, hardware run, replay, staging, or commit.

## Security assessment

`reports/security-seed.json` (SHA-256 `0b9cc411b07301f923eccc7ef7a0279d20fe1a68639d7acb989794b7a948a3e3`) records the security-builder's typed determination that this design introduces or moves no trust boundary. The KA2 qcomgpsd modem and XTRA paths are already active and remain unchanged; TICI provider paths are removed; PC simulation and replay paths remain unchanged. No XTRA transport or validation control is added by this scope.

## Stack lock and Taste

The commander-owned stack applicability report is `reports/report_stack-lock.md` (SHA-256 `56d47755b00e644302225a872bc01d3ba8187d068c72a2c356a49d4df0bbbc00`). This change adds no runtime or framework, so the exact sanctioned applicability is `no new runtime or framework - existing stack unchanged`. The runtime detector found host Python 3.12.3 below its 3.13 floor and stopped its project scan at the 10 MB limit. The classification is unverified; the scan is not described as passing.

The Taste snapshot is `artifacts/taste-snapshot.json` (SHA-256 `f8f03c10342a4dd3544cce957c71ded99583ae0691c9c8434afd62a971f18845`). Before the gate, Taste revalidated that project and global preference stores are still absent at revision 0, with empty lists/effective profile and no conflicts. The snapshot remains valid; no profile or new preference was introduced.

No user-facing surface changes. The architect's UI evidence applicability is `no user-facing surface - design system not engaged`.

## Open risks and limitations

- KA2 hardware behavior and historical replay were not exercised; the KA2 predicate equivalence is based on the scoped KA2's existing UBLOX check being false.
- This subsystem intentionally provides no GNSS manager path on TICI after the change.
- Repository inspection does not cover external launchers outside this checkout.
- Runtime classification is unverified because the host version and scan limits prevented a complete detector result.
- Taste source revisions were unchanged, but absent-store status digests are transient; no preference content exists to resolve.

## Gate handoff

This is a single coherent design revision (`r8`). `manifest.json` carries all nine `design-to-build` evidence keys and the content hashes. The `design-to-build` judgment remains pending the design gatekeeper.
