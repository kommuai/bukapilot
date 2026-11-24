---
submission_id: ubloxd-daemon-removal-20261103_handoff-1_attempt-1_2026-11-03T17:36:39Z
revision: r8
package_path: ../../design/manifest.json
submission_status: VERDICT_RECORDED
verdict: APPROVED
verdict_id: 3efd4f7b35fe77ba656dd8dd5f87373891564c34dc998257778e7e49ad63a106
---

# Cross-stage handoff — design-to-build

**Design submission**: `ubloxd-removal-design-20261103-v2`  
**Design revision**: `r8`  
**Next consumer**: `build/build-management`

## Approval and package

The design phase gate `design/gatekeeper-design` approved revision `r8` with no revise rounds. Its durable phase verdict is `design/verdict_design-to-build.json` (SHA-256 `40be09e2dbf18e99aa434f701a48b1f837056f447705c6a28a0b249fe2e1c511`), with judgment `APPROVED`, valid, and `held_for_user=false`. The approved design manifest is `design/manifest.json` (SHA-256 `697e2ee56ee816ccfeadb701cf45bb8ef350e3de84c0aaa0cddcbb9c19364e5c`).

The Admiral cross-stage gate also approved revision `r8`; its durable verdict is `design/verdict_design-to-build.cross-stage.json` (SHA-256 `7e98a6115a953d2d6ea02b1e82312d7f2b43cd80eaa0007b19e2b965f93062cc`) and judgment is valid with `prior_reusable=true`, `changed_evidence=[]`, and `held_for_user=false`.

The boundary self-check passed with no failures or warnings. The package-shape validator had no failures; its conditional API and UI items were resolved as out of scope. The phase verdict preserves Info limitations for incomplete runtime detection and unrun KA2 hardware, product-test, and replay checks.

## Build contract

Build the single atomic slice `slice.gnss.ublox-removal` from the approved architecture and plan. Remove UBLOXD/PIGEOND and this subsystem's TICI GNSS manager path, including qcomgpsd's TICI enablement, with no TICI replacement. Keep qcomgpsd's KA2 provider, onroad behavior, XTRA, transport, and GPIO unchanged; configure it as KA2-only with `only_onroad`. On KA2, clear persistent `UbloxAvailable=false` in `manager_init()` before any process starts. Preserve PC simulation external-GPS selection, historical route-derived replay behavior, cereal UBLOX schemas, `pyserial`, and `GNSS_PWR_EN`. Preserve unrelated dirty worktree changes and do not commit.

## Known limits

No product code or tests were changed during design. No KA2 hardware check or historical route replay was performed. The runtime detector is incomplete; the design does not claim a detected runtime classification. These limits are recorded in the approved package and must remain visible at build review.
