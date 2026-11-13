# Build package — loggerd KA2 encoder recovery and rotation r10

**Run:** `loggerd-ka2-efficiency-20261029`  
**Submission:** `loggerd-ka2-efficiency-20261029-build-r10`  
**Boundary:** `build-to-review`  
**Package/source revision:** 266  
**Owner:** `build-management`

## Outcome

Gatekeeper-build approved this schema-2 package for the `build-to-review` boundary at package revision 266. The persisted verdict is `verdict_build-to-review.json`, SHA-256 `76e69d9d53d296dc5ee5855758aa6eb8083421bcd1d6d78dfc9473d0cf92dbe5`; `judgment.verdict` is `APPROVED`, `pass` is true, and `revise_packet` is empty. The judgment carries an Info note about one stale sentence in the test report; the current r269 runtime evidence is complete, so no runtime rerun is needed.

Package revision 266 is the source snapshot shared by the runtime probe and final production smoke. The grouped tests ran at save revision 267; revisions 267–271 added test/report evidence refreshes and checkpoint metadata only. The source hashes in the final implementation change set, test probe, and runtime probe agree. The runtime probe retains `input_revision: 266`; it was not rewritten to match later evidence-only save revisions.

The run was resumed after stale-lock recovery at revision 256. Readiness identified Python 3.12 and missing Codex hooks; both are recorded as degraded environment readiness, not as product validation failures. No commit or deployment was requested or performed.

## Implementation

The first-party change set contains 18 changed files under `system/loggerd/`, all accounted for in the aggregate diff and schema-2 `implementation.changed_files` list. The diff path set matches those 18 paths exactly; each current file hash matches the manifest. No generated or vendored files are included. The unrelated dirty `common/params_pyx.so` and `selfdrive/appbridged/appbridged.py` are excluded.

- Report: `reports/report_implementation_r10.md` — SHA-256 `b0208b64f775f35c02588e380a28800538cf7fd1ef797e5f9e59a670008414aa`
- Aggregate diff: `evidence/impl-r10-aggregate.diff` — SHA-256 `f9ddb5a5f8d356e11c7cb4dc6b2a8dea325548a7df588c4723e4099fb5a562a4`
- Build/check record: `evidence/impl-r10-checks.log` — SHA-256 `a95d5ddefba4fb22de1af2c93a90569c8c627afd6714a8c141ab39d7bf4a4b8f`

The implementation includes DMA-BUF-only full-resolution MPP input, packet-progress-based isolated recovery, deterministic rotation and bounded audio handling, and the non-RK3588 qRoad FFmpeg path using configured bitrate/GOP/B-frame settings, monotonic frame PTS, and zero-latency encoding. The test-only all-output-reopen counter is compiled under `LOGGERD_TEST_BUILD` and runtime-gated by `LOGGERD_TEST=1`; production binaries omit it.

## Tests

The full grouped serial KA2 suite passed **14 tests, 0 skipped, in 301.56 seconds**. This includes the direct AC-6 drain-timeout assertion that all three road, driver, and wide-road encoder threads recorded zero calls to the all-output reopen helper after the first valid frame. Focused AC-6 coverage passed 1/1 before the full suite. Test evidence covers recovery timing/error isolation, DMA-BUF and SHM behavior, valid and faulted rotation lifecycles, cleanup, and the non-RK3588/HardwarePC software branch on KA2 silicon.

- Test report: `reports/report_tests_r10.md` — SHA-256 `81bb2b2edb12c92f647463f8668cc862d1dbe586b2bbfb97f2b9c1ae447f63d6`
- Typed test probe: `artifacts/tests-r10-probe.json` — SHA-256 `52f0d8b2b178d7f26178771efff2003dbad28046cbb5baeda419ff514061bd0b`
- Full suite: `evidence/tests-r10-ac6-full-r267.log` — SHA-256 `1a331173847b4e933a3be4d38f4e0a4e18e027514705de8d88e9a77a89cec328`
- Focused AC-6 test: `evidence/tests-r10-ac6-focused-r266.log` — SHA-256 `41d6d2d24ed85b9c8243529fef21bdea6b2f356ae8882b721e7454415ef050d8`

## Runtime health

The final production-binary smoke ran after the AC-6 implementation update. It sent 106 frames per camera stream through segments 0–2, produced all expected files and healthy reports with zero known drops, and decoded qRoad video/audio in each segment. Test-build hooks and fault injection were absent from the production binaries. Board preflight and postflight were clean.

- Runtime report: `reports/report_runtime_health_r10.md` — SHA-256 `da999830095353884fd24309c5bb36c17b5ba0d3accf83d3bc38f26a6fc62f84`
- Typed runtime probe: `artifacts/runtime-r10-probe.json` — SHA-256 `528ab4e04f008f13fb7417646560641d1ea914727b22b9b2d1ffb5856403e82e`
- Production smoke: `evidence/runtime-r10-smoke-r269.log` — SHA-256 `556f5ff807a2825d20034656b758e6ca5d56701dbeba388416d4e624cacbaf2f`
- Production build: `evidence/runtime-r10-build-r269.log` — SHA-256 `1c7706e78a0ae87a3ac6f7b3fe274a06eb56a904f0a74ba9bafa98491aae5590`
- Binary audit: `evidence/runtime-r10-binary-audit-r269.log` — SHA-256 `c6b86d4ecf089c786ce5331eb124fab899380654abe4566874fb08374604c7ae`

## Security and completeness

The typed security applicability record uses the sanctioned reason `no trust-boundary change - security-builder not engaged`. All 18 changed paths are first-party loggerd production/build/test files; no external actor, endpoint, credential, privilege boundary, or cereal schema changed.

The schema-2 trace map covers AC-1 through AC-12 and accounts for all 18 changed paths. The independent completeness report is `reports/report_completeness.md`, SHA-256 `b5d9052c96c69b4df5d52a486e8f23a2365ae31be440a4c90aa27248d2b19f88`. It verified all 12 acceptance rows, the runtime row, and the typed security waiver: 14 proven rows, none unproven. AC-6 is proven by the zero all-output-reopen counter assertion for each of the three encoder threads.

The shared build boundary self-check passed with **zero failures and zero warnings** using the supported Python 3.13 interpreter. The package-shape check passed with zero failures; its two conditional information findings ask the gate to confirm the typed security waiver applies and to decide gate-verdict lineage scope. Its revision-coherence note reports 231 because the preserved path-resolution record is the only artifact with revision front matter; it is historical layout evidence and not a source, test, or runtime input. The final manifest SHA-256 after completeness registration is `bab525088e0507e9fff1ad099708afcfbf02622f88c82facf0cd2574d7484787`.

## Qualification limits

The valid DMA-BUF lifecycle uses a synthetic DMA-heap fixture. It does not qualify live V4L2 DMA-BUF export, camera synchronization fences, capture-buffer reuse, or deployed production MPP post-EOS behavior. Persistent live camera-descriptor failure remains untested beyond deterministic policy assertions and isolated encoder fault seams. The HardwarePC/non-RK3588 lifecycle executed on KA2 hardware; independent non-KA2 silicon and physical PC hardware remain unqualified.

