Outcome: bug-review, approved package r266 (review handoff r274), 0 findings: 0 Critical, 0 Major, 0 Minor, 0 Info

Evidence: Read-only trace of the exact 18 first-party paths listed in the approved manifest and aggregate diff; intake, active r10 design/specification, implementation and completeness reports, test report/log/probe, runtime report/log/probe, and build verdict. The SHA-256 of each current scoped file matches its manifest entry. No test, build, runtime script, or reviewed program was run during this pass.

Findings: (none)

Open risks: No source-grounded suspected defect remains unverified. The following qualification limits remain open and are not findings:
- Live V4L2 export, camera synchronization fences, and DMA-BUF reuse under capture are not represented by the synthetic DMA-heap/memfd fixture. The current tests and smoke cannot reproduce a live-capture ordering or lifetime failure. To qualify this behavior, capture a bounded V4L2 run with buffer-generation/fence/reuse evidence and compare submitted and published frame identities through rotations.
- The local production-binary smoke exercises synthetic segment transitions, but the deployed MPP build/version and its post-EOS behavior remain unqualified per the package. Pin the deployed MPP identity and record a direct post-EOS close/open transition on that target before claiming that qualification.
- Persistent live descriptor failure is covered by deterministic policy assertions and isolated fault seams, not by a long-running live V4L2 failure. A target-side failure trace across retry deadlines, backoff, packet progress reset, and healthy sibling output would settle that behavior.
- The non-RK3588/HardwarePC path was compiled and exercised on KA2 silicon; a physical PC and independent non-KA2 silicon were not tested.

Next action: review/code-chief, merge this packet with the other review lenses; no correctness fix is requested by this lens.

Revision: source/package r266; review handoff/save revision r274. The source package was checked against manifest SHA-256 bab525088e0507e9fff1ad099708afcfbf02622f88c82facf0cd2574d7484787. No source delta from r266 was observed.

# Correctness review — loggerd KA2 r10

## Scope and review basis

Reviewed the approved 18-file first-party change set under `system/loggerd/`, including its eight production/build files and ten test/fixture files. The review followed the source paths for DMA-BUF validation/import and retirement, encoderd output-local watchdog/retry/quarantine and segment-drain handling, loggerd rotation/report/audio-queue finalization, and the legacy FFmpeg qRoad configuration and PTS changes. Unrelated dirty workspace paths were excluded.

The controlling user decisions are D-28 (reject invalid full-resolution DMA-BUF input without copy fallback) and D-29 (a failed encoder must not stall rotation or a healthy sibling), as preserved in `intake/report_grilling.md` (SHA-256 `be2b70d7df1922b807604d970a8101bab4034d6c7a36832be00267fa5df24e1f`). The active r10 design manifest is SHA-256 `138f8b5075d6adf7741fb25bf570e225d52b10c32bb9a3fb200b67baa6c43ea1`.

## Traced behavior

- In `encoder/mpp_encoder_input.h:10-24` and `encoder/mpp_encoder.cc:381-406, 409-493`, invalid or non-DMA-BUF inputs are rejected before submission; accepted full-resolution inputs use the DMA-BUF import path, while downscaled qRoad retains RGA. Pending IDs are added after submission and associated with output in `encoder/mpp_encoder.cc:495-593`.
- In `encoderd.cc:51-177, 181-194, 261-293`, retry state and backoff are per output. `encoderd.cc:495-570` shares the 900 ms drain deadline across outputs in a camera thread, quarantines outputs still pending at expiry, and advances healthy outputs. `encoderd.cc:898-946` drives local retry and packet-progress watchdog paths. The drain-timeout test directly observes zero calls to the all-output reopen helper for road, driver, and wide-road after valid output began.
- In `loggerd.cc:107-171, 224-403, 405-597, 693-832`, deterministic rotation uses a monotonic readiness/deadline protocol, finalizes old writers before advancing, maps queued video by segment, and bounds/ages the timestamp-ordered audio queue. The report’s fixed counters and degraded state match the design’s stated reporting contract.
- In `encoder/ffmpeg_encoder.cc:49-77, 86-160`, qRoad uses configured bitrate/GOP/B-frame settings, zero-latency encoder options, and frame-tick PTS reset on open. The dedicated non-RK software path completed on the KA2 host; this is not evidence for physical PC hardware.

No concrete trigger in the reviewed package produces a wrong output, crash, or corrupted segment state that contradicts the traced invariants. This is a source-level correctness result, not a claim that the listed hardware qualification gaps are resolved.

## Validation evidence considered

- Approved build verdict: `build/verdict_build-to-review.json`, SHA-256 `76e69d9d53d296dc5ee5855758aa6eb8083421bcd1d6d78dfc9473d0cf92dbe5`; package manifest: `build/manifest.json`, SHA-256 `bab525088e0507e9fff1ad099708afcfbf02622f88c82facf0cd2574d7484787`.
- Implementation report: `build/reports/report_implementation_r10.md`, SHA-256 `b0208b64f775f35c02588e380a28800538cf7fd1ef797e5f9e59a670008414aa`; aggregate diff: `build/evidence/impl-r10-aggregate.diff`, SHA-256 `f9ddb5a5f8d356e11c7cb4dc6b2a8dea325548a7df588c4723e4099fb5a562a4`.
- Completeness report: `build/reports/report_completeness.md`, SHA-256 `b5d9052c96c69b4df5d52a486e8f23a2365ae31be440a4c90aa27248d2b19f88`; it records 14/14 rows proven, including AC-6’s direct zero all-output-reopen counters.
- Test report: `build/reports/report_tests_r10.md`, SHA-256 `81bb2b2edb12c92f647463f8668cc862d1dbe586b2bbfb97f2b9c1ae447f63d6`; full serial suite log: `build/evidence/tests-r10-ac6-full-r267.log`, SHA-256 `1a331173847b4e933a3be4d38f4e0a4e18e027514705de8d88e9a77a89cec328` (14 passed, 0 skipped, 301.56 seconds); typed probe SHA-256 `52f0d8b2b178d7f26178771efff2003dbad28046cbb5baeda419ff514061bd0b`.
- Runtime report: `build/reports/report_runtime_health_r10.md`, SHA-256 `da999830095353884fd24309c5bb36c17b5ba0d3accf83d3bc38f26a6fc62f84`; r269 smoke log: `build/evidence/runtime-r10-smoke-r269.log`, SHA-256 `556f5ff807a2825d20034656b758e6ca5d56701dbeba388416d4e624cacbaf2f`; typed probe SHA-256 `528ab4e04f008f13fb7417646560641d1ea914727b22b9b2d1ffb5856403e82e`. The report records 106 frames per camera stream, segments 0–2, two healthy rotations, zero known drops, and decodable qRoad video/audio in each segment.

The build verdict carries an Info note that the test report still says runtime refresh is pending. The current r269 runtime report/probe and smoke log are present and bound to the same package source hashes, so I treated that sentence as stale report wording, not a missing runtime result or correctness defect.
