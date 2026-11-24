# Requirements Brief — KA2 encoder efficiency, r7

**Revision:** r7  
**Run:** loggerd-ka2-efficiency-20261029  
**Intake:** D-22–D-25 in intake/report_grilling.md  
**Prepared for:** design/architect  
**Sources mined:** revised user decisions; KA2 MPP encoder and shared encoder source; encoderd source; saved S0 feasibility/preflight, copy repeatability, and rotation-failure reports.

## Research question and result

Can a narrow KA2/RK3588 encoder change reduce CPU or memory under a synthetic workload while retaining the configured codec/profile and best-effort recording behavior, and can the result be measured despite unknown deployed MPP contracts?

The strongest source hypothesis is the input copy. In default copy mode, each non-downscaled encoder copies a full frame into a preallocated MPP buffer. An opt-in DMA-import path avoids that copy for eligible streams, but the source warns that zero-copy can produce zero-length HEVC packets on some KA2 MPP builds. There is no target performance sample showing whether the input copy dominates CPU or whether imported buffers work reliably on this image. A bounded A/B experiment is justified; the existing import-failure retry/reopen behavior must be addressed before testing.

## Actors and jobs

| Actor | Job to be done | Success looks like | Frequency / volume |
|---|---|---|---|
| KA2 runtime owner | Encode configured camera streams on RK3588 with acceptable best-effort behavior. | Codec, dimensions, FPS, stream selection, timestamps/index meanings, and route/file compatibility remain intact; any loss or decode difference is measured and disclosed. | Continuous runtime; synthetic comparison at a fixed 20 FPS. |
| openpilot maintainer | Reduce encoder CPU and/or memory use. | Matched synthetic input and settings show a favorable resource result, with output differences visible and no unsupported general claim. | A/B target measurements after bounded screening. |

## Requirements

| # | Requirement | Source | Confidence | Affects | Open assumption |
|---|---|---|---|---|---|
| R1 | Focus r7 on CPU and/or memory reduction in the KA2 encoder path; do not make the full capture/logging/Manager redesign the deliverable. | User D-22/D-25, intake/report_grilling.md | reported | Architecture scope and implementation slice | User intent is explicit. |
| R2 | Preserve codec/profile, resolution, frame rate, camera/audio selection, timestamp/index meaning, and route/file compatibility for the comparison. | User D-25; existing encoder/index code | reported plus observed source | Candidate scope and output oracle | No setting or format change is authorized. |
| R3 | Best-effort recording semantics are acceptable for r7; strict whole-process zero-drop D-13/D-16 gates are relaxed. | User D-23, intake/report_grilling.md | reported | Candidate acceptance | Do not claim lossless behavior. |
| R4 | Capture and report input/output IDs, loss counters, decode validity, timestamps, and segment mapping for the same synthetic input schedule. | User D-23/D-24; prior repeatability report | reported | Comparison protocol | Previous run lengths and segment assignments varied; fix the workload before comparing. |
| R5 | To claim an efficiency gain, use matched KA2 baseline/candidate measurements. Preferred protocol remains interleaved paired samples, warm-up, medians/tails, 95% confidence intervals, CPU, process RSS, and available DMA/device-memory metrics. | User D-15, retained by D-23/D-24 | reported | Whether the result supports an efficiency claim | A weaker result can be reported as exploratory, not as a statistically established gain. |
| R6 | Do not alter codec settings to manufacture a resource gain. Report loss and decoder differences against the copy arm. | User D-23/D-25; intake D-7/D-10 | reported | Candidate gate and interpretation | No numeric tolerated loss increase is specified. Recommended default is no worse loss/decode result than the matched copy arm; larger differences go back to the user. |
| R7 | Keep zero-copy test-only until an isolated KA2 run shows nonempty, decodable output for each applicable codec and acceptable loss behavior. | Intake D-4/D-8; current bounded-test authorization | reported | Whether to keep or enable the existing opt-in path | A pass applies only to the tested image/configuration. |
| R8 | Run only bounded synthetic tests with isolated log, Params, and IPC roots while KA2 is idle. No live camera/sensor input, on-road recording, or production-root writes. | User D-6 and D-24 | reported | Test setup and stop conditions | Recheck active processes/resource conditions before each run. |

## Constraints

| # | Constraint | Kind | Source | Confidence | Affects |
|---|---|---|---|---|---|
| C1 | The worktree contains unapproved MPP, encoderd, loggerd, and test edits plus unrelated appbridged and generated-binary changes. | operational | current git status/diff | observed | Use a clean isolated worktree or isolate the exact diff; preserve current edits. |
| C2 | Default MPP mode is copy mode. For non-downscaled frames, encode_frame copies frame_buf_size bytes before encode_put_frame; six buffers are preallocated. | technical | system/loggerd/encoder/mpp_encoder.cc:35-45, 197-211, 443-448; mpp_encoder.h:53-57 | observed | Plausible CPU/memory-bandwidth cost; profile before assuming it dominates. |
| C3 | DMA import is opt-in through ENCODER_ZERO_COPY and disabled for downscaled outputs. Source says some KA2 builds import successfully yet produce zero-length HEVC packets. | technical | mpp_encoder.cc:35-40; loggerd.h:80-100, 125-139 | observed | The on-road road/wideRoad/driver HEVC outputs are the eligible full-size test candidates; qRoad is downscaled H.264 and remains copy. Test H.264 only if an explicit synthetic stream test includes an eligible non-downscaled H.264 output. Import success alone is not qualification. |
| C4 | Zero-copy mode does not allocate the internal frame pool at encoder open. Before the 64-failure threshold, a failed import leaves no MPP buffer; acquire_frame_buffer returns null and encode_frame returns -1. encoderd reopens after 10 consecutive failures, and reopening resets zero_copy_import_failures to zero, so repeated import failures can cause a retry/reopen/drop loop that never reaches the fallback threshold. | technical | mpp_encoder.cc:195-218, 375-431; encoderd.cc:493-510 | observed | Before testing, make fallback/retry and reopen thresholds compatible. The source does not submit an unpopulated fallback frame in this path. |
| C5 | Exact deployed MPP identity/API, input-buffer lifetime, EOS/IDR behavior, packet bound, and active DMA/RSS headroom remain unknown. | technical/operational | saved S0 report and preflight | observed evidence gap | Any target result is image-specific; do not infer generic MPP safety. |
| C6 | R1–R3 preserved IDs within each run, but frame count and segment mapping varied; no performance sample was collected. | quality | build/report_copy_runtime_repeatability.md | observed | Use fixed paced input and compare per-run outputs; prior runs are not a performance baseline. |
| C7 | A previous synthetic run emitted 138 IDs from 144 inputs per output, while later runs differ. | quality | build/reports/report_rotation-gap.md; repeatability report | observed | Measure and disclose best-effort loss; do not generalize from one run. |
| C8 | Default Python is 3.12.3. The executable /data/openpilot/.harness-state/test-work/python313/cpython-3.13.15-linux-aarch64-gnu/bin/python3.13 reports Ready: yes for the active run. Codex hooks are missing/optional. The project-wide stack detector exceeded its byte limit and did not establish a runtime classification. | operational | check_readiness.py and check_runtime.py outputs at intake revision 110; alternate-interpreter readiness recheck | observed | Run validators with the 3.13.15 interpreter; keep hook routing advisory. Do not infer stack classification from the incomplete scan. |

## Non-functional targets

| Target | Number | Source | Confidence | Why this number |
|---|---:|---|---|---|
| Synthetic workload | Fixed 144+ frames per camera at 20 FPS, identical for each arm | Prior test and D-6; exact length to be fixed by build owner | reported/inferred | Prior failure had 144 inputs; R1–R3 had 147–148. Fix one length for the new comparison. |
| Efficiency claim | Favorable paired 95% confidence interval excluding zero for at least one of CPU or RSS | D-15 | reported | Existing quantitative criterion remains for a demonstrated gain; weaker evidence is exploratory. |
| Other resources | No statistically supported regression in the other supported resource metric; report DMA/device memory if available | D-15 | reported | Avoid hiding memory cost behind CPU improvement. |
| Loss/media guard | Report exact per-stream loss IDs/counts and decode status; recommended default is not worse than the paired copy arm | D-23/D-25 | inferred | User permits best-effort semantics but did not quantify acceptable extra loss. |
| Test envelope | 600 s, 768 MiB output, 80% system memory, 80°C; stop if competing camera/loggerd/encoderd/sensor/MPP work appears | Prior S0/D-6 limits | reported | Existing bounded target-test limits. |

## Non-goals

- No full capture-path, VisionIPC, rotation, storage, or Manager redesign.
- No codec, bitrate/QP, resolution, FPS, stream selection, route naming, or public schema change.
- No live camera/sensor, on-road recording, production log/Params/IPC use, or generic MPP claim.
- No unrelated LOC-minimization pass.
- No lossless claim based only on static source or one exploratory test.

## Prior art and evidence

| Source | What it shows | Why it transfers |
|---|---|---|
| system/loggerd/encoder/mpp_encoder.cc at HEAD | Default path copies frames; an environment-gated DMA-import path exists for non-downscaled streams. | Identifies a candidate surface, not target safety or measured cost. |
| build/report_copy_runtime_repeatability.md | Three runs conserved IDs per run, but workload and segment assignment differed; no performance data. | Supports fixed workload and output ledger, not an efficiency claim. |
| build/reports/report_rotation-gap.md | One run lost frames near startup and rotation. | Best-effort baseline evidence; it does not show zero-copy caused those losses. |
| build/reports/report_s0_feasibility.md | Target API identity and memory/lifetime assumptions remain unresolved. | Limits how broadly to generalize a zero-copy trial. |

## Conflicts preserved

| Conflict | Position A | Position B | Who decides |
|---|---|---|---|
| Strict zero-drop versus best effort | D-13/D-16 required exact conservation. | D-23 explicitly relaxes those gates for r7. | Current user decision controls r7. Keep loss diagnostics and make no lossless claim. |
| Potential DMA-copy savings versus compatibility | Copy is default; source warns of zero-length HEVC on some builds. | User permits bounded experimentation. | Architecture/build may test opt-in behavior; default enablement requires measured CPU benefit and decode/loss comparison on the tested image. |

## Open questions

| Question | Blocks | Owner | Latest safe decision point |
|---|---|---|---|
| Does input-copy profiling show enough cost to justify zero-copy? | Candidate choice versus serialization/drain alternatives. | design/architect, then build owner | Before locking the slice. |
| Can this exact test image import buffers for the eligible full-size main HEVC streams and produce decodable output without worse best-effort loss? qRoad is downscaled H.264 and stays copy; H.264 zero-copy applies only if the explicit synthetic stream path includes an eligible non-downscaled output. | Keeping zero-copy as a candidate. | Build/runtime owner on idle KA2 | Before enabling beyond the experiment flag. |
| What additional loss, if any, is acceptable for a resource gain? | Accepting a candidate worse than copy mode. | User | Before final acceptance. |
| Can the stack detector complete a narrow project scan to verify the existing stack lock? | Stack-lock evidence | Commander/runtime environment owner | Before design-to-build submission. |

## Limitations

No target performance profile or paired CPU/RSS/DMA sample exists. S0 did not identify the exact loaded MPP implementation or establish buffer lifetime/EOS/IDR. The zero-copy path is only a test candidate, not a general recommendation. The user relaxed strict zero-drop for r7 but gave no numeric tolerance for extra loss; compare against the copy arm and surface any degradation for user review.
