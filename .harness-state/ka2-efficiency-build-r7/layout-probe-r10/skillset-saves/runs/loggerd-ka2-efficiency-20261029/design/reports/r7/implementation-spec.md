# Implementation Spec — KA2 encoder efficiency evaluation, r7

**Revision:** r7 (design run revision 114)  
**Design inputs:** architecture r7; interface inventory r7; delivery plan r7; security seed r7.  
**Detected stack:** Incomplete. The project stack detector exceeded its total-byte inspection limit. No runtime or framework is proposed. Commander should record the sanctioned unchanged-stack lock: “no new runtime or framework - existing stack unchanged.” The target C++/MPP toolchain and deployed MPP package identity must be recorded during G0 where discoverable; unknown values stay unknown.

## Slice order

| # | Slice id | Depends on | Ships behind | Acceptance owner |
| - | -------- | ---------- | ------------ | ---------------- |
| 1 | `slice.mpp.bounded_dma_import_evaluation` | Approved r7 architecture, interface contract, security seed, and plan; inside the slice, G0 target preflight precedes G1 fallback repair, which precedes G2 comparison and G3 disposition. | Existing `ENCODER_ZERO_COPY` switch for the isolated candidate arm only; default remains copy (`0`). | Design/Commander, with KA2 build, encoder, test, and performance leads accepting their respective evidence. |

## Slice

### slice.mpp.bounded_dma_import_evaluation

**Architecture mapping:** r7 MPP adapter candidate and unchanged encoderd scheduling/loggerd output boundaries. Plan milestone 1, B1. The tested target is the exact available KA2/RK3588 image; results do not generalize to other images.

**Modules owned:** In the detached candidate worktree only:
- `system/loggerd/encoder/mpp_encoder.cc` and `.h`: MPP input choice, private sticky fallback state, internal pool initialization, imported-buffer references, and private test seam.
- `system/loggerd/encoderd.cc`: only the state needed to preserve copy fallback across recovery reopens if that state cannot remain owned by the MPP encoder instance.
- `system/loggerd/tests/test_loggerd.py` or a narrowly scoped adjacent test file: deterministic target synthetic input, isolated fault-injection and A/B harness coverage, and output-analysis assertions.

Do not edit `loggerd.cc`, camera/VisionIPC producers, writer/route behavior, cereal schemas, Params keys, public encoder signatures, other platform adapters, Manager behavior, or unrelated dirty checkout files. If the implementation cannot meet the existing interface contract within these modules, return to Architect before broadening ownership.

**Depends on:** No earlier build slice. G0 must establish a clean detached worktree from the run’s pinned HEAD before any implementation or target test. The existing shared checkout is dirty; none of its current modifications may be silently included in the candidate.

**Data / migration changes:** None. No schema, file, route, Params, or persistent data changes. The candidate may add private encoder state and test-only fault injection; it must not add a public service, production configuration, new dependency, or runtime/framework.

**Contract tests:** There are no public endpoint or message-contract changes. Test the private MPP adapter behavior and preservation of existing output contracts:
- An injected import failure switches to copy before the current input frame is submitted; that same frame is copied and submitted once if allocation/copy succeeds.
- Pool-allocation or copy failure returns through the existing encode error path and never submits a buffer without valid frame data.
- Once fallback is latched, recovery reopen and segment close/open do not re-enable import for that encoder instance.
- Imported references remain valid until pending MPP work is drained and the encoder closes; a successful import call alone is not considered proof of input lifetime.
- The candidate and copy arms retain the existing encoded event shape, frame metadata, timestamps, route files, camera/audio selection, resolution, frame cadence, and codec settings.
- Nonempty packets, required headers/keyframes, decodability, decoded frame identity/content, loss, duplicate IDs, segment assignment, and qRoad control behavior are measured and reported. No zero-loss contract is introduced.

**Proof plan:**
1. Add failing deterministic cases for import failure on the first frame, allocation/copy failure, repeated recovery reopen, and sticky fallback. The tests must fail against the current behavior because the zero-copy path can return `-1` before allocating a copy pool and encoderd can reopen after ten failures while open resets the 64-failure counter.
2. Make the smallest change in the MPP adapter that latches copy mode on the first import failure, initializes the existing pool once, and processes that current frame by copying. Keep the latch through encoder reopen. If the latch belongs to the encoder object, do not clear it in `encoder_open`; if the encoderd owner must preserve it, update only that private recovery seam.
3. Prove pool/allocation failure and copy failure do not submit invalid input. Keep existing imported references until drain/close; do not release them early to reduce memory.
4. Refactor only within the owned modules while the deterministic cases remain green. Preserve default copy mode; do not change encoder settings or public outputs.
5. After G0 and G1 pass, run the same-binary, fixed-workload target A/B: `ENCODER_ZERO_COPY=0` and `=1`, with identical synthetic inputs. Use deterministic changing patterns per frame (replace any uninitialized/random frame content in the harness). Cover full-size road, wideRoad, and driver HEVC; qRoad remains on its existing downscale/copy path as a control. Use the same timestamps, Params seed, segment setup, audio input if supplied, and isolated test roots for both arms.
6. Warm up, interleave paired samples, and before the first run record the fixed per-run and total time/output ceilings based on available isolated storage and idle conditions. Recheck idleness before every arm. Stop if camerad, loggerd, encoderd, sensor, or other MPP work appears. Do not raise the ceilings mid-run.
7. For each stream and arm, record input/submitted/emitted/missing/duplicate/unusable frame and packet counts, frame IDs, nonempty packet/header/keyframe checks, decoded count and decoded pattern identity, timestamps, segment mapping, route/file set and sizes, stalls, fallback transitions, import attempts/failures, reopen count, and pending depth. Record encoder CPU time/use normalized by input and emitted frames, steady/peak encoder RSS, and available DMA/CMA counters. Name metrics that are unavailable.
8. Compare paired medians/tails and paired confidence intervals where sample counts support them. If the bounded ceiling or host availability prevents adequate pairing, call the result exploratory and make no quantified benefit claim. Raw bitstream hash/size differences alone do not prove a content regression; decoded content, metadata, and decodability do. Report any loss or decode difference against the copy arm; no extra-loss tolerance is defined, so a material difference returns to the owner for a decision.
9. Publish one hash-bound disposition for the tested image and workload: qualifies for that image only, rejected, or exploratory/no-gain. Copy stays default in all outcomes in r7.

**Smallest green target:** Private import/fallback behavior tests plus isolated target synthetic output validation. The tests must exercise the current-frame fallback and reopen latch, not merely inspect state or environment variables.

**One meaningful verification command:** `pytest -q system/loggerd/tests/test_loggerd.py -k 'mpp_import_fallback or mpp_synthetic_ab'` (the slice must add these focused cases, or an equivalent single focused test module and command if the existing test layout cannot host the seam). Run from the detached pinned worktree with unique temporary roots and an explicit test output ceiling. This command does not replace the per-arm performance record or output decoder analysis.

**Refactor boundary:** Only the MPP encoder’s input-selection/fallback state and the minimum encoderd reopen seam needed to retain that state, plus the focused test harness. No queue, capture, storage, rotation, writer, route, or broader lifecycle refactor.

**Rollback:** Stop the isolated run and select `ENCODER_ZERO_COPY=0`. The candidate is opt-in and remains disabled by default; reverting the candidate diff returns to the copy path. Do not modify production configuration or enable the flag in this slice.

**Acceptance evidence:**  
- **G0 / KA2 build lead:** hash-bound record of pinned HEAD, clean detached worktree, candidate diff, build inputs and result, binary hash, target/image identity, available MPP package/ABI identity, idle observations before every run, isolated log/Params/IPC paths, and before-run resource counters. Missing MPP identity or DMA counters are explicit limitations; proceed only as bounded image-specific or exploratory evidence.
- **G1 / encoder lead:** deterministic failure-case results showing immediate same-frame copy, sticky mode across reopen, valid error behavior on allocation/copy failure, no invalid submission, and preserved buffer references until drain/close.
- **G2 / KA2 test and performance leads:** fixed workload and ceiling, paired arm manifests, per-stream loss/output/decode/timestamp/segment data, qRoad control, audio exercised or explicitly not exercised, raw CPU/RSS/DMA/CMA samples, sample count/order/warm-up, supported intervals, and unavailable metrics.
- **G3 / Design/Commander:** hash-bound final disposition with exact tested image and workload, copy-default confirmation, rollback setting, and any unresolved risk stated. Acceptance does not imply lossless recording or a generic MPP guarantee.

**Acceptance command after implementation:** `pytest -q system/loggerd/tests/test_loggerd.py -k 'mpp_import_fallback or mpp_synthetic_ab'`; the KA2 performance and decoded-output artifacts are separately required even when this command passes. Python 3.13.15 is available for Python-dependent checks. This spec did not run tests.

## Operational constraints

| Constraint | Slices affected | Owner | Removal criterion |
| ---------- | --------------- | ----- | ----------------- |
| Keep `ENCODER_ZERO_COPY=0` as the default and rollback arm; opt-in is only for isolated synthetic A/B. | B1 | Design/Commander and KA2 test lead | The test selection ends with the run; any default change requires a later owner decision for the exact qualified image. |
| Build from a clean detached pinned worktree; preserve all existing dirty and unrelated checkout changes. | B1 / G0 | Build lead | Candidate artifacts are captured and reviewed; shared checkout remains untouched. |
| Recheck KA2 idleness before every arm; use unique temporary logs, Params, IPC sockets/shared memory and run artifacts. Never use live camera/sensor input, on-road recording, or production roots. Stop on competing camera/loggerd/encoderd/sensor/MPP activity. | B1 / G0, G2 | KA2 test lead | Each bounded run ends or stops; cleanup follows run retention rules after evidence capture. |
| Set fixed run and aggregate time/output ceilings before the first test; include available storage/headroom and stop conditions in the run record. | B1 / G2 | KA2 test lead | The paired run set ends; do not enlarge ceilings during a run. |
| MPP implementation/API, input completion lifetime, EOS/IDR behavior, packet bounds, and DMA/RSS accounting are unknown. Report each as known, unknown, or unavailable; make claims only for the exact tested image and workload. | B1 / G0–G3 | KA2 build and performance leads | Evidence review closes the bounded experiment; generic support remains out of scope. |
| Keep any fault-injection seam private and test-scoped. Do not add a runtime dependency or production environment bypass. | B1 / G1 | Encoder lead | The permanent regression cases use a private seam without changing production selection or interfaces. |
| Do not claim lossless output. Report startup/rollover loss and compare candidate losses against the matched copy arm. Material candidate-versus-copy loss or decode differences need owner disposition. | B1 / G2–G3 | KA2 test lead and Design/Commander | Final image-specific report is accepted or marked rejected/exploratory. |
| The source stack scan is incomplete; no runtime/framework is proposed. Codex hooks are missing/optional. | B1 | Design/Commander | Stack lock records unchanged stack with the permitted fallback; hooks are not a build prerequisite. |

## Non-goals

- No camera/VisionIPC producer changes, generic MPP completion-fence design, new capture ownership, or cross-image guarantee; these belong to a later architecture if evidence requires them.
- No loggerd queue, rotation, storage transaction, route/file format, Manager lifecycle, schema, or broad reliability redesign.
- No codec/profile, bitrate, resolution, FPS, camera/audio selection, public interface, or route compatibility change.
- No production default change, live camera/sensor run, on-road test, or production-root writes.
- No claim that source-derived copy traffic or nominal pool capacity equals measured CPU, RSS, or DMA savings.

## Blocking decisions

| Decision | Owner | Blocks slices | Latest safe decision point |
| -------- | ----- | ------------- | -------------------------- |
| None on implementation branch or contract: copy default, candidate scope, fallback behavior, best-effort comparison, and bounded synthetic-only authorization are settled upstream by D-22–D-25 and r7 architecture/plan. | N/A | None | Not applicable. |

Unknown target details do not block this scoped experiment; they constrain the claim. If the target is not idle, isolation cannot be enforced, the required test exceeds its recorded ceiling, candidate compatibility requires changing a protected setting/interface, or the owner wants production enablement, stop and return to Commander/Admiral before proceeding.

