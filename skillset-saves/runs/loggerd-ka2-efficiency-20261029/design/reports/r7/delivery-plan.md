# Delivery Plan — KA2 encoder efficiency, r7

**Revision:** 113  
**Objective:** reduce encoder CPU and/or memory cost on KA2/RK3588 using one opt-in MPP DMA-import candidate, with the existing copy path retained as default. Best-effort recording semantics apply; this plan requires loss and media-quality reporting and does not claim lossless behavior.  
**Architecture:** `design/reports/r7/report_architecture.md` — SHA-256 `51198630b43ee77daf23900a5316866b98dca5abf4ebe766362c1c5972aa144e`  
**Interfaces:** `design/reports/r7/report_interfaces.md` — SHA-256 `2d979a9b3ba8b53f65a466ea6be68a3a56c18c76eb2b94fa53f557738e8bbfc2`  
**Security seed:** `design/reports/r7/report_security-seed.md` — SHA-256 `17507b9a8a910d7d07b7352e889ea918f67072f94c89a5b481958eb4c60a7e3d`; typed record `design/artifacts/r7/security_seed.json` — SHA-256 `1329311510eeaa61a6eb8a954dc368af3f06c3e2b5b44205d4c6e799e32013b4`  
**Requirements:** D-22 through D-25 in `intake/report_grilling.md` — SHA-256 `f4de6dc6f661c758f6327c2de1995f702b046b063c9b2572a3a05cfa4aa369bc`

## Scope and success measure

Evaluate the existing `ENCODER_ZERO_COPY` MPP DMA-import path for full-resolution HEVC `road`, `wideRoad`, and `driver`. Compare it with copy mode in a matched synthetic KA2 workload. `qRoad` remains on its existing RGA/downscale/copy path and is measured as a control. The candidate must show a repeatable CPU or memory benefit on the tested image without a material regression in output quality or loss behavior. If the bounded experiment cannot support paired statistics, report it as exploratory and make no quantified efficiency claim.

Preserve codec/profile, resolution, 20 FPS input cadence, configured cameras, audio selection, timestamps, route/file compatibility, and published interfaces. Do not use live camera or sensor input, perform on-road recording, write production log/Params/IPC roots, or enable the candidate by default in this revision. Copy mode remains the production default; `ENCODER_ZERO_COPY=0` is the rollback setting.

The source-derived motivation is about 69.12 MB/s of NV12 input-copy traffic per full-resolution encoder at 20 FPS, or 207.36 MB/s across the three eligible streams. Their six-slot pools have about 62.2 MB aggregate nominal capacity. These are arithmetic estimates, not measured CPU savings, RSS reduction, or DMA use.

## Milestone path

| # | Milestone | Delivery slices | Depends on | Decision gate before it | Target |
| - | --------- | --------------- | ---------- | ----------------------- | ------ |
| 1 | Scoped efficiency candidate and target comparison | One build slice, B1: G0 isolated preflight → G1 fallback fix → G2 fixed synthetic A/B → G3 result/disposition | Pinned project HEAD; approved r7 architecture, interfaces, and security seed | Advance through each gate only after its acceptance check passes. Unknown MPP behavior remains a recorded caveat, not a reason to claim a general guarantee. | One build-to-review package |

The dependency order follows the architecture boundary: isolate and identify the target first; repair the MPP adapter's failure transition before measuring; then compare both modes through the same encoderd/loggerd outputs. No camera/VisionIPC, writer, route, storage, Manager, schema, or broad reliability rearchitecture is planned.

## Single build slice

**B1 — Bounded MPP DMA-import evaluation.** This is one implementation and build slice. G0–G3 below are ordered acceptance gates inside B1, not separate rollouts or independently shippable changes.

### G0 — Isolated build and target envelope

Create a detached worktree from the run's pinned HEAD. Keep the current dirty checkout and all unrelated or previously unapproved changes untouched. Record the pinned commit, clean status, build inputs, output binary hash, and target build result. The current checkout's modifications to encoder, encoderd, loggerd, tests, appbridged, and Params remain outside the candidate unless a later approved package explicitly adopts them.

Before every KA2 run, recheck that camera, loggerd, encoderd, sensor, and MPP activity is idle. Put logs, Params, IPC sockets/shared memory, and run artifacts under unique temporary roots. Record the loaded MPP library/package and relevant ABI/header identity when discoverable; record unknown fields as unknown. Capture available memory, RSS, and DMA/CMA counters before starting. The experiment may proceed with missing MPP identity or unavailable DMA counters only as a bounded, image-specific or exploratory result, with those limits explicit.

### G1 — Immediate and bounded fallback

Add a private test seam for deterministic import and pool-allocation failures. On failed DMA import, switch that encoder instance immediately to copy mode, initialize the existing internal pool once, and copy/submit the current frame. Keep the fallback choice sticky through the encoderd recovery/reopen sequence so resetting the import-failure counter cannot restart repeated import failure and reopen/drop cycles. If pool allocation or the copy fails, use the existing encoder error path and record the failure; do not submit an invalid or uninitialized buffer. Preserve imported-buffer references until pending MPP work is drained and the encoder closes.

Exercise import failure, successful fallback on the same input frame, allocation/copy failure, and recovery/reopen behavior with deterministic tests. The default remains copy; test selection continues to use the existing environment switch.

### G2 — Bounded, same-binary synthetic A/B

Use one candidate binary for both arms so the only intentional treatment difference is `ENCODER_ZERO_COPY=0` versus `=1`. Feed the same 20 FPS synthetic schedule, stream set, timestamps, Params seed, route/segment configuration, and changing per-frame image patterns in both arms. Include full-resolution `road`, `wideRoad`, and `driver`; keep `qRoad` in its unchanged downscale path as a control. Use the same synthetic audio input when the harness supplies one; otherwise record audio behavior as not exercised, without altering audio selection or claiming it was validated.

Run warm-ups, then interleave paired copy and candidate samples. Before the first run, the build lead records a fixed per-run and total time/output ceiling based on available isolated storage and device-idle conditions; do not increase it during the experiment. Recheck idleness before every arm and stop if competing capture, loggerd, encoderd, sensor, or MPP work appears. Each arm has isolated logs, Params, and IPC roots. The existing authorization covers bounded synthetic tests only.

Record, for each arm and each eligible stream:

- submitted/input, emitted, missing, duplicate, and unusable packet counts, including frame IDs;
- HEVC packet non-emptiness, headers/keyframes, decoded frame count, decoded frame identity/content, timestamps, segment assignments, route/file set, and byte sizes;
- encoder CPU time and CPU use normalized by input and emitted frames;
- steady and peak encoder RSS, and available DMA/CMA or device-memory counters, with unavailable metrics named;
- import attempts/failures, fallback transitions, reopen counts, pending depth, and output stalls;
- qRoad control behavior and any synthetic audio results supported by the harness.

Use medians/tails and paired confidence intervals when the bounded sample count supports them. Report sample counts and the interval. If the ceiling or target availability prevents adequate pairing, label findings exploratory. Raw bitstream hashes and size changes are reported, but nondeterministic compressed bytes alone do not establish a content regression; decoded content, metadata, and decodability are the quality checks. A difference in loss, decodability, timestamps, or segment mapping is reported explicitly. No extra-loss tolerance has been specified: a material candidate-versus-copy difference returns to the user rather than being waived by the plan.

### G3 — Result and disposition

Publish a hash-bound comparison with exact image/build identity to the extent available, workload definition, sample count, resource data, per-stream loss/decode results, fallback/reopen behavior, and unsupported measurements. A candidate that emits empty or undecodable HEVC, has corrupted/stale decoded patterns, falls back/reopens repeatedly, shows materially worse loss than copy, or has no supported resource benefit does not qualify. Keep copy as default and record a no-go or exploratory result. A successful result supports only the exact tested image and workload; it does not establish generic MPP lifetime or behavior. Production default changes and on-road testing are outside this plan.

## Acceptance criteria

| ID | Slice | Observable condition | Check that proves it | Acceptor |
| -- | ----- | -------------------- | -------------------- | -------- |
| AC-1 | B1 / G0 isolated build and target envelope | A detached worktree is built from the pinned HEAD without modifying the dirty shared checkout; the target binary and available MPP/runtime identity are recorded; every test root is isolated and the target is idle at each run start. | Review the hash-bound preflight record for pinned commit, clean worktree status, build log and binary hash, idle-state observations, unique temporary log/Params/IPC paths, initial resource counters, and explicit known/unknown MPP fields. | KA2 build lead |
| AC-2 | B1 / G1 immediate fallback | Injected import failure switches to copy before submitting that frame; successful fallback encodes the same current input; the copy mode remains sticky across recovery/reopen; allocation/copy failures follow the existing error path without invalid-buffer submission. | Run deterministic adapter/encoderd fault-injection coverage for import success/failure, pool allocation failure, copy failure, and repeated reopen; inspect per-stream transition/error counters and verify no repeated import/reopen loop. | Encoder implementation lead |
| AC-3 | B1 / G2 output and compatibility comparison | Both arms use identical settings and synthetic schedule; route/file format, configured camera/audio selection, frame metadata, timestamp and segment behavior are reported; candidate HEVC output is nonempty and decodable, with frame loss and decoded content compared to copy and qRoad retained as a control. | Review paired run manifests and output analysis for input/output IDs, loss/duplicate/unusable counts, decode and pattern results, packet/header checks, timestamps, segment/file mappings, byte sizes, qRoad results, and audio coverage or an explicit not-exercised result. | KA2 test lead |
| AC-4 | B1 / G2 resource comparison | CPU and supported memory measures are collected for both arms under the same workload; any efficiency claim is supported by paired results, otherwise the report is explicitly exploratory or no-gain. | Review raw per-run CPU/RSS/DMA-CMA samples, normalized rates, warm-up and arm order, sample counts, medians/tails, and paired confidence interval where supported; confirm unsupported measures and ceilings are disclosed. | Performance lead |
| AC-5 | B1 / G3 disposition and rollback | Copy remains default; the report states qualify, reject, or exploratory for the exact tested image; `ENCODER_ZERO_COPY=0` is documented and verified as the rollback arm; no live-camera/on-road or production-root test is performed. | Check the final hash-bound result and configuration evidence against the copy-default rule, candidate gates, isolation log, and rollback invocation; confirm any production enablement is explicitly deferred to a later owner decision. | Design/Commander |

## Rollout and rollback shape

| Stage | Audience | Mechanism | Rollback | Trigger to advance |
| ----- | -------- | --------- | -------- | ------------------ |
| Isolated test build | KA2 synthetic test harness only | Detached pinned worktree; separate binary and temporary roots; copy and candidate arms selected with the existing environment flag | Stop the run and set `ENCODER_ZERO_COPY=0`; discard isolated outputs after evidence capture under the run retention rules | S0 and S1 pass; idle preflight passes before every run |
| Candidate qualification | Same synthetic harness on the same KA2 image | Interleaved same-binary A/B; no live camera or sensor input | Run copy arm with `ENCODER_ZERO_COPY=0`; keep production default unchanged | S2 output and resource evidence meets AC-3/AC-4, or record exploratory/no-go |
| Production selection | Not included in this plan | Requires a later explicit decision for the exact tested image; no production config/default change here | `ENCODER_ZERO_COPY=0` | Separate approved rollout plan after image-specific qualification |

## Risk handling

| Severity | Risk | Kind | Consequence if it lands | Mitigation | Owner |
| -------- | ---- | ---- | ----------------------- | ---------- | ----- |
| Major | MPP import may succeed while HEVC output is empty or undecodable on the deployed image. | Technical dependency | Candidate cannot provide usable recordings. | Verify packets, headers, decoder output, and frame patterns on the target; reject qualification and retain copy mode if they fail. | KA2 test lead |
| Major | MPP may read an imported VisionIPC allocation after the producer reuses it. | Technical dependency | Stale or corrupted frames can be emitted despite a successful import call. | Use changing per-frame patterns and decoded identity checks; report that synthetic success cannot prove a generic completion fence. | Encoder implementation lead |
| Major | Persistent import failure can interact with the 10-failure reopen path and reset the 64-failure fallback counter. | Technical dependency | Repeated reopen/drop loop prevents reaching fallback and wastes CPU. | Make fallback immediate and sticky across recovery; inject failures and count transitions/reopens before A/B. | Encoder implementation lead |
| Major | DMA/CMA accounting, MPP identity, packet bounds, and RSS attribution may be unavailable or image-specific. | Technical dependency | A memory benefit or portability claim may be unsupported. | Record available counters and unknowns; make only measured, image-specific claims; mark insufficient sampling exploratory. | Performance lead |
| Major | Another production workload starts during the synthetic test. | Rollout | Resource contention can invalidate measurements or collide with production activity. | Recheck idle state immediately before each run and stop when camera/loggerd/encoderd/sensor/MPP activity appears; keep all test roots isolated. | KA2 test lead |
| Minor | The current shared checkout contains dirty and unrelated modifications. | Technical dependency | Candidate evidence may accidentally include unreviewed changes. | Build and edit only in a detached worktree from pinned HEAD; hash the exact diff and binary; preserve the shared tree. | Build lead |
| Minor | Bounded output/time ceilings may not support stable paired intervals. | Staffing / environment | Result can be too noisy to support a quantified gain. | Set ceilings before testing, record actual pairs, use paired statistics where supported, and report exploratory otherwise. | Performance lead |

The r7 security seed identifies no new or moved adversarial trust boundary. It retains DMA lifetime, output integrity, and resource risks and does not claim that existing MPP handling is secure. Reassess if production default, privilege, interface exposure, image, or input actor changes.

## Decision Register

| Decision | Status | Chosen | Source | Rejected options (why) | Owner | Reopen trigger / latest safe point |
| -------- | ------ | ------ | ------ | ---------------------- | ----- | ---------------------------------- |
| D-22 objective and scope | resolved | Improve KA2 encoder CPU/memory efficiency with a practical encoder-path attempt; do not continue the broad capture-path rearchitecture as the active scope. | user | Broad camera/VisionIPC, storage, or Manager redesign; not required for this objective. | Design/Commander | Reopen only if the user changes scope; latest safe point is before implementation spec approval. |
| D-23 recording semantics | resolved | Keep best-effort semantics and report per-stream losses, decode quality, timestamps, and segments; do not claim lossless behavior. | user | Strict zero-drop gate inherited from earlier decisions; superseded for r7. | Design/Commander | Reopen if user requires a zero-drop guarantee; latest safe point is before implementation spec approval. |
| D-24 experiment authorization | resolved | Bounded synthetic KA2 A/B is allowed when idle, isolated, and within recorded time/output ceilings; no live input or production-root writes. | user | Treat unknown MPP behavior as an absolute blocker; the user permits bounded practical evidence with caveats. | KA2 test lead | Reopen if target is not idle, isolation cannot be enforced, or a proposed run exceeds its ceiling; decide before that run. |
| D-25 compatibility contract | resolved | Preserve codec/profile, resolution/FPS, camera/audio selection, and route/file compatibility; scope changes to MPP input choice and fallback behavior. | user | Changing capture settings or route/message/file behavior to obtain a nominal gain. | Encoder implementation lead | Reopen only if the candidate cannot be evaluated without a compatibility change; latest safe point is before implementation spec approval. |
| Candidate and default | resolved | Evaluate zero-copy only for full-resolution road/wideRoad/driver HEVC; qRoad remains control; copy stays default pending qualification. | prior-artifact | Enabling zero-copy by default before target evidence; importing qRoad despite its downscale path. | Design/Commander | Reopen after S2 produces image-specific evidence and before any production selection. |
| Production selection | deferred | No production enablement or on-road validation in r7; retain `ENCODER_ZERO_COPY=0` as rollback. | delegated-default | Automatic enablement from a synthetic result; a separate owner decision is required. | User / product owner | Reopen after a qualified result, in a separate rollout decision before changing any production configuration. |
| Broader optimization and LOC reduction | yagni-deferred | Do not optimize unrelated utilities, queues, route writing, or perform LOC minimization in this revision. | user | Expanding scope without evidence that those paths dominate encoder CPU/RSS. | Design/Commander | Reopen if profiling shows material cost outside MPP input handling; decide before expanding implementation scope. |

## Migration / deprecation

Not applicable. No existing public interface, message, schema, route format, file naming, service, or consumer is removed or replaced. The existing copy mode remains the default and the existing environment switch supplies the test-arm selection and rollback.

## Non-goals and YAGNI deferrals

| Deferred | Why it is safe to defer | Reopen trigger | Owner |
| -------- | ----------------------- | -------------- | ----- |
| Changing camera/VisionIPC ownership or adding a completion-fence contract | This revision evaluates only the existing import path; generic lifetime safety cannot be established by rearchitecting the producer in the bounded scope. | Pattern/decode evidence shows reuse corruption, or a future production requirement demands a proven lifetime contract. | Design/Commander |
| Changing loggerd queues, rotation, storage transactions, or Manager lifecycle | These are outside the scoped CPU/RSS candidate and would confound the one-candidate comparison. | Target profiling attributes a material portion of measured cost to these paths. | Build lead |
| Generic MPP support or cross-image enablement | The loaded MPP implementation and buffer/EOS contracts are not established. | A separate per-image qualification program supplies ABI, lifetime, EOS/IDR, packet bound, and resource evidence. | KA2 platform owner |
| Production default change and on-road test | The user authorized bounded synthetic testing only; a synthetic result does not qualify live capture behavior. | A separate explicit rollout and live-capture authorization is recorded after S2. | User / product owner |

## Environment and evidence limits

Python 3.13.15 is available for Python-dependent package checks; Codex hooks are missing and advisory. The source stack detector exceeded its total-byte inspection limit, so stack detection remains incomplete; this plan adds no runtime or framework. Prior synthetic runs had changing workloads/segment maps and no performance samples, so they are not a performance baseline. A prior run lost frame IDs at startup/rollover; r7 measures and reports losses against the matched copy arm without asserting zero loss.

Unknowns remain: exact loaded MPP/API identity, zero-copy buffer completion/lifetime, EOS/IDR behavior, packet bound, and DMA/RSS headroom. Available target evidence can qualify only the exact tested image and workload. A successful import call or synthetic run does not prove a generic MPP guarantee.
