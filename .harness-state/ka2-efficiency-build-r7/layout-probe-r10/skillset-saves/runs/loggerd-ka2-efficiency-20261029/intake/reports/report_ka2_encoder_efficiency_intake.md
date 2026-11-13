# Intake update — KA2 encoder efficiency

**Active revision:** user update 110  
**Supersedes:** D-18 full capture-path robustness review  
**Goal:** make a practical attempt to reduce CPU and memory in the KA2/RK3588 encoder path.

The user accepts the existing best-effort recording semantics. Strict zero-drop D-13/D-16 gates are relaxed for this revision. Continue capturing and reporting frame IDs, losses, output decodability, timestamps, and segment behavior; do not claim lossless behavior. Preserve the normal codec, resolution, frame rate, selected cameras/audio, and route/file compatibility. Do not introduce a material regression under the same synthetic workload.

Scope a concrete encoder efficiency candidate and quantify the outcome on the authorized idle KA2 synthetic path where possible. Keep all tests isolated from production logs, Params, IPC, live camera/sensor input, and on-road recording. Unknown deployed MPP identity/API and buffer/EOS behavior remain limitations to report, but no longer block a bounded experiment. Do not infer a generic MPP guarantee.

Do not continue the whole-path reliability rearchitecture as the active deliverable. The prior architecture report remains evidence and can inform candidate risks. The worktree has prior dirty candidate changes; preserve them and make any new candidate in a clean isolated worktree or isolate the exact diff. Do not overwrite unrelated changes.

The build objective is an observed CPU/RSS improvement or a clear no-gain result. Use the same synthetic input and settings for baseline/candidate; report test conditions, output counts/losses, media decode status, CPU, RSS, and any unsupported measurement. If paired statistical confidence is not practical, label the result as exploratory rather than a quantified production gain.

**Open technical limits:** prior S0 did not establish exact loaded MPP headers/library identity, zero-copy input lifetime, EOS/IDR guarantees, maximum output packet size, or active DMA/RSS headroom. Any candidate that depends on those facts requires a bounded target test and explicit caveat.

**Route:** revise design for a narrow KA2 encoder-efficiency scope, then build and verify only the approved candidate. No unrelated manager, VisionIPC, storage transaction, UI, or logging-schema rearchitecture.


## Intake tool readiness

- Runtime check through the workspace Python 3.13.15 reports Python ready and an active coherent save. Codex hooks are missing and optional; routing is advisory only. No hook configuration was changed.
- The MCP registry is a blank template; external tools are not needed for this repository task, so the intake status is not-needed.
- Project stack detection over system/loggerd found no runtime classification but exceeded its inspection byte limit. This is incomplete detection, not proof of an empty stack. The proposal adds no language runtime or framework; preserve the existing C++/SCons build environment and record stack evidence as incomplete if a gate requires repository-wide detection.

