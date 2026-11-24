# Checkpoint — investigation r8 ready for cross-stage review

## Boundary and result

Investigation r8 is assembled under `investigation-review` for the active SideCamera run. Its schema-2 manifest passed the mechanical self-check with no failures, missing keys, or warnings. The current step is Admiral’s cross-stage gate; the package has not been returned to BUILD.

The r2.3 runtime capture is reused as the executed reproduction. AC-4 remains failed: 6,050 matched IDs over 302.471744064 seconds, mean 19.998562241636 fps, maximum EOF gap 90.981562 ms, and 716 of 6,049 intervals above 60 ms. Publication and cleanup passed; startup Params all-false remains unverified. Source hashes are unchanged.

## Investigation conclusion

No causal mechanism is established. The evidence bounds suspects to FFmpeg decode/filter/output pacing or synchronization, with host scheduling still possible. Sustained pipe backpressure, primary runqueue delay, subscriber backlog, a repeatable flush effect, and SOE PTS as EOF are unsupported. GDB stacks are callsite observations only.

The package proposes a future, maximum 600-frame no-payload timeline correlating V4L2 dequeue sequence/completion, FFmpeg decoder/filter completion, raw-pipe writes/full-read EOF, and per-thread scheduling during a >60 ms gap. It does not authorize that capture or any source instrumentation/tracer. BUILD must submit the plan to Admiral before new measurement work. No speculative source fix is proposed.

## Evidence

- Investigation package: `investigation/reports/investigation-package.md` — SHA-256 `24ce991fb29bf826b6dcfebf6b89ec63907b145028983624e50acfc3b9f07f4b`
- Schema-2 manifest: `investigation/manifest.json` — SHA-256 `e37ae87160cd26cadc70b430a01783c112a2283a9fb44fa8f2ad0e4aedea496a`
- Evidence chain: `investigation/evidence/evidence-chain-r8.log` — SHA-256 `8a6456d11053abf666f3517f9122232bf3213d21e2fe7f04a182c4ed88b11ab8`
- Self-check checkpoint: `investigation/reports/report_checkpoint-self-check-r8.md` — SHA-256 `e4c9f4fa6928acc8c3fda2e36a83cccfa86561633929c1f02774711418036ad1`
- The prior approved r7 package, manifest, and cross-stage verdict remain byte-preserved at their revisioned archive paths.

## Next action

Submit the r8 manifest to Admiral’s `investigation-review` cross-stage gate. If approved, route the bounded measurement plan to BUILD for an explicit authorization decision before any new capture or tracer; if revised or escalated, follow the gate verdict and preserve its lineage.
