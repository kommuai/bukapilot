# KA2 capture and recording architecture review

**Run:** loggerd-ka2-efficiency-20261029  
**Review date:** 2026-10-30  
**Scope:** camerad capture through VisionIPC, encoderd and KA2 MPP, cereal transport, loggerd rotation and media/log writers, and Manager lifecycle.  
**Disposition:** architecture recommendation only. No product source was changed and no new target test was run.

## Recommendation

Build a recorder-specific, credit-based data path with explicit frame identity and ownership. Put one recording-session coordinator in loggerd in charge of the session, active streams, segment epochs, frame cutoffs, drain results, and final segment status. Give each platform encoder adapter a strict contract for input ownership, frame-to-packet correlation, keyframes, draining, and failure. Replace the current best-effort rotation and encoded-data delivery assumptions with acknowledged, bounded transitions.

Keep general VisionIPC and messaging behavior for other consumers. Give the recorder its own bounded input slots or leases so a slow encoder cannot read a buffer after camerad has reused it, and so backpressure does not silently become a recording gap. Bound queues by both item count and bytes. Derive capacity from measured KA2 worst-case service times and the supported operating envelope; do not choose a convenient count and call it safe.

A route is complete only when every frame admitted to recording has a matching encoded packet, index entry, and segment assignment, every required stream reaches its terminal watermark, and writers finalize successfully. Startup, saturation, rotation timeout, encoder restart, camera loss, memory pressure, and storage errors must either preserve that contract or make the affected route/segment explicitly incomplete with stream and frame ranges. A bounded system cannot preserve every input through an indefinitely slow or failed consumer. It can prevent silent loss and false claims of completeness.

Do not begin an MPP lifecycle rewrite from the current checkout. The exact KA2 MPP build/API, input-buffer lifetime, EOS/IDR behavior, packet upper bound, and supported memory headroom remain unresolved in the saved S0 evidence. Validate those adapter capabilities on the deployed image before selecting pool sizes, drain limits, or DMA ownership rules. The working tree is dirty and contains an unapproved loggerd/encoderd candidate, so it is not a clean baseline.

## Evidence and confidence

This review combines source inspection with saved synthetic evidence from the prior run. The prior isolated rotation run is concrete evidence of an integrity failure: each of four encoded outputs emitted 138 unique frame IDs out of 144 inputs. All four omitted IDs 1 and 2 at startup; four more were missing around rotation, with one stream also missing ID 100. There were no duplicate or unexpected output IDs. The saved report narrows the boundary loss window but does not prove which exact loss site removed each frame. Later R1–R3 synthetic runs conserved IDs within each individual run, but input counts and segment maps varied; they do not establish a stable clean baseline or exact repeatability.

Findings below are marked **direct** when the code explicitly performs the behavior, **observed** when the saved run measured it, and **unproven target contract** when correctness depends on a runtime/API property not established for KA2.

## Data path today

- camerad dequeues V4L2 frames, assigns a per-camera frame ID, then places buffer indices in a three-entry queue. If full, it removes the oldest queued index, increments a counter, and requeues that buffer.
- camerad publishes a frame descriptor over VisionIPC. In the copy path it copies into a round-robin VisionIPC buffer; in the external-buffer path it exposes a camera buffer. VisionIPC packets identify a server and slot, but have no generation number or reader lease/ack.
- encoderd reads the frame, submits it to a platform encoder, and publishes an EncodeData event. KA2/RK3588 selects MPP; other builds select V4L2 or FFmpeg implementations.
- Encoded events go through PubMaster/ZMQ to loggerd. loggerd writes video and a separate EncodeIndex event into qlog/rlog. Audio arrives on a separate service and is fed to eligible video writers.
- loggerd rotates route files and communicates segment state through Params. Manager starts and stops processes, but the recording components do not share one drain/commit lifecycle.

## Main flaws

| Priority | Finding | Evidence and impact |
|---|---|---|
| Critical | Frame storage can be reused without the recorder acknowledging that it has finished reading it. | VisionIPC selects a ring slot by modulo and sends only slot/server ID plus metadata; the client returns the mapped buffer without a lease. The camera worker also drops the oldest queued frame at depth three. A slot-generation check could catch stale descriptors, but would not prevent reuse while a consumer is reading. See system/camerad/camera/common.cc:111-145, msgq/visionipc/visionipc_server.cc:179-208, msgq/visionipc/visionipc_client.cc:83-123. **Direct.** |
| Critical | The saved rotation experiment lost admitted frames at startup and rollover. | The 144-input synthetic run produced 138 IDs per output. loggerd discards packets until it sees a keyframe; it bounds the future-segment queue by 200 packets and deletes the oldest. Deterministic rotation waits for every expected encoder but has no timeout fallback. encoderd follows segment state by polling three separate Params values and can force segment catch-up. The evidence supports the loss windows, but not each exact loss branch. See system/loggerd/loggerd.cc:57-93, 108-166, 230-254 and build/reports/report_rotation-gap.md. **Observed plus direct source behavior.** |
| Critical | Rotation has no atomic control message or per-stream end-of-segment proof. | loggerd writes rotate state, segment, and epoch as three separate Params keys. encoderd samples them every 100 ms. loggerd increments readiness from configured encoder outputs, while encoderd starts workers only for discovered camerad streams. In deterministic mode, loggerd waits without a timeout. A missing configured stream can leave a segment open indefinitely. No message proves every admitted old-segment frame was output and written before commit. See loggerd.cc:39-72, 328-335; encoderd.cc:353-433, 558-583. **Direct.** |
| Critical | MPP input acceptance and output publication are not represented by one unambiguous result. | The adapter appends metadata after encode_put_frame, but can return -2 later while the frame remains pending. encoderd counts -2 as a dropped frame even though MPP already accepted it. Output metadata is paired by FIFO; this code does not set a per-frame MPP timestamp/token. An empty packet increments the drain count without removing pending metadata. Forced recovery clears pending metadata; close after a timed-out drain also clears it. This can make accounting or packet attribution wrong unless the exact deployed MPP contract rules these cases out. See mpp_encoder.cc:227-282, 292-331, 447-468, 474-555. Exact MPP behavior is an **unproven target contract**. |
| High | Encoded packet delivery to loggerd is best effort, with no receiver acknowledgment. | PubMaster returns the socket send result, but publisher_publish ignores it. The ZMQ publisher uses nonblocking send. A successful call is not proof loggerd received or persisted the event. See encoder.cc:12-40, cereal/messaging/messaging.h:75-79, msgq/impl_zmq.cc:135-143. **Direct.** |
| High | Queue limits do not describe one complete memory or loss budget. | loggerd’s future queue is capped by packet count, not aggregate bytes, and drops its oldest item. A service queue size is not a maximum single encoded packet or a retention guarantee. MPP pending metadata, camera/VisionIPC rings, message queues, loggerd’s future queue, and writer buffers are not one accounted budget. Exact packet maximum and DMA/RSS headroom are absent from S0. **Direct plus unproven target contract.** |
| High | Memory pressure and storage failures can make a route incomplete without changing route status. | At critical memory pressure loggerd skips rlog writes; it may poll while skipping filesystem work and discard the remaining batch if pressure becomes critical. VideoWriter logs short writes and mux errors without returning failure to the session. The route can continue without an incomplete result. See logger.cc:218-245, loggerd.cc:365-389, video_writer.cc:91-143, 221-240. **Direct.** |
| High | Process restart and shutdown do not preserve recording state. | loggerd and encoderd are NativeProcess entries without a restart_if_crash path. ensure_running restarts only when that flag is true, while NativeProcess.start returns if its process object still exists. Offroad shutdown has a VisionIPC order; manager_cleanup stops processes in managed-list order. Neither waits for a shared encoder drain and writer-finalization acknowledgment before a process timeout can force SIGKILL. See process_config.py:80-82, process.py:69-77, 91-145, 150-175, 257-289, manager.py:99-107. **Direct.** |
| High | Camera and encoder failures are logged, not consistently promoted to recording state. | V4L2 requeue failure only logs. Camera queue overflow is counted and periodically printed; it is not sent as a per-frame gap to loggerd. Encoder recovery can discard pending frames and reopen in the same segment without an explicit discontinuity record. See camera/rk.cc:154-191; encoderd.cc:192-209, 485-550. **Direct.** |
| Medium | Frame IDs are local to a camera lifetime, while rotation is route-wide. | V4L2 sequence-derived IDs reset relative to the first published sequence. Equal IDs across cameras do not establish synchronized capture, and a camerad restart can reuse IDs. Use stream identity, producer generation, and timestamps as well as sequence. Treat each camera cutoff as its own watermark. See camera/rk.cc:146-166 and cereal/log.capnp EncodeIndex. **Direct.** |
| Medium | Audio does not participate in the same segment acceptance barrier. | Audio reaches loggerd over another service and is fed to writers separately from video. The current design has no shared per-segment audio sample watermark or commit record that proves continuity alongside video watermarks. The prior candidate test checked for a qcamera audio stream; it did not prove decoded audio-sample continuity. **Evidence gap.** |

## Target architecture

### 1. Define one recording identity

Use a versioned token for each admitted frame:

    FrameToken = {session_uuid, stream_id, producer_generation, sequence64}

Carry capture timestamps and recording epoch in the envelope. Sequence is monotonic within one producer generation. A restart creates a new generation; it never reuses an identity in an existing session. Camera streams get independent IDs and independent watermarks; timestamps express cross-camera timing.

Define **admitted** precisely: the recorder reserved bounded capacity for the frame, assigned a stable token, and owns a safe reference or private copy until the encoder releases it. Frames before admission are pre-roll; frames after admission require a terminal output result. If product behavior requires pre-roll, hold it in a separately bounded pool and include it in the same ledger.

### 2. Give the recorder an ownership-aware input path

Keep general VisionIPC behavior for model/UI consumers. Add a recorder-only path with per-slot generation and explicit credits or leases. The producer cannot reuse a recorder slot until encoderd returns source-release acknowledgment. A slot descriptor contains session, stream, generation, sequence, timestamps, and stable slot generation.

Use bounded storage with both count and byte limits. Derive the maximum from target frame size, active stream count, worst-case drain and storage latency, and available DMA/RSS headroom. If admission cannot reserve capacity, report an explicit recording gap/error with the frame range and reason. Never turn overflow into a successful recording state.

The three-entry camera work queue should not silently evict an old frame on the recorder path. Reserve recorder capacity before declaring a frame admitted, or attach an explicit gap at the capture sequence where it was lost. Requeue errors and capture-sequence holes also become stream-health events.

### 3. Make encoder adapters prove their capabilities

Give each platform adapter the same logical contract:

- submit(token, frame) returns **accepted** only if the adapter owns the input or has a documented release fence.
- source_released(token) means the camera buffer can safely be reused. This is separate from output completion.
- packet(token, bytes, timestamps, keyframe, header, epoch) maps output to the exact input token.
- request_keyframe(epoch) and keyframe_ready(epoch) prove a decodable segment start and required codec header.
- drain(cutoff_token) returns a per-stream terminal watermark and missing/error tokens. Timeout is an error, never successful drain.

Prefer a hardware-provided per-frame PTS or token mapping, validated on the exact target. If the deployed MPP API cannot supply one, the KA2 adapter must prove ordered one-output-per-input behavior, including empty/config packets and EOS, or use a serialized in-flight mode whose cost is measured. Do not infer these properties from current upstream headers or another MPP package.

Keep the current copy path until the adapter’s buffer-release contract is known. Do not enable zero-copy, change pool depth, or reuse input slots based on assumptions about DMA lifetime.

### 4. Replace Params rotation with a recording-session protocol

loggerd owns a typed session control channel. Params may mirror state for UI/diagnostics, but are not the control plane.

Negotiate active streams at session start. Classify each stream as required or optional before recording begins. An absent required stream prevents a complete session; an absent optional stream is recorded as unavailable. Do not count configured outputs and then wait for a different dynamically discovered set.

Use this rotation state machine:

1. **ACTIVE:** loggerd owns session UUID, segment, epoch, and per-stream accepted/output watermarks.
2. **PREPARE:** loggerd requests a new epoch and boundary. Each stream chooses and reports its own old-segment cutoff token; global equality of camera frame IDs is not assumed.
3. **DRAIN:** encoders stop assigning old-epoch input after their cutoff, finish every accepted old-epoch token, and report a contiguous output watermark. New-epoch input uses a bounded staging allowance while the old tail drains.
4. **VERIFY:** loggerd confirms every required old-epoch token has a packet and index entry; writers report successful close/flush; the next segment has codec configuration and a verified keyframe. Audio reports its sample watermark too.
5. **COMMIT:** loggerd writes a versioned segment manifest and marks the segment complete only after all required acknowledgments succeed. A wall-clock boundary alone does not complete a segment.
6. **FAIL/RECOVER:** timeout, missing stream, writer error, or process crash marks the current segment incomplete with per-stream missing ranges. Advance to a fresh epoch and keyframe when healthy. Never guess an offset, silently discard a tail, or wait forever.

The short overlap between old-segment drain and new-segment capture depends on KA2 drain latency. A bounded staging queue or a continuously running encoder with a verified keyframe boundary may work. Select only after measuring the target and proving the buffer budget; if the bound is exceeded, mark the segment incomplete instead of letting backlog grow.

### 5. Make encoded delivery and file completion acknowledged

Use a bounded local recorder channel for encoded data and control. A sender keeps an event until loggerd acknowledges its token after acceptance into the writer transaction. Sequence numbers allow duplicate detection and reconciliation after reconnect. If retaining ZMQ for data, add explicit credits, sequence numbers, retransmission, and receiver acknowledgment; nonblocking PUB send alone is not the contract.

Treat EncodeData and its EncodeIndex metadata as one logical record. Persist a per-segment manifest with session/epoch, required streams, first and last tokens, audio sample ranges, packet/index counts, keyframe/header presence, explicit gap ranges, writer status, and complete/incomplete reason. Write temporary segment files and publish complete status only after successful finalization. Recovery must distinguish a finalized segment from a file that merely exists.

Writer APIs return status for short writes, mux errors, trailer/flush failures, ENOSPC, and memory-pressure skips. Propagate the first failure to the coordinator and stop claiming that segment is complete. Logs remain diagnostics, not the integrity record.

### 6. Give Manager a shared recording lifecycle

Manager remains the process supervisor; loggerd owns the recording session.

- **Startup:** camerad and encoderd register stream capabilities; loggerd starts or resumes only after required streams are negotiated.
- **Crash:** Manager reports a new producer generation. The coordinator closes the prior epoch as incomplete if its watermark is unknown, starts a new epoch, and requires a fresh keyframe. It does not restart the frame counter under the old identity.
- **Offroad and cleanup:** both issue the same STOP_DRAIN operation. Stop frame admission, drain camera/encoder/audio watermarks, finalize writers, commit route status, then stop processes under one shared deadline. SIGKILL after that deadline is recorded as incomplete.
- **Restart:** supervise loggerd/encoderd explicitly, but do not simply enable generic crash restart. Reconcile session UUID, writer state, producer generation, and epoch first.

## Regression proof and rollout gates

The current dirty working tree and saved results are not a no-regression baseline. Preserve them and make a clean, pinned baseline separately before implementation. The earlier rotation run is a known failure; first explain startup IDs 1 and 2 and every rollover ID, or accept that the current reference itself is incomplete. Do not call a candidate regression-free merely because it matches a broken reference.

1. **Contract tests:** model the session protocol with deterministic event ordering. Inject late, duplicate, missing, reordered, and empty packets; absent streams; stale epochs; duplicate acknowledgments; timeout; and crash after every state transition. Assert COMMIT is impossible while any accepted token or writer result is outstanding.
2. **Fake end-to-end recorder:** fixed token ledger across streams and audio; exercise startup, rotations, reconnect, shutdown, queue pressure, and writer failures. Assert exact admitted-token to packet/index equality, no duplicates, monotonic timestamps, keyframe/header at every segment start, continuous audio sample ranges, and correct complete/incomplete manifest.
3. **Failure injection:** camera queue overflow and requeue failure; VisionIPC slot reuse and disconnect; MPP delay, submit rejection, output delay, EOS timeout, empty/config packet, IDR failure; transport disconnect and full queue; loggerd short write, ENOSPC, memory pressure, and process kill. In every case, either preserve the admitted ledger or mark the route incomplete with the correct range.
4. **Exact KA2 qualification:** pin deployed encoder binary, MPP/RGA libraries, package/header provenance, and startup maps. Measure input ownership, packet bounds, keyframe behavior, EOS/drain latency, and active RSS/DMA headroom. The saved S0 review says these facts were unavailable, so this gate remains open.
5. **Platform regression:** build and exercise the shared adapter contract on PC/FFmpeg, TICI/V4L2, and KA2/MPP. Keep the recorder-specific path behind a KA2 feature switch initially.
6. **Canary:** compare old and new recorders in shadow accounting before writing candidate data; then use a parked synthetic/target run, decode every segment, validate audio and logs, and exercise shutdown/recovery. Roll back if any admitted token is missing, duplicated, misassigned, undecodable, or unaccounted for.

**Go/no-go rule:** no rollout until exact token conservation and media/audio decodability pass through startup, rotation, crash recovery, and STOP_DRAIN, with every intentional failure producing a correct incomplete status. Make no performance or memory claim without separate target measurements.

## Limits of this review

The review is source-backed for the checked-in paths above and reuses saved synthetic rotation evidence. It does not establish behavior of the loaded KA2 MPP runtime or run new target experiments. The saved S0 report leaves MPP API/identity, EOS/IDR behavior, VisionIPC lifetime, packet-size upper bound, and active DMA/RSS headroom unresolved. The worktree has unrelated and prior encoder changes; no files in product code were edited for this review.
