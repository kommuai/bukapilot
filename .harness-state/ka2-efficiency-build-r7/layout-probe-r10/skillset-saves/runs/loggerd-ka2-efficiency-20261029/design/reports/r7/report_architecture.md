# Architecture decision — KA2 MPP input-copy efficiency, r7

**Goal:** reduce encoder CPU and/or memory cost on KA2/RK3588 while keeping current codec settings and best-effort recording semantics.  
**Decision:** evaluate the existing opt-in MPP DMA-import path as one isolated candidate against the current copy path. Keep copy mode as the default until the tested image shows a repeatable resource benefit and acceptable output. The architecture changes no route format, cereal schema, manager protocol, camera selection, audio selection, resolution, frame rate, or bitrate.

## Candidate and rationale

For each full-resolution road, wideRoad, and driver encoder, copy mode moves one 3,456,000-byte NV12 frame into a preallocated MPP buffer at 20 FPS. That is 69.12 MB/s of frame bytes copied per encoder, or 207.36 MB/s across the three encoders. Six full-resolution internal MPP input buffers per encoder have nominal capacity of about 62.2 MB total across those three outputs. These figures are source-derived traffic and allocation arithmetic. They are not measured CPU savings, process RSS reduction, or DMA use.

The existing ENCODER_ZERO_COPY option imports eligible VisionIPC DMA buffers for MPP. The code disables this path for downscaled outputs, so qRoad remains on its RGA/copy path. The main opportunity is to avoid the full-frame memcpy and the three internal six-buffer pools. The benefit is a hypothesis until a matched KA2 measurement confirms it.

The source warns that some KA2 MPP builds accept DMA import but produce zero-length HEVC packets. The deployed MPP identity, API pairing, input-buffer lifetime, EOS behavior, packet limits, and DMA/RSS accounting are not established in the saved S0 record. A successful import call alone does not qualify the candidate.

## Components and boundaries

- **camerad/VisionIPC:** no changes. It continues to publish existing frames and maintain the current ring.
- **encoderd:** unchanged scheduling, stream selection, timestamps, segment indexes, and message publishing. It creates the existing MPP encoder for each configured output.
- **MPP adapter:** owns the candidate input choice. Copy mode allocates and copies into its preallocated internal pool. Candidate mode imports an eligible full-resolution VisionIPC buffer and attaches that buffer to the MPP frame.
- **loggerd/writers:** no changes. It receives the same EncodeData and EncodeIndex records and writes the same route files.
- **Manager:** no changes. The candidate is selected through the existing ENCODER_ZERO_COPY environment switch for the isolated A/B arm.
- **Test harness:** supplies fixed synthetic frames through isolated IPC/Params/log roots and captures resource/output measurements. It does not use live camera or sensor input.

The existing environment switch is internal to the encoder process. The candidate introduces no public message, schema, or service endpoint. The interface inventory is therefore change=none.

## Input and lifetime model

Copy mode is the reference: encoderd receives a shared VisionIPC buffer, copies frame bytes into an MPP-owned buffer, and submits that buffer. The MPP buffer pool has six slots per encoder.

In candidate mode, MPP receives an imported reference to the VisionIPC DMA allocation. The import map is keyed by file descriptor. The camera/VisionIPC producer continues its existing ring reuse behavior; it does not lease a slot to MPP. The candidate therefore depends on the deployed MPP implementation finishing its read before that slot is overwritten. Source code does not establish that completion bound. The synthetic workload must change visible image content by frame so overwritten or stale reads can be detected in decoded output.

## Import failure and fallback

The current import-failure sequence needs a focused repair before an A/B test:

1. Zero-copy mode does not allocate the internal six-buffer pool at open.
2. If both import attempts fail, the adapter increments its import-failure count. Before 64 failures, it has no imported buffer and no internal pool; acquire_frame_buffer returns null, so encode_frame returns -1.
3. encoderd reopens after ten consecutive encoder failures. Reopening resets the import-failure counter, so a persistent import failure can repeatedly restart the encoder and drop inputs before the counter reaches 64.

The candidate implementation should transition immediately to copy mode on an import failure, allocate the internal pool once, copy and submit the current frame, and keep that encoder instance in copy mode through subsequent reopen operations. If fallback allocation fails, return the existing encoder error and report it; never submit a buffer without valid frame contents. Existing imported-buffer references remain held until pending MPP work is drained and the encoder closes. Copy mode stays the constructor default; the test arm opts in.

This fallback fixes a retry/reopen loop. It does not prove the zero-copy input-lifetime contract or HEVC behavior.

## Failure behavior

- **Import fails:** switch that encoder instance to copy mode immediately, initialize its pool, and process the current frame from the copy path. Record one mode transition and the stream name.
- **Pool allocation or copy fails:** report encode failure through the current error path. The test marks the candidate arm failed; it does not retry the same import indefinitely.
- **MPP produces empty or non-decodable HEVC output:** candidate fails qualification for that tested image and stays disabled. Empty output is counted as unusable output in the comparison.
- **Input reuse corrupts a frame:** per-frame pattern/decode comparison detects it; candidate fails qualification.
- **Candidate output loses more frames than the matched copy arm:** reject enablement for that image. Best-effort semantics remain; the test reports exact loss and does not claim lossless operation.
- **CPU/RSS improves without acceptable output:** reject the candidate. Resource benefit alone is not sufficient.
- **No material resource benefit:** keep copy mode as the default and record the no-gain result.

## Test and acceptance architecture

Run on the idle KA2 using the existing synthetic test pathway. Recheck active camera, encoderd, loggerd, sensor, and MPP work immediately before each test. Use isolated temporary logs, Params, and IPC paths. Do not use production roots, live camera/sensor input, or on-road recording. Keep the current dirty checkout untouched; build from a clean pinned worktree and isolate the candidate diff.

The copy and candidate arms use the same binary, settings, frame schedule, stream set, segment length, timestamps, and synthetic frame contents. Alternate arm order after warm-up. The candidate uses ENCODER_ZERO_COPY=1; the copy arm uses ENCODER_ZERO_COPY=0. Test the three full-resolution HEVC streams that are eligible for import. The qRoad downscale output remains on its existing copy/RGA path and is checked as a control.

Record for each arm:
- encoder process CPU time and CPU use normalized by input and emitted frames;
- peak and steady process RSS, plus available system DMA/CMA metrics;
- per-stream submitted, emitted, missing, duplicate, and unusable packet counts;
- keyframe/header presence, decoded frame count, frame content/identity, timestamps, segment assignment, and output file sizes;
- import failures, fallback transitions, encoder reopen counts, pending depth, and output stalls.

For a production-efficiency claim, use matched interleaved samples, warm-up, medians/tails, and a paired confidence interval. Report the interval and supported resource metrics. If sample/output ceilings or target availability prevent adequate paired sampling, report the result as exploratory and make no quantified gain claim. Candidate loss/decode results must be no worse than the copy arm beyond paired variation. A material difference returns to the user for a decision; no tolerance for extra loss has been specified.

## Rollout and rollback

Qualification is image-specific. Keep the default copy path and the existing opt-in flag as the rollback. Only enable zero-copy for the tested KA2 image after the fallback path and synthetic output pass, HEVC packets are nonempty and decodable, loss behavior is acceptable against copy mode, and CPU or memory benefit is measured. If the image changes MPP/RGA libraries, kernel, allocator, or camera buffers, repeat the bounded test. Do not claim the result applies to all KA2 builds.

## Open risks

1. The deployed MPP ABI and implementation are not matched to saved headers/package evidence.
2. No source-proven completion fence states when MPP has finished reading an imported VisionIPC buffer.
3. The source warning about zero-length HEVC output may apply to the tested image.
4. DMA/CMA and imported-buffer RSS accounting are not available in the saved preflight.
5. The previous copy experiments had varying workloads and segment maps and no resource samples; they are not a clean performance reference.
6. A prior synthetic run lost IDs at startup and rollover. The user accepts best-effort behavior, so r7 reports loss against the matched copy arm and does not claim zero loss.

This architecture authorizes one bounded candidate experiment and a narrow fallback fix. It does not authorize a capture-path, rotation, storage, or Manager redesign.
