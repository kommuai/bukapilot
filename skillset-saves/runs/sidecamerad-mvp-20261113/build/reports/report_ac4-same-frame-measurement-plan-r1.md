# AC-4 same-frame measurement plan (r1)

**Status:** Proposal for Admiral review only. **No execution is authorized.** This plan was prepared after `investigation-review` revision 8 was APPROVED. It does not authorize capture, tracing, source instrumentation, testing, source edits, or FFmpeg argv changes.

## Purpose

Find where the existing AC-4 cadence gaps arise before proposing a source fix. The accepted r2.3 run remains a cadence failure: 6,050 matched frames over 302.471744064 s, mean EOF cadence 19.998562241636 fps, maximum EOF gap 90.981562 ms, and 716/6,049 intervals over 60 ms. The mechanism remains unresolved.

The measurement should associate one camera frame across dequeue, FFmpeg decode/filter/output, and the reader's complete-frame EOF, with monotonic timestamps at each boundary. It should correlate the same interval with scheduling for the FFmpeg threads. The output is timing metadata only; no camera payload is saved.

## Proposed bounded run

- Target: the same assigned front-left UVC camera, 1280×720 MJPEG, requested 20 fps, on the same host.
- Keep the production binary and default FFmpeg argv unchanged. Do not change camera mode, reader, decoder/filter thread counts, or queue configuration.
- Maximum: 600 captured output frames. Stop after the first fully observed EOF gap greater than 60 ms, or at the 600-frame cap if no such gap occurs.
- Do not repeat or extend the existing five-minute acceptance run as part of this diagnostic. A separate five-minute AC-4 run would require its own authorization after any resulting fix.
- Do not persist images or frame bytes. Record identifiers, monotonic timestamps, counts, status flags, and bounded scheduling events only.

## Observation points and frame identity

After separate authorization, use a minimal diagnostic build/instrumentation layer that leaves production behavior and argv unchanged. Record into a preallocated in-memory metadata buffer and write the log after the bounded run, so per-frame logging does not synchronously block the capture path.

For each frame, record:

1. **V4L2 dequeue:** `VIDIOC_DQBUF` completion time from `CLOCK_MONOTONIC`, V4L2 sequence, buffer index, bytes-used, flags, and source timestamp/flags. Record ioctl entry and return times to distinguish a slow dequeue from work after dequeue.
2. **FFmpeg input and decode:** the originating dequeue sequence attached to the corresponding compressed input packet; packet acceptance time; decoder output completion time; output frame number/PTS; and drop or reorder indication. Verify the mapping rather than inferring identity from PTS or counts alone.
3. **Filter:** input and output frame identifiers and monotonic completion times around each relevant filter boundary. Preserve the original input sequence through the filter path.
4. **Raw-pipe output:** frame identifier, write entry/return times, byte count, return status, and cumulative bytes. Record retries/partial writes and whether the write blocked.
5. **Reader EOF:** existing host-monotonic timestamp after each complete raw-frame read, associated to the same frame identifier and full-frame byte count.
6. **Scheduling:** for the FFmpeg TIDs only, capture on-CPU/runqueue transitions spanning any observed >60 ms gap using a narrowly scoped scheduler trace selected after checking host support. Record TID mapping and trace loss. The exact trace method must be included in the separate authorization request; no trace has been run for this plan.

Use one monotonic clock for all timestamps where possible. If a source provides another clock, record paired samples at start and end and quantify the conversion error. Validate one-to-one sequence continuity from DQBUF through EOF; explicitly mark drops/reorders and never treat SOE PTS as EOF.

## Observer controls and abort conditions

- Review the proposed instrumentation diff and its callsites before execution; it must only collect metadata and must not alter frame contents, scheduling policy, buffering, timeouts, or production argv.
- Buffer events in memory and flush after stop. Record buffer overflow, dropped events, clock conversion uncertainty, and any observer error.
- Confirm the selected device, production executable, process ancestry, binary hash, and exact argv before starting. Abort if any differs from the approved target.
- Abort and clean up if frame identity cannot be maintained, timestamps cannot be placed on a common monotonic timeline, instrumentation loses required events, the FFmpeg child exits unexpectedly, or the camera/device changes.
- Stop at the first complete >60 ms EOF interval or at 600 frames, whichever occurs first. Reap the child, stop the scoped scheduler observer, verify no listener or diagnostic process remains, and preserve metadata logs even on abort.
- After the run, compare observed frame count and cadence with existing r2.3 evidence and report observer CPU/time cost. If observer overhead or trace loss makes the boundary attribution unreliable, mark the result inconclusive; do not compensate with an unapproved repeat.

## Interpretation branches

| Observed location of the same-frame delay | Next investigation boundary |
| --- | --- |
| Before DQBUF completion or in the driver/source timestamps | Revisit the camera/driver/source boundary; do not assume FFmpeg is responsible. |
| Between DQBUF completion and FFmpeg decoder output | Inspect input handoff/demux/decode pacing with the validated frame mapping. |
| Between decoder output and filter completion | Inspect the FFmpeg filter path and synchronization. |
| Between filter completion and pipe write completion | Determine whether output conversion or a blocked write accounts for the delay. |
| Long gap overlaps FFmpeg thread descheduling/runqueue delay | Investigate host scheduling for the mapped TID and interval. |
| No single boundary is attributable, or identity/time alignment fails | Report the mechanism as unresolved and return to Admiral; propose no source fix. |

A diagnostic result localizes the timing boundary; it does not by itself prove a causal fix or pass AC-4. Any code change must be separately scoped and reviewed, followed by a separately authorized five-minute AC-4 acceptance run.

## Required authorization package

Before execution, BUILD should return this plan to Admiral with the concrete instrumentation diff/tool choice, expected observer overhead, device/binary/argv verification method, log paths, and cleanup command. Admiral must review and record a separate authorization decision. Until then, no capture, trace, instrumentation, test, source edit, or argv change may run.
