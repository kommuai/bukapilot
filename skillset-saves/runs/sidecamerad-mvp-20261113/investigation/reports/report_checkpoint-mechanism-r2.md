# Checkpoint — mechanism revision 2

## Observed boundary
The accepted five-minute reproduction still fails the max-gap threshold on unchanged source. In the corrected 600-frame same-run trace, after-startup DQBUF and V4L2-source PTS intervals remain regular while FFmpeg raw-output write boundaries and Python full-read completions contain 146/598 and 147/598 gaps above 60 ms. The DQBUF trace has 602 successful calls for 600 outputs and omits sequence fields, so it localizes the cadence divergence but does not prove exact dequeue-to-output frame identity.

## Surviving inference
Medium confidence: jitter occurs after V4L2 buffer dequeue, inside FFmpeg processing/output and before raw-pipe delivery. The near-identical write-boundary/read-completion cadence and 0.116 ms median write-start-to-read lag do not support a separate Python reader backlog. Long-gap schedstat deltas are only slightly above other gaps; 3/77 long gaps had a thread delay >=5 ms, so CPU runqueue delay is not supported as the primary cause. Corrected trace write durations max at 10.554 ms with none >50 ms, so sustained pipe backpressure is not supported.

## Hypotheses displaced or unresolved
- Reader-side backlog: not supported by the same-run boundary/read correlation and sub-millisecond completion lag.
- CPU runqueue delay as primary cause: not supported by post-read per-thread schedstat deltas.
- Sustained pipe backpressure: not supported by write syscall durations; shorter transient interactions are not ruled out.
- Exact FFmpeg decode/filter/output pacing or internal synchronization wait: unresolved. The capture does not isolate this internal wait, and exact raw-output writer-thread attribution is unproven.

Timestamp semantics remain unchanged: host monotonic time after complete raw-frame read is `timestampEof`; PTS is V4L2 SOE, not EOF.
