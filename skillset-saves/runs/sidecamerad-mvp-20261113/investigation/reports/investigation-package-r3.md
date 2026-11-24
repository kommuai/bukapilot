---
run_id: sidecamerad-mvp-20261113
submission_id: sidecamerad-mvp-20261113_investigation-ac4-timing-r3
revision: "3"
---

# Investigation Package — SideCamera AC-4 Timing Jitter

**Boundary:** `investigation-review` · **Owning phase:** BUILD · **Build input revision:** 68 · **Investigation revision:** 3

## Scope and impact

Investigate the reproduced single-camera AC-4 cadence failure in the source-to-publication timing path for one 1280×720 MJPEG camera requested at 20 fps. AC-6 physical reconnect/sibling continuity and AC-8 four-camera performance/CPU soak remain outside scope. No product source was edited. The approved revision-1 and revision-2 packages are preserved byte-for-byte at `reports/investigation-package-r1.md` (SHA-256 `b4afe53167fb5ff722d3b8174f7e339f98d3cc66578d84d420e12acb8e6bc796`) and `reports/investigation-package-r2.md` (SHA-256 `b35b75239d451b80d9cf7015028d4598a821f0e110debd3927213fab67e44cfd`). Revision 1's source-EOF PTS proposal remains superseded: the FFmpeg PTS are V4L2 `ts-src-soe`, not EOF.

AC-4 requires at least 6,000 complete published frames over five steady-state minutes, mean cadence of at least 20 fps, no EOF-to-EOF timestamp interval above 60 ms, and matching cereal/VisionIPC frame IDs.

## Reproduction

Reuse the accepted current-source five-minute capture; it was not rerun in revisions 2 or 3. It records 6,050 matched frames over 302.506666 seconds, 6,049 EOF intervals, mean interval 49.998692 ms (20.000523 fps), maximum 89.163715 ms, and 753 intervals above 60 ms. The strict max-gap condition fails. IDs were contiguous and cereal/VisionIPC timestamps matched.

`evidence/reproduction.log` is byte-identical to the build runtime capture, SHA-256 `de804d35326c6925862d2c8b36708d4a14a10b05120076924f523397976e8c55`. The accepted build probe/report carries source lineage. `system/sidecamerad.py` remained SHA-256 `494fb8b88fda8bcf1d8301225019f4f01f4b77a0bb5cb3e69dde47ed96e1309b` through the saved captures and diagnostics. The reproduction record and hashes are bound in `manifest.json`.

## Observed evidence chain

The timestamped evidence trace is `evidence/evidence-chain-r3.log` (SHA-256 `0210a1d7b7f1643e93408616bc240692c1fa216eae31ea2ecc79d9d7dbc04376`). It preserves the established DQBUF, PTS, output, and schedstat findings and adds the one-variable output-flush A/B.

- In the corrected same-run DQBUF/PTS/output/read trace, post-startup DQBUF gaps averaged 49.994824 ms (max 58.634 ms, none >60 ms) and SOE PTS gaps averaged 50.006737 ms (max 52.128 ms, none >60 ms). Output write-boundary gaps averaged 49.957065 ms (max 89.580 ms, 146/598 >60 ms), while full-read gaps averaged 49.957056 ms (max 89.587 ms, 147/598 >60 ms). Write-start to full-read completion was median 0.116 ms, p95 0.196 ms. There were 602 successful DQBUF calls for 600 output frames and the ioctl omitted sequence fields; exact dequeue-to-output identity is not proven.
- In the corrected DQBUF trace, 25,800 output writes had median syscall duration 0.058 ms, p95 0.153 ms, p99 0.256 ms, max 10.554 ms, none >50 ms. This does not support sustained pipe backpressure. The separate writes-only trace was observer-intrusive and unpaired with DQBUF; its cadence comparison is only corroboration.
- The per-thread schedstat probe completed 600/600 frames; 77/599 read gaps were >60 ms, max 80.384 ms. Long-gap intervals averaged 1.469 ms summed runqueue delay versus 1.273 ms on other gaps; 3/77 long gaps had any thread delay >=5 ms. This does not support CPU runqueue delay as the primary cause. Post-read `/proc` sampling adds cost to the next interval and does not identify the exact writer thread.
- The new output-flush A/B used the same camera and reader with one FFmpeg argv difference, output-side `-flush_packets 1`. Both sequential, straced arms completed 600/600 frames and 25,800 writes; no payload was saved and source SHA stayed unchanged. Default: 122/599 read gaps >60 ms, maximum 88.425 ms; write-boundary starts 120/599 >60 ms, maximum 88.367 ms. Flush: 107/599 read gaps >60 ms, maximum 86.679 ms; write-boundary starts 106/599 >60 ms, maximum 86.696 ms. The apparent change is 15 fewer read gaps and a 1.746 ms lower read maximum; write-boundary counts fell by 14 and the maximum by 1.671 ms. Both arms still contain substantial >60 ms gaps.
- The A/B arms were run sequentially and were unpaired; `strace` can perturb scheduling. Order effects and run-to-run variation could explain some or all of the small difference. Write syscall duration maxima were 9.632 ms default and 11.402 ms flush, with no duration >60 ms. This pair is diagnostic only and does not prove a causal effect or pass AC-4.
- FFmpeg PTS are monotonic-domain V4L2 SOE timestamps (`ts-src-soe`), not EOF. Keep `timestampEof` as host-monotonic time sampled after complete raw NV12 frame read.

A/B report: `../build/reports/report_ffmpeg-flush-pacing-ab-r2.3-r1.md` SHA-256 `f6e9f0a2a172d9401e28627d180823bc806bd1f6c1e1627789a489f513902c06`. Supporting JSON and trace hashes are bound in the manifest.

## Mechanism

**Medium confidence:** cadence jitter appears after V4L2 buffer dequeue and inside FFmpeg processing/output before raw-pipe delivery. The close write-boundary/read-completion cadence and sub-millisecond write-to-read lag do not support a separate Python reader backlog. Scheduler deltas do not support CPU runqueue scheduling as the primary cause, and output write syscall durations do not support sustained pipe backpressure.

The one sequential `-flush_packets 1` pair showed a modest improvement in gap counts and maxima, but both arms retain substantial >60 ms gaps. Because the pair was sequential and straced, the result is inconclusive and not causal proof. It does not establish output flushing as the cause or justify a production argv change. Exact FFmpeg internal decode/filter/output pacing or synchronization wait remains uncertain; exact output writer-thread attribution and per-frame DQBUF identity are not established.

## Bounded BUILD path

**Owner:** BUILD through `build/build-management`.

To assess whether the small flush-option shift is repeatable, run one read-only ABBA diagnostic on the same assigned camera: output-flush default (A), `-flush_packets 1` (B), `-flush_packets 1` (B), default (A), with 600 frames per arm. Keep the existing fixed executable, 1280×720 MJPEG-to-NV12 mode, one decoder thread, one filter thread, queue size 1, reader, and all other arguments fixed. Apply identical tracing to all four arms, retain only raw write-call timing/arguments and monotonic full-read completion times, and save no image payload. Make no source edit and do not change production argv.

Compare both A arms with both B arms for complete-frame counts, >60 ms read/write-boundary gaps, maxima, and write durations. If the B arms do not consistently separate from the A arms, report the output-flush effect as non-repeatable; if they do, report only a repeatable diagnostic sensitivity. In either case the ABBA result cannot pass AC-4. Preserve `timestampEof` as host-monotonic time after complete raw-frame read; PTS remain SOE. The required five-minute acceptance remains the decision point after any separately approved product change.

## Residual uncertainty

- The observed A/B difference may be a real small output-flush sensitivity, tracing overhead, arm order, or ordinary run-to-run variation; a single sequential pair cannot separate these explanations.
- The proposed ABBA diagnostic has not been run. Tracing remains a source of observer effect, and the probe does not isolate internal FFmpeg decode/filter/synchronization waits or identify the precise output writer thread.
- The DQBUF capture counted 602 successful calls for 600 outputs and omitted sequence fields, so exact buffer-to-output identity remains unproven.
- AC-4 remains failed on the accepted five-minute reproduction; the 600-frame diagnostics do not replace it. AC-6 reconnect/sibling continuity and AC-8 four-camera soak/CPU headroom remain unverified hardware deferrals. The existing minor FFmpeg child resource-limit finding remains open.
