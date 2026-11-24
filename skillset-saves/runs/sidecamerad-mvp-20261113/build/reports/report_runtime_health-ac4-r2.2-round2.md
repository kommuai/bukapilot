---
submission_id: sidecamerad-mvp-20261113_build-r2.3-ac4-handoff-c68
revision: "68"
run_id: sidecamerad-mvp-20261113
source_submission_id: sidecamerad-mvp-20261113_build-r2.2-c68
---

# Runtime health — AC-4 cadence recheck, SideCamera r2.2

**Boundary:** `build-to-review` · **Package revision:** 68 · **Target source:** `6d92d63a2` · **Approved design:** r2  
**Probe window:** 302.506666 seconds from first matched frame · **Observed:** 2026-11-15T08:20:31.035145+00:00

## Result

**Runtime functional check: pass. AC-4 strict cadence: fail.** The assigned front-left device produced 6,050 selected matching cereal/VisionIPC frame IDs over an EOF timestamp span of 302.442086351 seconds. The interval mean was 20.000523316652 fps, meeting the ≥20.0 fps mean. The maximum EOF gap was 89.163715 ms, above the 60 ms limit, and 753 of 6,049 intervals exceeded 60 ms. AC-4 therefore remains unproven and fails the strict gap criterion.

The selected frame-ID sequence was contiguous, with zero frame-ID gaps and zero cereal/VisionIPC SOF or EOF timestamp mismatches. The evidence retains every per-frame ID, SOF/EOF timestamp, receive timestamp, and derived EOF interval in `artifacts/runtime_cadence_intervals-r2.2-round2.json`.

## Timing evidence and failure localization

The long EOF intervals are present in the sidecar timestamps and in both readers' arrival times. Across the selected 6,050 matching frames, the VisionIPC subscriber receive interval mean was 49.998402 ms; its maximum was 89.389179 ms, with 747 of 6,049 receive intervals above 60 ms. Median VisionIPC receive lag behind the published EOF timestamp was 0.881836 ms (maximum 7.017679 ms). This shows the reader did not accumulate a queue backlog or diverge from the publisher. Instead, complete decoded frames reached the sidecar publication boundary with irregular intervals: 753 EOF gaps exceeded 60 ms, while 754 intervals were under 40 ms; their mean remained near 50 ms.

The source timestamps are assigned by `time.monotonic_ns()` when the complete FFmpeg pipe frame is read. This evidence localizes jitter to frame delivery before or at that publication boundary; it does not distinguish V4L2 input timing, FFmpeg decode/output timing, and host scheduling, because no camera capture timestamp is exposed in this setup. No product source changed during this probe. Build-management should route a scoped timing investigation/fix before claiming AC-4.

## Runtime and cleanup

- The assigned path `/dev/v4l/by-path/platform-xhci-hcd.11.auto-usb-0:1:1.0-video-index0` resolved to `/dev/video60`; only one USB decoder was attached.
- VisionIPC reported 1280×720, stride 1280, three buffers, and 1,382,400 bytes per frame. The inspected 4,096-byte sample was nonzero.
- The local controller had `ZMQ` unset; the child began with `ZMQ=1`, and `sidecamerad` removed it before constructing IPC publishers. Ports 9007–9010 had no listeners at startup, before shutdown, or afterward.
- Params started all-false; front-left was true immediately before shutdown; all feeds were false after SIGTERM. The `sidecamerad` process exited 0, its FFmpeg child was gone, and the VisionIPC stream was unavailable after exit.
- The isolated temporary `PARAMS_ROOT` was removed. No recording file was created, `/data/params` was untouched, and no packages were installed.
- All nine runtime input hashes matched the manifest before and after the capture; `source_hash_drift` is empty.

## Controller attempts

The first attempt (`evidence/runtime-smoke-ac4-r2.2-round1.log`, typed probe `artifacts/runtime_probe-ac4-r2.2-round1.json`) is retained as **harness-only diagnostic evidence and is invalid for cadence acceptance**. Its controller compared `Poller.poll()`-returned `SubSocket` wrappers by Python identity and did not drain cereal readiness events; it stopped at the 20-second no-match timeout. VisionIPC frames were observed, sidecamerad exited 0, and cleanup succeeded. Round 2 uses the corrected readiness dispatch and contains the timing series.

In round 2 the controller sampled Params immediately after the first matching pair, before the availability update completed, so its first overall functional label was false. The recorded pre-shutdown snapshot showed front-left true and the final post-shutdown snapshot showed all-false; this report and typed probe correct the functional status from those observed transitions. The raw timing series is unchanged by that correction.

## Evidence and remaining limits

- Smoke log: `evidence/runtime-smoke-ac4-r2.2-round2.log`
- Typed probe: `artifacts/runtime_probe-ac4-r2.2-round2.json`
- Full per-frame timestamps and intervals: `artifacts/runtime_cadence_intervals-r2.2-round2.json`
- Invalid first controller attempt is preserved separately as noted above.
- AC-6 physical reconnect/child reap/backoff/sibling continuity remains unverified. AC-8 four-camera 30-minute/144,000-frame soak and CPU headroom remain unverified because only one decoder is available.
