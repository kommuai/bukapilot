# S0 Feasibility Report — loggerd/encoderd Encoder Efficiency

**Run:** `loggerd-ka2-efficiency-20261029`  
**Design:** r5; design phase and Admiral cross-stage verdicts are APPROVED  
**Slice:** `slice.loggerd.feasibility` / AC-13  
**Disposition:** **STOP — AC-13 is not met. Do not delegate S1.**  
**Formal handoff:** ESCALATE to Admiral for the missing exact-target evidence. This is not a build-to-review submission or gate verdict.

## Outcome

I completed the source-backed feasibility review and took a fresh idle/resource preflight on KA2 at `2026-10-30T07:49:13.645511Z`. The preflight passed. No direct-MPP capability probe started: the old loaded-MPP API and matching headers/package are unresolved, and a probe built against guessed or newer headers could exercise the wrong ABI. The remaining independent packet-size and DMA/headroom unknowns would still block AC-13 after such a probe.

No source, test, or unrelated file was changed. I did not run the loggerd/encoderd integration regression, STOP_DRAIN, or a performance sample. Existing candidate edits remain in the worktree and are not approved by this report. No performance improvement has been measured.

## AC-13 Feasibility Matrix

| Requirement | S0 finding | Status |
| --- | --- | --- |
| Exact SConstruct/link configuration, headers, built/launched binaries, and actual MPP/RGA mappings | SConstruct specifies KA2 MPP/RGA links, includes `/usr/local/include/rockchip`, uses `/usr/local/lib` before `/usr/lib/aarch64-linux-gnu`, and has `RPATH=[]`. Current files are AArch64. Their hashes differ from the binaries recorded for the prior regression, and neither the exact build/header hashes nor actual launch-process maps are present. | **UNKNOWN — STOP** |
| Loaded 2024 MPP API/version and matching source/package | Existing r5 identity notes name `/usr/lib/aarch64-linux-gnu/librockchip_mpp.so.0` and a file hash, but this is not paired with the exact process mapping or matching source/package/header evidence. Newer `/usr/local` headers/library are explicitly not runtime proof. | **UNKNOWN — STOP** |
| Supported EOS/completion, observable success, nonempty output per accepted ID, zero pending, initial and segment IDR | No probe ran and no matching old-API source/package establishes these behaviors. Candidate EOS code is unapproved and is not evidence. | **UNKNOWN — STOP** |
| `conflate=false` ring, cadence, reuse, lifetime, maximum pause | Static source sets four RK camera/VisionIPC slots and nominal 20 FPS, so the nominal slot wrap is 200 ms. VisionIPC has no reader lease/ack; its client receives a pointer into shared ring memory while the producer advances modulo slots. The ring-wrap limit is not a guaranteed scheduling or MPP-copy bound. | **Partly source-proven; retention/pause UNKNOWN — STOP** |
| Current queue/writer semantics | Source confirms 10 MiB IPC segments for EncodeData; a 200-message per-service loggerd future queue drops oldest; the MPP pending metadata cap is eight; current writer starts on a keyframe and writes each payload and EncodeIdx. | **Source-proven** |
| Exact-target encoded-message maximum and raw/future queue caps | AC-13 formulas are recorded below. Configured CBR/QP values do not provide a hard maximum encoded packet size, and historical samples are not a worst-case bound. | **UNKNOWN — STOP** |
| RSS plus supported DMA/device-memory headroom under 80% | Fresh whole-system memory use is 25.93% while daemon processes are idle. No active encoder RSS, exact DMA/CMA/device-memory use, or computed queue total is available; the packet bound and supported pause are also unknown. | **UNKNOWN — STOP** |
| Static Manager signal/wait order and timeout | Static source confirms loggerd then encoderd are signaled nonblocking; waits run encoderd first and loggerd afterward. Each non-camerad stop has a 5-second timeout. | **Source-proven; no runtime shutdown proof** |

## Sizing Evidence and Limits

Camera input is 1920×1200 NV12, or 3,456,000 bytes per aligned full-resolution frame. For `F_s=20/s`, the AC-13 staging formula is:

```text
C_raw,s >= ceil(F_s * T_pause / 1000) + 1
B_raw,s = C_raw,s * aligned_NV12_frame_bytes
```

The static ring wraps in 200 ms. If a future implementation could prove a 199 ms maximum pause and copy completion, the formula gives five frames / 17,280,000 bytes per full-resolution camera stream. This is only a conditional example: neither that pause nor the copy lifetime has been proved.

There are four output services at 20 messages/s: road, qRoad, wideRoad, and driver. For a future fence horizon `H_ms<1000`:

```text
C_fifo,s >= ceil(R_s * H_ms / 1000) + 1
B_fifo,s >= C_fifo,s * P_s
```

At 999 ms, the count would be 21 messages per output service, 84 total. `P_s` must include encoded payload, Cap’n Proto metadata/padding, and allocation accounting. No defensible exact-target upper bound for `P_s` is available, so the future FIFO byte cap and combined memory budget cannot be evaluated. A CBR target/max bitrate and QP limits constrain the encoder configuration but do not prove a per-frame packet maximum. The current 10 MiB IPC queue setting also does not prove a maximum single MPP message.

The preflight’s current memory reading cannot stand in for proposed-cap feasibility: `/proc/meminfo` reported 3,730,980 kB total and 2,763,604 kB available (25.93% used), but no daemon was running. `/data` had 2,004,910,080 bytes free; `/tmp` had 838,819,840 bytes free. The highest listed thermal-zone temperature was 47.153°C. The planned probe ceilings were 600 seconds, 805,306,368 output bytes (768 MiB), 80% memory use, and 80°C. The probe consumed no time/output because it was not started.

## Manager Source Record

`system/manager/process_config.py` puts loggerd before encoderd. `ensure_running` first sends all stop signals without blocking in process-list order, then follows `VISION_SHUTDOWN_ORDER`, where encoderd is first; loggerd is waited afterward. `ManagerProcess.stop` applies five seconds to each non-camerad process. This satisfies only the requested static S0 record. The existing sequence has not been shown to finish inside a shared 4500 ms window; that remains an S1 AC-16 obligation.

## Why the Direct-MPP Probe Was Not Started

The probe is allowed only on the exact loaded image with a safe, version-matched API. Current evidence has a build/runtime skew: the link configuration searches newer local headers/libraries, while the prior target identity points to the older `/usr/lib` MPP object. No matching package/source/header set or actual launched-process maps are saved. Testing against the newer API by guess would not prove the old image and risks an invalid probe. Independently, packet-size and memory feasibility remain unknown, so the allowed probe cannot make AC-13 pass by itself. Per the handoff and Admiral instruction, I stopped after preflight.

## Evidence and Next Owner Action

- Fresh preflight: `build/evidence/loggerd-s0-preflight.txt` (raw SHA-256 `86c733c3b38dc2fe4e5bbdde87c30c4515315d21bae2eb100319624af60ffcb3`).
- Source/static review and raw source hashes: `build/evidence/loggerd-s0-static-review.txt`.
- Approved contract: design `reports/delivery-plan.md` AC-13 and `reports/implementation-spec.md` `slice.loggerd.feasibility`.

Admiral should retain the S0 stop and obtain exact 2024 MPP package/source/header evidence plus an exact launch-time identity record, then request a fresh decision on a narrowly scoped direct-MPP synthetic probe. The target/runtime owner must also provide a supported packet upper bound, a measured/defensible slot-retention pause, and RSS plus supported DMA/device-memory accounting for the proposed caps. If any remain unknown or exceed 80%, S1 remains stopped. S1 implementation, daemon regression, STOP_DRAIN, the S2 copy reference, candidate profiling, and efficiency sampling remain unauthorized.
