# Admiral checkpoint — AC-4 same-frame diagnostic r1

**Active boundary:** BUILD_ACTIVE; investigation-review r8 is APPROVED. The user authorized continued metadata-only AC-4 diagnostic execution on 2026-11-15. No product source or production FFmpeg argv changed.

## Finding

The captured steady-state >60 ms reader interval is not present in the camera’s V4L2 timestamp or dequeue spacing for the same contiguous frame pair. It first exceeds 60 ms at FFmpeg’s filter-sink output, after decoder output, and persists through mux and complete reader EOF. This localizes the observed event to the FFmpeg filter/output path or host scheduling around it. It does not yet establish which one caused the delay.

| Boundary, frames 25 → 26 | Interval |
| --- | ---: |
| V4L2 source timestamps (monotonic SOE) | 52.023 ms |
| DQBUF completion | 50.732 ms |
| Decoder output completion | 55.893 ms |
| Filter-sink output completion | 62.067 ms |
| Mux packet completion | 63.028 ms |
| Complete raw-frame reader EOF | 63.049 ms |

The 695.988 ms V4L2 timestamp jump from sequences 0→1 was a separate startup event; it is not the post-warmup 24→25 source interval associated with the triggered output gap. The previous uninstrumented five-minute run still establishes that AC-4 cadence remains failing (716/6,049 intervals over 60 ms; maximum 90.982 ms).

## Limits and cleanup

The metadata observer did not capture stdout syscalls or a scheduler trace, and its overhead was not independently calibrated. Treat this as localization evidence, not an AC-4 acceptance run or proof of a root cause. FFmpeg required forced `-9` cleanup after the trigger; the process was reaped and subsequent checks found no remaining FFmpeg/sidecamerad process or `/dev/video60` holder.

## Evidence and hashes

- Detailed report: `skillset-saves/runs/sidecamerad-mvp-20261113/build/reports/report_ac4-same-frame-diagnostic-r1.md` — SHA-256 `edc55036493c4737be0fc0a32cc29c8f3e84438ace0fdaf9e6242541a40d1a4b`
- Normalized result: `skillset-saves/runs/sidecamerad-mvp-20261113/build/artifacts/ac4-same-frame-diagnostic-r1.json` — SHA-256 `c3a0a47e2ca05acc7e4d6d10b602fe2963879269b64d58d672c0c841afe8edb0`
- Per-frame metadata: `skillset-saves/runs/sidecamerad-mvp-20261113/build/evidence/ac4-same-frame-diagnostic-r1.jsonl` — SHA-256 `3dc54ebda52502b3bf432979b251820ee79f8370c79dd0068b6955fe5622d4a8`
- Observer ring: `skillset-saves/runs/sidecamerad-mvp-20261113/build/evidence/ac4-same-frame-observer-r1.bin` — SHA-256 `a05aec12d482b39c7732361639bf87f492525f09696849fd8dc40e5e43517079`
- Observer source/library: `build/artifacts/ac4-dqbuf-write-observer-r1.c` — SHA-256 `26b07153b036b2e134b255afeb285ad51a0577610ff4e142d241d0522f920050`; `build/artifacts/ac4-dqbuf-write-observer-r1.so` — SHA-256 `88b7f51fc9107e878f4680c87dfaaba99c3225fc7afdac15d277ec7df32cf19a`

## Next action

Continue with one bounded metadata-only capture that records wall and thread CPU time inside FFmpeg’s scale conversion call, correlated by frame PTS, using the same camera, production binary and argv, 20-frame warmup, 600-frame cap, and first post-warmup >60 ms stop rule. If the required scale symbol cannot be safely observed without changing the production command or materially perturbing cadence, record that limitation and use a lower-overhead available trace. Keep AC-4 open and do not apply a source fix until the mechanism is established.
