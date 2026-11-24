# Checkpoint — evidence chain revision 2

- Evidence-chain stage complete; run revision 87 was the input.
- Preserved the approved revision-1 package byte-for-byte at `investigation/reports/investigation-package-r1.md`; SHA-256 remains `b4afe53167fb5ff722d3b8174f7e339f98d3cc66578d84d420e12acb8e6bc796`.
- New observed chain: `investigation/evidence/evidence-chain-r2.log`; scheduler artifact and log registered at revision 87.
- Findings: after-startup DQBUF and SOE PTS cadence is regular; raw output write/read cadence has >60 ms gaps. Exact DQBUF/output frame identity remains unproven (602 successful dequeues vs 600 outputs, no sequence fields).
- Scheduler deltas do not support CPU runqueue delay as the primary cause; write durations do not support sustained pipe backpressure; reader backlog is not supported. The exact FFmpeg internal processing/output wait remains uncertain.
- Timestamp contract retained: EOF timestamp stays host monotonic after full raw-frame read; FFmpeg PTS is SOE, not EOF.
- Next action: state the bounded BUILD-owned diagnostic path and residual uncertainty, then assemble manifest revision 2 and run its mechanical self-check.
