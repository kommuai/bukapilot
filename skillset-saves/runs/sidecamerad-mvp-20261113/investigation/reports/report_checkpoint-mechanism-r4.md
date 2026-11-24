# Checkpoint — mechanism reassessment revision 4

The ABBA repeatability check does not reproduce the earlier single-pair `-flush_packets 1` improvement. Pooled read gaps >60 ms are 228/1,198 default versus 227/1,198 flush, but flush has the higher maximum (96.677 ms versus 90.374 ms). Pooled write-boundary gaps are 228/1,198 in both groups, with a higher flush maximum (96.578 ms versus 90.280 ms). B1 looked better than A1; B2 was worse than A2. This is not a repeatable cadence effect and does not support a production argv change.

The surviving localization remains after V4L2 dequeue and before raw-pipe delivery inside FFmpeg processing/output. Reader-side backlog, CPU runqueue delay as the primary cause, and sustained pipe backpressure remain unsupported. The exact wait or compute phase inside FFmpeg is still unknown. PTS remains V4L2 SOE, not EOF; keep `timestampEof` host-monotonic after complete raw-frame read.
