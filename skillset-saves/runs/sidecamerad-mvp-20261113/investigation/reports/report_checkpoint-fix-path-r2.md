# Checkpoint — bounded fix path revision 2

Owner: BUILD, through build/build-management.

Next action: run a read-only, one-variable 600-frame FFmpeg output-pacing A/B on the same assigned camera. Compare the current output-flush default with a second fixed argv that adds only output-side `-flush_packets 1`; keep the existing FFmpeg executable, 1280x720 MJPEG-to-NV12 mode, `-threads 1`, `-filter_threads 1`, `-thread_queue_size 1`, frame count, pipe reader, and all other arguments unchanged. This toggle adds no decoder threads or application queues. Do not save frame payloads or edit source. Preserve `timestampEof` as host-monotonic time after complete raw-frame read; retain PTS only as SOE diagnostic data.

For both 600-frame captures, record full-frame read completions, output write-boundary gaps, write syscall durations, and complete-frame counts. Compare the >60 ms gap count and maximum gap. A change is diagnostic evidence only: it neither proves the internal FFmpeg wait nor passes AC-4; only the existing five-minute acceptance can decide AC-4 after any later approved source change.

The installed FFmpeg help exposes `-flush_packets` as an output I/O flush-after-each-packet option. If BUILD/security determines that even this fixed diagnostic argv variant crosses the approved fixed-argv or resource contract, do not run it; escalate the exact configuration decision to the design/security owner before further intervention.
