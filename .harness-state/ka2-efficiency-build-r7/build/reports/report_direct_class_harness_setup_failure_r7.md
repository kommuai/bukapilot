# Direct MppEncoder class-harness setup failure — r7

## Outcome

The first direct-class harness attempt produced no benchmark data. The run owner observed the process exit with SIGBUS after about 36 ms. The preserved log contains only the mode/configuration banner; it contains no accepted-frame counts, packet data, decoder result, or latency sample. The owner reports that the failure occurred before MPP initialization. There is no core file, and the read-only kernel log inspection found no user-process fault entry; the precise crash PC is therefore not independently established.

Do not count this as a warm-up or sample. It does not change the successful direct MPP import/encode readiness results, and it does not establish a performance gain or regression.

## Source finding and correction

The failed harness constructed `roadEncodeData` and `qRoadEncodeData` subscribers with a 256-byte MsgQ segment. The production service definitions require 10,485,760 bytes for each topic. MsgQ's `msgq_new_queue()` calls `ftruncate()` on the shared queue file for every publisher or subscriber open. The subscriber could therefore shrink the same backing file used by the publisher and make a later write through its larger mapping SIGBUS.

The failed source SHA-256 was `2299430943ccd80774c55e0bb7b7a16d94ab1e4ce3576c08a49afd2201607484`; it was linked into harness binary SHA-256 `7dab5fa63920d50e65fcc6c6a32c39357e1dbaeff4cc9302eae4cd353676075d`. The run owner changed the collector to use `services.at(topic).queue_size`. This is the bounded harness correction; it does not change the production candidate binary or MppEncoder implementation.

The queue-size mismatch is a concrete crash mechanism consistent with the observed SIGBUS. The short raw log does not locate the fault, so this report preserves it as the likely source finding rather than claiming a captured stack trace.

## Evidence

- Failed attempt log: `build/evidence/class-warmup-copy-1.log`, SHA-256 `5a3a4a501fac6c8224dd2b15e4024b0abcb72100ad1a8b0a1ca5dc330054ea75`.
- Failed harness source: `build/evidence/mpp_encoder_direct_class_harness.cc`, SHA-256 `2299430943ccd80774c55e0bb7b7a16d94ab1e4ce3576c08a49afd2201607484`.
- Failed harness binary: `build/evidence/mpp_encoder_direct_class_harness`, SHA-256 `7dab5fa63920d50e65fcc6c6a32c39357e1dbaeff4cc9302eae4cd353676075d`.
- Board process/device check after failure found no harness/encoderd/camera/sensor process and no open `/dev/mpp_service` or `/dev/dma_heap/system` user. No core file was present; `coredumpctl` is unavailable on the image.

## Next boundary

The run owner will rebuild the corrected harness, verify its MPP runtime path and hashes, then repeat fresh read-only process, device, memory, and disk checks before any planned discarded warm-up. If the corrected harness fails before a measurable sample, stop and report the exact evidence. No performance claim is supported until the planned five-pair exploratory pilot and, if valid, the final paired set complete under the saved ceiling.
