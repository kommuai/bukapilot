# Security seed — sidecamerad default-on availability MVP

**Run:** `sidecamerad-mvp-20261113` · **Revision:** r2  
**Stage:** `security-seed` (`design` → `design-to-build`)  
**Architecture:** `report_architecture-r2.md`, SHA-256 `9f4cb41838303bd661048d62ddbcfea0ced765d4fc10ec228ef2c453e1a94991`  
**Interfaces:** `report_interfaces-r2.md`, SHA-256 `2ae1cfbe0746ae3a03df5a682e2dbe02d11337270809f3ace4dd220a6b0c05ad`

## Assessment

The sidecar now starts by default under the existing `driverview` condition, so a configured USB camera and its untrusted MJPEG input reach FFmpeg without the former opt-in flag. This increases the capture and decoder exposure when explicit device paths are configured. The sidecar remains a separate, bounded process; native `camerad` keeps its existing policy. The design adds no package, remote listener, credential, external service, or persistent image sink.

Camera imagery can contain sensitive personal or operational context. Pixels remain in memory and the existing host-local VisionIPC/cereal paths. The single `SideCameraAvailability` Param contains only four boolean status fields; it contains no image data or device paths. This is a forward-looking design seed: it defines controls the build owes and does not claim that implementation or security checks have passed.

## Trust boundaries and required controls

| Boundary | Trust crossing and risk | Control the build owes |
|---|---|---|
| `default-on-lifecycle` | With the manager flag unset, a configured sidecar can begin capturing whenever the existing `driverview` condition is true. This increases access to locally attached camera imagery and decoder work. | Keep the sidecar separate from native `camerad`; retain the explicit `SIDECAMERAD=0` opt-out if used. Start only workers for explicitly configured feeds. Cap the process at four workers and one FFmpeg thread per worker. When no paths are configured, create no workers or buffers, warn once per feed, and remain in the signal-wait loop without capture activity. |
| `device-config` | Operator environment variables select V4L2 image devices and cross into FFmpeg arguments. A malformed or redirected path could select an unintended node or become command input. | Keep one explicit variable per named side: `SIDECAM_FRONT_LEFT`, `SIDECAM_FRONT_RIGHT`, `SIDECAM_BACK_LEFT`, and `SIDECAM_BACK_RIGHT`. Recommend `/dev/v4l/by-path/...-video-index0` for stable physical-port identity; accept legacy `/dev/videoN`. Validate the allowed path shape, resolve beneath `/dev`, and verify the resolved target is a character-device V4L2 image node on open/retry. Reject metadata-only `video-index1`; never infer a side from USB enumeration order. Keep a missing by-path symlink in that side's retry loop so a reappearing same-port link is revalidated before use. |
| `peripheral-to-decoder` | A locally attached device supplies attacker-controlled MJPEG bytes to FFmpeg. Malformed data can crash the child or consume CPU and memory. | Preserve the fixed 1280×720 MJPEG-to-NV12 mode, at most four workers, one decoder thread per worker, three VisionIPC buffers plus one reusable scratch frame per configured camera, and no BGR conversion or application frame queue. Accept only a complete 1,382,400-byte output frame. On child exit, EOF, short read, startup/stall timeout, or device loss, stop publishing for that feed, mark its availability false, close/reap the child, and retry with the specified capped backoff. Apply target-supported child resource limits where safe; if unavailable, report the remaining decoder resource exposure for owner/security review. |
| `sidecar-to-ffmpeg-process` | A selected path reaches a spawned decoder, where shell interpretation or configurable command fragments could turn device selection into command execution. | Invoke only the existing fixed FFmpeg executable using a fixed argv and no shell. Pass one validated device path as one argument; do not let environment values select the executable, codec, filter, output, or flags. Close unrelated inherited descriptors and retain the unprivileged identity. Send bounded lifecycle diagnostics to the existing log path; do not capture unbounded stderr or log camera bytes. |
| `sidecar-to-local-status` | Concurrent camera workers update shared availability state. Malformed or unsynchronized writes could misreport camera presence; status may remain stale after an uncatchable process death. | Register exactly one `SideCameraAvailability` key as `JSON` with `CLEAR_ON_MANAGER_START` and the compact all-false JSON literal as its C++ declared default: `{"frontLeftCameraState":false,"frontRightCameraState":false,"backLeftCameraState":false,"backRightCameraState":false}`. The Python manager and sidecar pass Python dictionaries directly to `Params.put()`, which serializes them as JSON; never pass a JSON text string. Keep all four stable fields present and boolean-only. At sidecar startup, write the complete all-false dictionary before workers start. Use a lock around shared-map mutation and serialization, then one synchronous `Params.put(dict)` per state transition; never write per frame. Change a field to true only after the same full frame ID has completed both VisionIPC and cereal sends; change it to false on capture loss/failure/stop and write all false on orderly shutdown. Treat the key as a readiness summary, not an acknowledgement to each subscriber. Manager-start clearing/default initialization restores all-false state after restart; a hard kill may leave the last value until the manager next initializes it. |
| `sidecar-to-local-ipc` | Decoded images and frame metadata become readable by local VisionIPC and cereal consumers. Image content is sensitive; routing to a broader transport could expose it outside the host. | Publish only over the existing host-local `sidecamerad` VisionIPC endpoint and the four position-specific cereal services. Preserve local endpoint binding and permissions, stream isolation, and fixed position mapping. Add no routable listener, persistent sink, or remote consumer. Keep frame metadata and pixels on their existing separate channels; do not duplicate pixels in cereal. |

## Unchanged or bounded surfaces

- Native `camerad` remains under its original manager policy and behavior.
- No external API, network service, package, credential, or secret flow is added.
- The four-camera 80 fps load, CPU headroom, disconnect injection, and by-path replug recovery remain unproven because current hardware evidence covers one decoder only. One decoder thread and bounded application buffers do not provide a hard bound on FFmpeg's internal memory or worst-case decode time.
- No source edits, dependency scan, runtime test, or security verification is claimed by this design-stage seed.

## Typed `security_seed` assessment

```yaml
applicable: true
scope: "Default-on sidecamerad capture of explicitly configured local V4L2 image nodes, untrusted MJPEG decoding through FFmpeg, transition-only Params availability state, and host-local VisionIPC/cereal publication."
decided_by: "security-builder"
architecture_revision: "report_architecture-r2.md SHA-256 9f4cb41838303bd661048d62ddbcfea0ced765d4fc10ec228ef2c453e1a94991; report_interfaces-r2.md SHA-256 2ae1cfbe0746ae3a03df5a682e2dbe02d11337270809f3ace4dd220a6b0c05ad"
reason: "Default-on operation increases capture exposure for configured peripherals, and the design crosses a local device/decoder boundary while publishing potentially sensitive imagery to local IPC. Explicit path validation, bounded least-privilege decoding, controlled status writes, and host-local publication are required."
boundaries:
  - id: "default-on-lifecycle"
    control: "Keep the sidecar separate, cap workers/decoder threads, run only configured feeds, preserve the signal-wait idle branch, and leave native camerad unchanged."
  - id: "device-config"
    control: "Validate explicit per-side device paths, resolve targets under /dev as image-node character devices, reject metadata-only nodes, pass a path only as data, and never infer sides from USB order."
  - id: "peripheral-to-decoder"
    control: "Treat MJPEG as untrusted; use fixed format and worker/thread/buffer limits, accept exact complete frames only, close/reap/retry failed children, and report any unavailable hard resource limit."
  - id: "sidecar-to-ffmpeg-process"
    control: "Use fixed direct argv without a shell, pass only the validated path, close unrelated inherited descriptors, retain the unprivileged identity, and bound diagnostics."
  - id: "sidecar-to-local-status"
    control: "Write only the complete four-boolean JSON value under one lock on state transitions, initialize all false, set true only after matching dual publication, clear on loss/orderly shutdown, and reset at manager start."
  - id: "sidecar-to-local-ipc"
    control: "Keep imagery on existing host-local VisionIPC/cereal interfaces with local permissions; add no network listener, persistent sink, or remote consumer."
```

The `security_seed` key has no sanctioned fallback at `design-to-build`; the applicable assessment and controls above are required evidence.
