# Security seed — sidecamerad MVP

**Run:** `sidecamerad-mvp-20261113`  
**Stage:** `security-seed` (`design` → `design-to-build`)  
**Architecture basis:** `report_architecture.md`, revision 11, SHA-256 `da35a916fdf28e9ab1a5a5849f598c9783232026fb9d4e8e465d47033ee68ca5`  
**Interface basis:** `report_interfaces.md`, revision 11, SHA-256 `7b6299040d93ab96a875639f24656988ef3ac97f0643987c146faf4855a62755`

## Assessment

The design introduces a path from locally attached UVC/V4L2 devices into an external decoder process, then publishes decoded camera frames and metadata to local clients. The device and MJPEG stream are untrusted inputs. Camera imagery can contain sensitive personal or operational context; the designed data path is in-memory local IPC, with no new network service, persistent storage, credentials, or secret handling.

The build must implement the controls below. This is a forward-looking design seed: it does not grade implementation, scan dependencies, or claim that any control is already present.

| Boundary | Trust crossing and risk | Control the build owes |
|---|---|---|
| `device-config` | Operator environment variables select V4L2 paths used by workers. Malformed values could select unintended inputs or affect process arguments. | Validate each non-empty value against the supported V4L2 device-path shape before use; reject malformed paths without affecting sibling workers. Treat each path as one data argument, never as command text. |
| `peripheral-to-decoder` | A locally attached camera supplies attacker-controlled MJPEG bytes to FFmpeg's parser; malformed input can crash or consume decoder resources. | Keep decoding at the fixed 1280×720, 20 fps NV12 contract and one FFmpeg thread per camera. Keep the four-worker cap, fixed reusable/scratch and three-buffer allocations, and no app frame queue. Publish only complete frames of the exact expected size (1,382,400 bytes); discard partial/invalid output. On EOF, short read, decoder error, or child exit, close and reap the child, publish no frame during failure, and apply the specified per-camera capped backoff. Bound child resources and inherited capabilities/descriptors to the minimum needed to open its selected device and return frame bytes; do not run it with elevated privileges. |
| `sidecar-to-ffmpeg-process` | The selected device path crosses from sidecar configuration into a child-process invocation. Shell interpretation or configurable command fragments could turn a device selection into arbitrary command execution. | Invoke the existing FFmpeg executable directly with a fixed argument vector and no shell. Pass only the validated device path as a single argument; do not interpolate environment values into a command string or permit them to select executable, codec, filter, output target, or extra flags. Close unrelated inherited file descriptors and keep the child at the sidecar's unprivileged identity. |
| `sidecar-to-local-ipc` | Decoded imagery and frame metadata become readable by local VisionIPC and cereal clients. Images may carry sensitive scene content; a broader transport would expose them beyond the intended host-local consumer set. | Use only the `sidecamerad` local VisionIPC/control endpoint and the specified sidecar cereal services. Preserve existing local IPC binding and access permissions; do not bind a routable network interface or add a remote listener. Keep stream IDs 7–10 and the fixed position mapping isolated from native and encoder streams; publish only configured-camera frames and the defined frame metadata. Verify no persistent or external sink is introduced. |

## Boundaries explicitly unchanged

The architecture leaves native `camerad` unchanged and adds no external API, network service, package, credential, or secret flow. Those surfaces do not gain a new boundary in this design. Keep the sidecar opt-in (`SIDECAMERAD=1`) and preserve failure isolation so a sidecar or camera failure does not change native camera behavior.

## Residual risk for the build handoff

The design's four-camera throughput and CPU headroom remain unmeasured. One decoder thread per worker, bounded application buffers, and retry backoff constrain load, but do not by themselves establish a hard bound on FFmpeg's internal memory or worst-case decode time. The build must apply an enforceable child resource limit where supported, or report the remaining decoder-resource exposure explicitly for owner review. This seed does not establish decoder sandboxing or OS-level device permissions beyond the controls stated above.

## Typed `security_seed` assessment

```yaml
applicable: true
scope: "New sidecamerad capture path: operator-provided V4L2 device selection, untrusted MJPEG decoding through FFmpeg, child-process launch, and local VisionIPC/cereal publication."
decided_by: "security-builder"
architecture_revision: "report_architecture.md revision 11, SHA-256 da35a916fdf28e9ab1a5a5849f598c9783232026fb9d4e8e465d47033ee68ca5; report_interfaces.md revision 11, SHA-256 7b6299040d93ab96a875639f24656988ef3ac97f0643987c146faf4855a62755"
reason: "The design moves untrusted local peripheral bytes across a decoder process boundary and introduces host-local IPC publication of potentially sensitive camera imagery; each requires explicit build controls."
boundaries:
  - id: "device-config"
    control: "Validate each configured V4L2 path shape, reject malformed values per worker, and pass it only as one data argument."
  - id: "peripheral-to-decoder"
    control: "Treat MJPEG as untrusted; preserve fixed format, one thread per worker, bounded buffers and worker count, exact-size complete-frame acceptance, per-camera close/reap/backoff, and least-privilege child resource limits."
  - id: "sidecar-to-ffmpeg-process"
    control: "Use the fixed existing FFmpeg executable through direct argv without a shell or configuration-controlled flags; pass only a validated device path, close unrelated inherited descriptors, and retain the unprivileged identity."
  - id: "sidecar-to-local-ipc"
    control: "Publish only through host-local sidecar VisionIPC/cereal interfaces with existing local permissions, no routable listener, fixed isolated stream mapping, and no persistent/external sink."
```

The `security_seed` key has no sanctioned fallback at `design-to-build`; the applicable assessment and boundary controls above are required evidence.
