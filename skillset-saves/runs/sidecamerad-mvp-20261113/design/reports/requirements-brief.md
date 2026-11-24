# Requirements Brief — Sidecamerad MVP

**Run:** `sidecamerad-mvp-20261113` · **Revision:** 5  
**Prepared for:** `design/architect`  
**Scope:** An opt-in side-camera capture process alongside native `camerad`, publishing four side-camera state topics and distinct VisionIPC streams.

**Sources mined:** approved intake at `skillset-saves/runs/sidecamerad-mvp-20261113/intake/reports/report_grilling.md`; `system/manager/process_config.py`; `cereal/services.py`; `cereal/log.capnp`; `system/camerad/camera/common.h` and `common.cc`; `msgq_repo/msgq/visionipc/visionbuf.h` and `visionipc_server.cc`; `msgq_repo/README.md`. The workspace `msgq` path resolves to `msgq_repo/msgq`.

## Actors and jobs

| Actor | Job to be done | Success looks like | Frequency / volume |
| --- | --- | --- | --- |
| Device operator / integrator | Opt in, assign explicit V4L2 device paths, and smoke-test available cameras | Each configured camera publishes its state and frames at the target rate; partial setups can start | Setup and diagnostics; up to four cameras |
| VisionIPC / messaging consumer | Receive the frame stream and matching state for a selected side-camera position | Four position-specific topics map to four distinct streams | Continuous; 20 frames/s per configured camera |

## Requirements

| # / requirement | Source | Confidence | Affects | Open assumption / what would raise confidence |
| --- | --- | --- | --- | --- |
| R1. Side-camera capture runs as a separate process, enabled only by `SIDECAMERAD=1`, while native `camerad` remains enabled under its existing policy. | Approved scope; intake D-1 and D-5 (`report_grilling.md`) | reported | Process-manager entry, opt-in default, and lifecycle | None; user-approved scope |
| R2. For configured positions, publish `frontLeftCameraState`, `frontRightCameraState`, `backLeftCameraState`, and `backRightCameraState`. | Approved scope; intake D-2 | reported | Messaging service names and event schema | None; user-approved names |
| R3. Each position's state topic maps to its own distinct VisionIPC stream. | Approved scope; intake D-2 | reported | Stream identifiers and publisher/subscriber namespace | None; architecture must account for the existing transport constraint in C3 |
| R4. Configure each camera with an explicit V4L2 device path. | Approved scope; intake D-5 | reported | Device-to-position configuration | None; user-approved scope |
| R5. Accept a partial set of configured camera paths so a one-device smoke test can run. | Approved scope; intake D-5 | reported | Configuration validation and process startup behavior | None; user-approved scope |
| R6. Do not install packages and use an already available FFmpeg executable to decode MJPEG directly to NV12. | User-approved scope; intake D-4 | reported | Runtime dependency, pixel path, and deployment boundary | None; user-approved scope |
| R7. Keep all source changes inside `/data/openpilot`. | User-approved scope; intake D-4 | reported | Build and repository boundary | None; user-approved scope |
| R8. Use fixed-size VisionIPC buffers and one reusable frame buffer per active capture worker; avoid BGR conversion and application-level frame queues. | Approved intake D-6 (`delegated-default`) | reported | Buffer ownership and capture data path | None; approved intake decision |

## Non-functional targets

| # / target | Source | Confidence | Affects | Open assumption / derivation |
| --- | --- | --- | --- | --- |
| N1. Capture mode is 1280×720 MJPEG input, with NV12 output. | User-approved scope; intake D-3 and D-4; prior capture summary below | reported | V4L2 mode selection, decoder output, and VisionIPC dimensions | The prior capture summary reports this mode; raw capture files were not supplied in the run artifacts |
| N2. Target 20 frames/s per configured camera (80 frames/s aggregate with four cameras). | User-approved scope; intake D-2; prior capture summary below | reported | Per-camera cadence and acceptance criteria | Aggregate four-camera throughput is unmeasured; see Open questions |
| N3. Bound camera image-payload storage to at most **27,648,000 bytes (26.37 MiB)** for four configured cameras: four VisionIPC buffers plus one reusable frame per camera, each NV12 frame sized at 1280×720×1.5 = 1,382,400 bytes. | Intake D-6; `system/camerad/camera/common.h:17` (`YUV_BUFFER_COUNT = 4`); `msgq_repo/msgq/visionipc/visionipc_server.cc:44-56` (NV12 allocation size) | inferred | Buffer count and memory acceptance target | Derived using the existing four-buffer camera precedent and the approved one-scratch-buffer-per-worker rule. This is a pixel-payload bound; it excludes FFmpeg internals, kernel/V4L2 buffers, metadata, and other process RSS. |

## Observed repository contracts

| # / contract | Source | Confidence | Affects | Open assumption / implication |
| --- | --- | --- | --- | --- |
| C1. `process_config.py` registers native `camerad` under `driverview`; its process list has no `sidecamerad` entry. `driverview` is `started or IsDriverViewEnabled`. | `system/manager/process_config.py:13-14,65-74` | observed | Additive process registration and opt-in gate; native startup is an existing contract | No requirement change; sidecamerad is not implemented yet |
| C2. Messaging service rates are declared in `cereal/services.py`; current camera-state services are 20 Hz. `FrameData` carries frame IDs, timestamps, and image data, while `Event` currently has road, driver, and wide-road camera-state arms only. | `cereal/services.py:13-19,67-74`; `cereal/log.capnp:259-280,2569-2575` | observed | Add the four service declarations and schema arms needed for the requested topics | Keep topic, schema, and rate declarations consistent |
| C3. VisionIPC has a typed stream enum and per-stream buffer creation. Under the ZMQ backend, `get_endpoint_name` asserts that server names are only `camerad` or `navd`; the current camera server uses the `camerad` name. | `msgq_repo/msgq/visionipc/visionbuf.h:15-28`; `visionipc_server.cc:19-25,44-56`; `system/camerad/camera/common.cc:504-517` | observed | Four distinct streams and compatibility with the repository's alternate ZMQ transport | Architecture must account for the existing name restriction when choosing the side-camera server namespace |
| C4. VisionIPC allocates NV12 payload as `width × height × 3 / 2`; native camera public streams use `YUV_BUFFER_COUNT = 4`. | `msgq_repo/msgq/visionipc/visionipc_server.cc:44-56`; `system/camerad/camera/common.h:17`; `system/camerad/camera/common.cc:73-85` | observed | N3's numeric frame-payload bound | Four slots are an observed precedent used for the inferred target, not a measured four-camera allocation |

## Non-goals

| # / non-goal | Source | Confidence | Affects | Reopen trigger |
| --- | --- | --- | --- | --- |
| NG1. Do not change native `camerad`, model subscriptions, UI, or driving behavior. | Approved scope; intake D-8 and D-13 | reported | Keeps the MVP to publishing and capture validation | A downstream model, UI, or driving feature requires the side-camera feed |
| NG2. Do not add side-camera logging or encoder output. | Approved scope; intake D-7 and D-13 | reported | Logger and encoder integration | A logging or encoding consumer is approved |
| NG3. Exact cross-camera synchronization and camera-specific transforms are out of scope. | Approved scope; intake D-9 | reported | Timestamp and image-transform contract | A downstream consumer needs alignment or calibrated orientation |
| NG4. Do not auto-discover identical USB cameras or persist camera-to-position assignment. | Approved scope; intake D-14 | reported | Device configuration and assignment persistence | A deployment needs unattended discovery or persistent mapping |

## Prior art

| Evidence | Source | Confidence | Affects | Limitation |
| --- | --- | --- | --- | --- |
| One MacroSilicon/AHD UVC decoder reportedly captured 1280×720 MJPEG for about five minutes / 6,000 frames at 20.000 fps, with no gaps over 60 ms and full decode success. Four compressed streams were estimated at about 73 Mbps. | Prior capture summary supplied with the approved delegation; no raw capture artifact path was supplied | reported | Supports the selected single-camera mode and rate; informs USB bandwidth planning | Single-camera evidence does not establish concurrent four-camera CPU or throughput capacity |

## Conflicts preserved

No conflicting requirements or repository evidence were found in the approved intake and sources mined.

## Open questions

| # / question | Source | Confidence | Affects | Owner / latest safe decision point |
| --- | --- | --- | --- | --- |
| Q1. What sustained frame cadence and CPU headroom do four concurrently active streams achieve on the target device? | Prior capture summary covers one stream only; no four-camera run was supplied | reported (single-camera result); four-camera result unmeasured | Performance acceptance and whether the MVP target is met | Feature owner with the target device; measure before build/review performance sign-off |

## Limitations and severity

| Finding | Source | Confidence | Affects | Severity / limitation |
| --- | --- | --- | --- | --- |
| Four-camera concurrent performance has not been demonstrated. | Prior capture summary supplied with delegation | reported for one camera; unmeasured for four | CPU capacity and sustained 80 frames/s aggregate target | **Major** evidence gap; the single-camera capture does not prove four-camera operation |
| The supplied prior-capture summary has no raw capture artifact in the run's evidence paths. | Delegation summary; intake artifacts under `skillset-saves/runs/sidecamerad-mvp-20261113/` | observed as a source limitation | Independent reproduction of the one-camera measurements | **Minor** provenance limitation; figures above remain reported evidence |
