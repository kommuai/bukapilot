# Intake decision log: side camera MVP

## Resolved decisions

| ID | Decision | Chosen | Source |
| --- | --- | --- | --- |
| D-1 | How side-camera capture relates to native camera capture | Add a separate `sidecamerad` process and leave native `camerad` enabled. | user |
| D-2 | Camera topics and target rate | Publish `frontLeftCameraState`, `frontRightCameraState`, `backLeftCameraState`, and `backRightCameraState` with matching VisionIPC streams at 20 fps. | user |
| D-3 | MVP resolution | Use the previously exercised 1280x720 MJPEG capture mode. | prior-artifact |
| D-4 | Dependency and change boundary | Keep all source changes within `/data/openpilot`; install no packages. Use an already available FFmpeg executable to decode MJPEG directly to NV12. | user |
| D-5 | Device configuration and rollout | Make the process opt-in with `SIDECAMERAD=1`; configure each camera by an explicit V4L2 path and permit a partial set for smoke tests. | delegated-default |
| D-6 | Resource budget | Use bounded VisionIPC buffers and one reusable frame buffer per capture worker; avoid BGR conversion and application-level frame queues. | delegated-default |

## Deferred decisions

| ID | Decision | Why deferral is safe | Owner | Reopen phase |
| --- | --- | --- | --- | --- |
| D-7 | Include side-camera frames in drive logs or encoder output | The MVP can validate live capture through VisionIPC and the state topics without changing loggerd or encoderd. | side-camera feature owner | design before logging integration |
| D-8 | Feed side-camera imagery to the driving model or expose it in the UI | The first milestone only publishes frames and does not change current model or UI consumers. | side-camera feature owner | design before model or UI integration |
| D-9 | Cross-camera synchronization and camera-specific transforms | Independent streams with host timestamps are enough to validate capture rate; downstream use may set stricter synchronization and orientation needs. | side-camera feature owner | design before downstream consumer integration |

## Rejected options

| ID | Option | Rationale |
| --- | --- | --- |
| D-10 | Replace native `camerad` or route through the webcam override | The user requested native capture remain active; these paths would alter its startup behavior. |
| D-11 | Require all four devices before starting capture | A partial configuration lets the current single-device setup exercise the MVP. |
| D-12 | Add a Python camera or image-processing package | The user prohibited package installation; a direct FFmpeg-to-NV12 path avoids Python image conversion dependencies. |

## Non-goals and YAGNI deferrals

| ID | Non-goal | Reopen trigger | Owner |
| --- | --- | --- | --- |
| D-13 | Change native `camerad`, model subscriptions, UI, loggerd camera encoding, or driving behavior | A side-camera consumer needs one of those integrations. | side-camera feature owner |
| D-14 | Auto-discover identical USB cameras or persist camera-to-corner assignment | A deployed installation needs configuration beyond explicit V4L2 paths. | side-camera feature owner |
