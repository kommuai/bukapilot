# Scope change — sidecamerad availability

**Tier:** 2 — the request changes the approved process default and adds a persisted runtime interface.  
**Run:** `sidecamerad-mvp-20261113`, revision 38, active/pinned.  
**Prior approved design:** `r1`, `design-to-build` cross-stage verdict SHA-256 `36b70ad825fe8f036460fb8d7eb994b40195d3d1661e548f09610d4d81275832`.

## User-confirmed requirements

- Enable `sidecamerad` by default while leaving native `camerad` intact.
- Acknowledge missing or disconnected side cameras.
- Add a Params key reporting availability for all four camera positions.
- Keep the implementation small and validate both device execution and frame publication.
- Do not install packages; source edits stay in `/data/openpilot`.

## Existing evidence

- Host has one connected UVC device: `/dev/video60`, USB ID `534d:2109`; `/dev/video61` is that device's metadata node.
- V4L2 reports `/dev/video60` as MJPEG 1280×720 at 20 fps; no process owned either node during the probe.
- Existing `sidecamerad` was run against `/dev/video60`. It published a valid `frontLeftCameraState` with frame ID 1 and VisionIPC frames with IDs 1 and 2; ID 1 matched, buffer size was 1,382,400 bytes, and the receiver observed nonzero image data. It shut down with exit 0 and removed its socket.
- Only one decoder is attached on this host, so four-device behavior cannot be physically verified here.

## Design decisions to resolve before the next build

- The accepted r1 contract was opt-in with explicit per-position device paths and no auto-discovery. The new default-on requirement supersedes the opt-in policy.
- One proposed Params key is `SideCameraAvailability`, a JSON object with four stable topic-name boolean fields, cleared/reset at manager startup. Availability should become true after the first frame and false on disconnect; a missing feed should be logged once and remain false.
- `/dev/video0` through `/dev/video59` on this host are native Rockchip camera/ISP nodes, so they must not be selected as side cameras. USB discovery must select actual USB capture nodes and ignore metadata node `video61`.
- Identical USB decoder nodes expose no unique serial/side identity. The user has been asked whether physical hub-port order should map to front-left, front-right, back-left, back-right; automatic side labels remain pending that answer.
- `cereal/services.py` currently groups the four services beside the native camera services. Although `SERVICE_LIST` is built from insertion order, its `idx` is unused and the generated C++ registry is keyed by service name; no numeric service-port shift was found. Appending the new entries is optional ordering cleanup, not an ABI fix.
- A first `python -m system.sidecamerad` probe returned immediately because the file has no module entry point. The successful probe invoked `main()` as the manager does. A `__main__` guard is a small candidate change to make the manual validation command match normal module launch.

The next owning boundary is design revision 2, followed by build revision 2. No source has yet been changed for this scope update.
