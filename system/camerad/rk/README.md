# KA2 choice-2: userspace ISP (no rkaiq_3A)

Default KA2 path (no params toggle):

1. Stop `rkaiq_3A.service` (no competing process)
2. camerad embeds **librkaiq CamHw** (`sysctl_init/prepare/start`) per camera
3. AE is **manual** — camerad drives exposure/gain from NV12 Y-mean (comma-style)
4. Pixel path unchanged: rkisp HW → `rkisp_mainpath` NV12 → VisionIPC

CamHw runs in-process; the 3A daemon stays dead.

## Debug frame capture

With camerad streaming:

```bash
cd /data/openpilot && PYTHONPATH=/data/openpilot \
  python3 system/camerad/rk/capture_frames.py --out /tmp/cam_dbg
```

Compares NV12 vs NV21 RGB decode JPEGs and dumps flip / mediabus meta for Bayer-phase checks.

## Spectra-style calib (no /etc/iqfiles)

CamHw loads JSON from `/tmp/camerad_calib/` only. Files are copied at start from
`system/camerad/rk/calib_embed/` (camerad-owned). Vendor `/etc/iqfiles/ox03c10*`
must stay absent for the no-cheat check.

## Reference

Behavioral reference: **commaai/openpilot master** camera
(`openpilot/system/camerad/` — Spectra + ox03c10). See Admiral run spec
`reference-openpilot-master-camera.md` on the build host.

## Tone (gamma / linearization)

- Tables: `rk_tone_tables.h` from openpilot master `ox03c10.cc` + sensor PWL12 inverse.
- **Gamma:** applied via `agamma_v11` manual (verified SetAttrib OK).
- **Linearization:** `adegamma` uapi returns -255 — this librkaiq has `RKAIQ_HAVE_DEGAMMA_V1=0`.
  Full PWL-inverse as a separate ISP stage is not available; composed `g(lin(x))` crushed
  exposure-limited scenes. Degamma Y table kept for when HW/build supports it.
- DRC forced off in runtime calib scaffold.
