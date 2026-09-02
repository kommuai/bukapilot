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

## Spectra-style calibration (no /etc/iqfiles)

Production CamHw registers the compiled ISP30 calibration blobs from
`system/camerad/rk/calib_embed/*.bin` directly with librkaiq. Startup fails if
the selected blob is missing or invalid; it does not silently fall back to JSON.
The matching JSON files remain in `calib_embed/` as human-readable reference and
rollback fixtures. Vendor `/etc/iqfiles/ox03c10*` must stay absent for the
no-cheat check.

## Reference

Behavioral reference: **commaai/openpilot master** camera
(`openpilot/system/camerad/` — Spectra + ox03c10). See Admiral run spec
`reference-openpilot-master-camera.md` on the build host.

## Tone (gamma / linearization)

Comma Spectra sequence (ox03c10):

1. **Linearization** — PWL12 decompand (Spectra `linearization_lut` inverse)
2. **Gamma** — ox03c10 analytic curve on linear data

On RK3588 CamHw, `adegamma` uapi does not affect pixels (A/B verified). We apply the
**mathematically equivalent** composed LUT `kOx03c10GammaCommaV11` = gamma(linearize(x))
via `agamma_v11`, plus calib/uapi degamma for future HW. Tables in `rk_tone_tables.h`.
