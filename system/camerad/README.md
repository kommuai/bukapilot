# RK3588 camerad

The RK3588 camera path is self-contained in `camera/`, `sensor/`, and `isp/`:

1. `rkaiq_3A.service` is stopped so there is no competing ISP process.
2. camerad embeds librkaiq CamHw (`sysctl_init/prepare/start`) per camera.
3. AE is manual: camerad drives exposure and gain from the NV12 luma mean.
4. The pixel path is RK ISP mainpath NV12 into VisionIPC.

## Calibration

The OX03C10 calibration and tuning data are compiled into the RK ISP path. Runtime
startup must fail if the selected calibration is missing or invalid; there is no
silent fallback to an unrelated platform configuration.

## Tone and linearization

The OX03C10 pipeline applies PWL12 linearization followed by the sensor gamma curve.
On RK3588 CamHw, the composed equivalent LUT is applied through `agamma_v11`, with
future-compatible degamma configuration retained in the ISP calibration path.
The tables are in `isp/tone_tables.h`.
