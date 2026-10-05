# Live Tune Plotter (on-demand)

Lightweight on-device HTTP server + browser UI to live-plot:

- **Cards**:
  - `carState.vEgo` (km/h)
  - `carState.vCruise` (km/h)
  - Lead distance / relative speed / probability from `radarState.leadOne`
  - `longitudinalPlan.dangerOverrideActive` (also plotted)
- **Longitudinal PID plot**: `controlsState.upAccelCmd/uiAccelCmd/ufAccelCmd`
- **Lateral PID plot (auto-detected)**:
  - `controlsState.lateralControlState.pidState.p/i/f`, or
  - `controlsState.lateralControlState.torqueState.p/i/f`
- **Stock contribution plot**: `carState.stockAccelCmd`

Plot y-axis ranges:

- Longitudinal PID: fixed `[-3.5, 3.5]`
- Lateral PID: fixed `[-2.0, 2.0]`
- Stock contribution: dynamic over visible window

## Run (on device)

From the openpilot repo root:

```bash
python3 tools/tune_plotter/server.py
```

Server always binds to `0.0.0.0:8080`.

Open in a browser:

- `http://<device_ip>:8080/`

## Optional flags

- `--sample-hz <Hz>`: backend sampling rate (default: 20)

