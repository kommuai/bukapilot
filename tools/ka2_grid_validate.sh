#!/usr/bin/env bash
# KA2 wide/road capture + RG/BG/Y metrics vs baseline.
set -euo pipefail

OPDIR="${OPDIR:-/data/openpilot}"
OUTDIR="${OUTDIR:-/tmp/ka2_grid_validate}"
BASELINE="${BASELINE:-$OPDIR/docs/ka2_baseline_metrics.json}"
CAPTURE="${CAPTURE:-/tmp/ka2_vipc_capture}"
SETTLE_S="${SETTLE_S:-35}"
BASELINE_MODE="${BASELINE_MODE:-0}"
export OPDIR OUTDIR BASELINE CAPTURE SETTLE_S BASELINE_MODE

mkdir -p "$OUTDIR"

if ! pgrep -x camerad >/dev/null; then
  echo "error: camerad not running" >&2
  exit 1
fi

if [[ ! -x "$CAPTURE" ]]; then
  if [[ ! -f /tmp/ka2_vipc_capture.cc ]]; then
    CAPTURE_SRC="$OPDIR/tools/ka2_vipc_capture.cc"
  else
    CAPTURE_SRC="/tmp/ka2_vipc_capture.cc"
  fi
  if [[ ! -f "$CAPTURE_SRC" ]]; then
    echo "error: missing ka2_vipc_capture.cc" >&2
    exit 1
  fi
  echo "building $CAPTURE ..."
  clang++ -std=c++17 -O2 -I"$OPDIR" -I"$OPDIR/msgq" -I"$OPDIR/msgq_repo" \
    -o "$CAPTURE" "$CAPTURE_SRC" \
    -L"$OPDIR/msgq_repo" -L"$OPDIR/common" -L"$OPDIR/third_party" \
    -lvisionipc -lmsgq -lcommon -ljson11 -lzmq -pthread -lOpenCL
fi

echo "waiting ${SETTLE_S}s for AE settle ..."
sleep "$SETTLE_S"

"$CAPTURE" 0 "$OUTDIR/grid_wide.nv12" 10000
"$CAPTURE" 1 "$OUTDIR/grid_road.nv12" 10000

python3 << 'PY'
import json, os, struct
from datetime import datetime, timezone
from pathlib import Path

outdir = Path(os.environ["OUTDIR"])
baseline_path = Path(os.environ["BASELINE"])
baseline_mode = os.environ.get("BASELINE_MODE", "0") == "1"
commit = os.popen(f'cd {os.environ["OPDIR"]} && git rev-parse --short HEAD 2>/dev/null').read().strip() or "unknown"

def nv12_to_rgb(nv12, meta):
    w, h, s = map(int, meta.read_text().split())
    data = nv12.read_bytes()
    y_plane = data[: s * h]
    uv_plane = data[s * h : s * h + s * h // 2]
    rgb = bytearray(w * h * 3)
    for row in range(h):
        for col in range(w):
            y = y_plane[row * s + col]
            uv_i = (row // 2) * s + (col // 2) * 2
            u = uv_plane[uv_i] - 128
            v = uv_plane[uv_i + 1] - 128
            r = max(0, min(255, int(y + 1.402 * v)))
            g = max(0, min(255, int(y - 0.344 * u - 0.714 * v)))
            b = max(0, min(255, int(y + 1.772 * u)))
            o = (row * w + col) * 3
            rgb[o], rgb[o + 1], rgb[o + 2] = r, g, b
    return rgb, w, h

def stats(rgb, w, h):
    rs = gs = bs = 0.0
    n = 0
    ys = ytops = 0.0
    for row in range(h):
        yrow = row < 200
        for col in range(w):
            o = (row * w + col) * 3
            r, g, b = rgb[o], rgb[o + 1], rgb[o + 2]
            y = 0.299 * r + 0.587 * g + 0.114 * b
            ys += y
            if yrow:
                ytops += y
            if 20 < g < 240:
                rs += r / g
                bs += b / g
                n += 1
    rg = rs / n if n else 1.0
    bg = bs / n if n else 1.0
    return {"rg": round(rg, 4), "bg": round(bg, 4), "y": round(ys / (w * h), 2),
            "top_y": round(ytops / (w * min(200, h)), 2)}

try:
    from PIL import Image
except ImportError:
    Image = None

metrics = {"commit": commit, "timestamp": datetime.now(timezone.utc).isoformat(), "cameras": {}}
for cam, nv, meta in [("wide", "grid_wide.nv12", "grid_wide.nv12.meta"),
                      ("road", "grid_road.nv12", "grid_road.nv12.meta")]:
    rgb, w, h = nv12_to_rgb(outdir / nv, outdir / meta)
    metrics["cameras"][cam] = stats(rgb, w, h)
    if Image is not None:
        Image.frombytes("RGB", (w, h), bytes(rgb)).save(outdir / f"ka2_{cam}.jpg", quality=92)
    s = metrics["cameras"][cam]
    print(f"KA2 {cam}: RG={s['rg']:.3f} BG={s['bg']:.3f} Y={s['y']:.1f} top={s['top_y']:.1f}")

metrics_path = outdir / "metrics.json"
metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
print(f"metrics -> {metrics_path}")

if baseline_mode:
    baseline_path.parent.mkdir(parents=True, exist_ok=True)
    baseline_path.write_text(metrics_path.read_text())
    print(f"baseline saved -> {baseline_path}")
    raise SystemExit(0)

if not baseline_path.exists():
    print(f"warn: no baseline at {baseline_path}")
    raise SystemExit(0)

baseline = json.loads(baseline_path.read_text())
ok = True
tols = {"rg": 0.05, "bg": 0.05, "y": 5.0, "top_y": 5.0}
for cam in ("wide", "road"):
    for key, tol in tols.items():
        b = baseline["cameras"][cam][key]
        c = metrics["cameras"][cam][key]
        pass_ = abs(c - b) <= tol
        if not pass_:
            ok = False
        print(f"  {cam} {key}: {c} vs {b} ({'OK' if pass_ else 'FAIL'})")
print("VALIDATION:", "PASS" if ok else "FAIL")
raise SystemExit(0 if ok else 1)
PY
