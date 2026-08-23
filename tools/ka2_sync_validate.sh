#!/usr/bin/env bash
# Restart camerad repeatedly and measure wide/road timestamp alignment.
set -euo pipefail
OPDIR="${OPDIR:-/data/openpilot}"
CYCLES="${CYCLES:-5}"
SAMPLES="${SAMPLES:-30}"
SETTLE_S="${SETTLE_S:-15}"
MEASURE="$OPDIR/tools/ka2_sync_measure"
SCONS="$OPDIR/opendbc_repo/.venv/bin/scons"
CAMERAD="$OPDIR/system/camerad/camerad"

if [[ ! -x "$MEASURE" ]]; then
  clang++ -std=c++17 -O2 -I"$OPDIR" -I"$OPDIR/msgq" -I"$OPDIR/msgq_repo" \
    -o "$MEASURE" "$OPDIR/tools/ka2_sync_measure.cc" \
    -L"$OPDIR/msgq_repo" -L"$OPDIR/common" -L"$OPDIR/third_party" \
    -lvisionipc -lmsgq -lcommon -ljson11 -lzmq -pthread -lOpenCL
fi

pass=0
fail=0
for cycle in $(seq 1 "$CYCLES"); do
  echo "=== cycle $cycle/$CYCLES ==="
  pkill -9 -x camerad 2>/dev/null || true
  sleep 2
  LOG="/tmp/camerad_sync_${cycle}.log"
  (cd "$OPDIR/system/camerad" && nohup ./camerad >"$LOG" 2>&1 &)
  sleep "$SETTLE_S"
  if ! pgrep -x camerad >/dev/null; then
    echo "FAIL: camerad not running"
    tail -20 "$LOG" || true
    ((fail++)) || true
    continue
  fi
  grep -E "STREAM_START|sysctl_start" "$LOG" | tail -12 || true
  set +e
  out=$("$MEASURE" "$SAMPLES" 2>&1)
  rc=$?
  set -e
  echo "$out"
  if [[ $rc -eq 0 ]]; then
    echo "cycle $cycle: PASS"
    ((pass++)) || true
  else
    echo "cycle $cycle: FAIL (rc=$rc)"
    ((fail++)) || true
  fi
  pkill -9 -x camerad 2>/dev/null || true
  sleep 2
done

echo "=== summary: pass=$pass fail=$fail cycles=$CYCLES ==="
[[ $fail -eq 0 ]]
