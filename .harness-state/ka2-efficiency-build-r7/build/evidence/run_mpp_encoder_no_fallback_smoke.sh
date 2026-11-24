#!/usr/bin/env bash
set -euo pipefail

EVIDENCE_DIR=/data/openpilot/skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence
HARNESS="$EVIDENCE_DIR/mpp_encoder_no_fallback_smoke"
PREFIX=ka2_mpp_nofallback_r8
MSGQ_DIR="/dev/shm/msgq_$PREFIX"
PARAMS_DIR="/dev/shm/params_$PREFIX"
STREAM="/dev/shm/$PREFIX.hevc"
LOG="$EVIDENCE_DIR/mpp_encoder_no_fallback_smoke.log"
HASH="$EVIDENCE_DIR/mpp_encoder_no_fallback_smoke.framemd5"
CREATED_MSGQ_DIR=0
CREATED_PARAMS_DIR=0

cleanup() {
  rm -f -- "$STREAM"
  if (( CREATED_MSGQ_DIR )); then
    if fuser -s "$MSGQ_DIR/roadEncodeData" "$MSGQ_DIR/qRoadEncodeData"; then
      echo "cleanup_skipped_msgq_still_in_use=$MSGQ_DIR" >&2
    else
      rm -f -- "$MSGQ_DIR/roadEncodeData" "$MSGQ_DIR/qRoadEncodeData"
      rmdir -- "$MSGQ_DIR" 2>/dev/null || true
    fi
  fi
  if (( CREATED_PARAMS_DIR )); then
    rm -rf -- "$PARAMS_DIR"
  fi
}
trap cleanup EXIT

[[ ! -e "$MSGQ_DIR" && ! -e "$PARAMS_DIR" && ! -e "$STREAM" && ! -e "$LOG" && ! -e "$HASH" ]] || {
  echo "refusing_to_overwrite_existing_smoke_artifact_or_runtime_path" >&2
  exit 30
}
mem=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)
(( mem >= 1572864 )) || { echo "abort_memavailable_kb=$mem" >&2; exit 31; }
[[ -r /dev/dma_heap/system && -r /dev/mpp_service ]] || { echo abort_hardware_device_missing >&2; exit 32; }
if fuser -s /dev/mpp_service /dev/dma_heap/system; then
  echo abort_existing_mpp_or_dma_heap_user >&2
  exit 33
fi
if ps -eo comm= | rg -q '^(encoderd|camerad|sensord)$'; then
  echo abort_live_camera_sensor_or_encoder >&2
  exit 34
fi
for cooling_file in /sys/class/thermal/cooling_device*/cur_state; do
  [[ -r "$cooling_file" ]] || continue
  [[ "$(cat "$cooling_file")" == 0 ]] || { echo "abort_thermal_cooling $cooling_file" >&2; exit 35; }
done

mkdir -m 700 "$MSGQ_DIR"
CREATED_MSGQ_DIR=1
mkdir -m 700 "$PARAMS_DIR"
CREATED_PARAMS_DIR=1
printf 'started_utc=%s memavailable_kb=%s\n' "$(date -u +%FT%TZ)" "$mem" > "$LOG"
set +e
env -u ZMQ -u LD_LIBRARY_PATH -u LD_PRELOAD \
  PARAMS_ROOT="$PARAMS_DIR" OPENPILOT_PREFIX="$PREFIX" \
  timeout --signal=TERM 15s "$HARNESS" "$STREAM" >> "$LOG" 2>&1
rc=$?
set -e
printf 'exit=%s completed_utc=%s memavailable_kb=%s\n' \
  "$rc" "$(date -u +%FT%TZ)" "$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)" >> "$LOG"
(( rc == 0 )) || { cat "$LOG"; exit "$rc"; }
rg -q 'road_accepted=100 road_failures=0 road_packets=100' "$LOG" || { cat "$LOG"; exit 37; }
rg -q 'qroad_accepted=100 qroad_failures=0 qroad_packets=100' "$LOG" || { cat "$LOG"; exit 38; }
rg -q 'road_imported_fds=24 qroad_imported_fds=0' "$LOG" || { cat "$LOG"; exit 39; }
ffmpeg -v error -i "$STREAM" -an -f framemd5 "$HASH"
sha256sum "$HASH" >> "$LOG"
printf 'decoded_stream_valid=1\n' >> "$LOG"
