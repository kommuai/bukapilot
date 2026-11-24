#!/usr/bin/env bash
set -euo pipefail

EVIDENCE_DIR=/data/openpilot/skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence
HARNESS="$EVIDENCE_DIR/mpp_encoder_zero_default_dma_guard_smoke"
MSGQ_DIR=/dev/shm/msgq_ka2bench
PARAMS_DIR=/dev/shm/ka2-bench-params
DEFAULT_STREAM=/dev/shm/ka2-zero-default.hevc
COPY_STREAM=/dev/shm/ka2-copy-override.hevc
CREATED_MSGQ_DIR=0

cleanup() {
  rm -f -- "$DEFAULT_STREAM" "$COPY_STREAM"
  if (( CREATED_MSGQ_DIR )); then
    if fuser -s "$MSGQ_DIR/roadEncodeData" "$MSGQ_DIR/qRoadEncodeData"; then
      echo "cleanup_skipped_msgq_still_in_use=$MSGQ_DIR" >&2
    else
      rm -f -- "$MSGQ_DIR/roadEncodeData" "$MSGQ_DIR/qRoadEncodeData"
      rmdir -- "$MSGQ_DIR" 2>/dev/null || true
    fi
  fi
}
trap cleanup EXIT

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
[[ ! -e "$MSGQ_DIR" || -d "$MSGQ_DIR" ]] || { echo "abort_msgq_path_is_not_directory=$MSGQ_DIR" >&2; exit 36; }
if [[ ! -d "$MSGQ_DIR" ]]; then
  mkdir -m 700 "$MSGQ_DIR"
  CREATED_MSGQ_DIR=1
fi

printf 'started_utc=%s memavailable_kb=%s\n' "$(date -u +%FT%TZ)" "$mem"
for mode in default copy; do
  if [[ "$mode" == default ]]; then stream="$DEFAULT_STREAM"; else stream="$COPY_STREAM"; fi
  log="$EVIDENCE_DIR/mpp_encoder_zero_default_dma_guard_smoke-$mode.log"
  [[ ! -e "$log" ]] || { echo "refusing_to_overwrite=$log" >&2; exit 30; }
  set +e
  env -u ZMQ -u LD_LIBRARY_PATH -u LD_PRELOAD \
    PARAMS_ROOT="$PARAMS_DIR" OPENPILOT_PREFIX=ka2bench \
    timeout --signal=TERM 15s "$HARNESS" "$mode" "$stream" > "$log" 2>&1
  rc=$?
  set -e
  printf 'mode=%s exit=%s completed_utc=%s memavailable_kb=%s\n' \
    "$mode" "$rc" "$(date -u +%FT%TZ)" "$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)"
  (( rc == 0 )) || { cat "$log"; exit "$rc"; }
  rg -q 'road_accepted=100 road_failures=0 road_packets=100' "$log" || { cat "$log"; exit 37; }
  rg -q 'qroad_accepted=100 qroad_failures=0 qroad_packets=100' "$log" || { cat "$log"; exit 38; }
  if [[ "$mode" == default ]]; then
    rg -q 'road_route_configured_zero_copy=1 road_imported_fds=24 qroad_route_configured_zero_copy=0 qroad_imported_fds=0' "$log" || { cat "$log"; exit 39; }
  else
    rg -q 'road_route_configured_zero_copy=0 road_imported_fds=0 qroad_route_configured_zero_copy=0 qroad_imported_fds=0' "$log" || { cat "$log"; exit 40; }
  fi
done

ffmpeg -v error -i "$DEFAULT_STREAM" -an -f framemd5 "$EVIDENCE_DIR/mpp_encoder_zero_default_dma_guard_smoke-default.framemd5"
ffmpeg -v error -i "$COPY_STREAM" -an -f framemd5 "$EVIDENCE_DIR/mpp_encoder_zero_default_dma_guard_smoke-copy.framemd5"
cmp "$EVIDENCE_DIR/mpp_encoder_zero_default_dma_guard_smoke-default.framemd5" \
    "$EVIDENCE_DIR/mpp_encoder_zero_default_dma_guard_smoke-copy.framemd5"
sha256sum "$EVIDENCE_DIR/mpp_encoder_zero_default_dma_guard_smoke-default.framemd5" \
          "$EVIDENCE_DIR/mpp_encoder_zero_default_dma_guard_smoke-copy.framemd5"
printf 'decoded_frame_hashes_match=1\n'
