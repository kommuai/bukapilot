#!/usr/bin/env bash
set -euo pipefail

EVIDENCE_ROOT=/data/openpilot/skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence
HARNESS="$EVIDENCE_ROOT/qroad-tail-memory-harness"
PHASE=${1:-}
case "$PHASE" in
  warmup) REPETITIONS=3 ;;
  pilot) REPETITIONS=5 ;;
  final) REPETITIONS=10 ;;
  *) echo "usage: $0 warmup|pilot|final" >&2; exit 2 ;;
esac
MODES=(copy zero zero-preimport zero-reverse qroad-only)
MSGQ_DIR=/dev/shm/msgq_ka2bench
CREATED_MSGQ_DIR=0

cleanup() {
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

preflight() {
  local mem cooling_file cooling_state temp_bytes msgq_bytes param_bytes log_bytes
  mem=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)
  (( mem >= 1572864 )) || { echo "abort_memavailable_kb=$mem" >&2; return 31; }
  [[ -r /dev/dma_heap/system && -r /dev/mpp_service ]] || { echo "abort_hardware_device_missing" >&2; return 32; }
  if fuser -s /dev/mpp_service /dev/dma_heap/system; then
    echo "abort_existing_mpp_or_dma_heap_user" >&2
    return 33
  fi
  if ps -eo comm= | rg -q '^(encoderd|camerad|sensord)$'; then
    echo "abort_live_camera_sensor_or_encoder" >&2
    return 34
  fi
  for cooling_file in /sys/class/thermal/cooling_device*/cur_state; do
    [[ -r "$cooling_file" ]] || continue
    cooling_state=$(cat "$cooling_file")
    [[ "$cooling_state" == 0 ]] || { echo "abort_thermal_cooling $cooling_file=$cooling_state" >&2; return 35; }
  done
  temp_bytes=$(df -B1 --output=used /dev/shm | tail -1 | tr -d ' ')
  msgq_bytes=0
  param_bytes=0
  [[ -d /dev/shm/msgq_ka2bench ]] && msgq_bytes=$(du -sb /dev/shm/msgq_ka2bench | awk '{print $1}')
  [[ -d /dev/shm/ka2-bench-params ]] && param_bytes=$(du -sb /dev/shm/ka2-bench-params | awk '{print $1}')
  log_bytes=0
  if compgen -G "$EVIDENCE_ROOT/qroad-tail-*.log" >/dev/null; then
    log_bytes=$(du -cb "$EVIDENCE_ROOT"/qroad-tail-*.log | tail -1 | awk '{print $1}')
  fi
  (( temp_bytes <= 268435456 && msgq_bytes <= 268435456 && param_bytes <= 268435456 && log_bytes <= 10485760 )) || {
    echo "abort_temp_ceiling shm=$temp_bytes msgq=$msgq_bytes params=$param_bytes logs=$log_bytes" >&2
    return 36
  }
}

run_one() {
  local mode=$1 repetition=$2 log mem temp_bytes rc qroad_max
  printf -v repetition_id '%02d' "$repetition"
  log="$EVIDENCE_ROOT/qroad-tail-${PHASE}-${mode}-${repetition_id}.log"
  [[ ! -e "$log" ]] || { echo "refusing_to_overwrite=$log" >&2; return 30; }
  preflight
  if [[ -e "$MSGQ_DIR" && ! -d "$MSGQ_DIR" ]]; then
    echo "abort_msgq_path_is_not_directory=$MSGQ_DIR" >&2
    return 32
  fi
  if [[ ! -d "$MSGQ_DIR" ]]; then
    mkdir -m 700 "$MSGQ_DIR"
    CREATED_MSGQ_DIR=1
  fi
  mem=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)
  temp_bytes=$(df -B1 --output=used /dev/shm | tail -1 | tr -d ' ')
  {
    printf 'phase=%s mode=%s repetition=%s started_utc=%s mem_before_kb=%s shm_used_bytes=%s\n' \
      "$PHASE" "$mode" "$repetition_id" "$(date -u +%FT%TZ)" "$mem" "$temp_bytes"
    for cooling_file in /sys/class/thermal/cooling_device*/cur_state; do
      [[ -r "$cooling_file" ]] && printf 'thermal_cooling_state %s=%s\n' "$cooling_file" "$(cat "$cooling_file")"
    done
    for freq_file in /sys/devices/system/cpu/cpufreq/policy*/scaling_cur_freq; do
      [[ -r "$freq_file" ]] && printf 'cpu_frequency %s=%s\n' "$freq_file" "$(cat "$freq_file")"
    done
  } > "$log"

  set +e
  env -u ZMQ -u LD_LIBRARY_PATH -u LD_PRELOAD \
    PARAMS_ROOT=/dev/shm/ka2-bench-params OPENPILOT_PREFIX=ka2bench \
    timeout --signal=TERM 15s "$HARNESS" "$mode" /dev/null >> "$log" 2>&1
  rc=$?
  set -e
  printf 'launch_exit_code=%s completed_utc=%s mem_after_kb=%s\n' \
    "$rc" "$(date -u +%FT%TZ)" "$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)" >> "$log"
  (( rc == 0 )) || { echo "harness_failed mode=$mode repetition=$repetition exit=$rc log=$log" >&2; return "$rc"; }
  rg -q 'qroad_accepted=100 qroad_failures=0 qroad_packets=100' "$log" || {
    echo "qroad_packet_gate_failed mode=$mode repetition=$repetition log=$log" >&2; return 37;
  }
  if [[ "$mode" == qroad-only ]]; then
    rg -q 'road_accepted=0 road_failures=0 road_packets=0' "$log" || {
      echo "qroad_only_gate_failed repetition=$repetition log=$log" >&2; return 38;
    }
  else
    rg -q 'road_accepted=100 road_failures=0 road_packets=100' "$log" || {
      echo "road_packet_gate_failed mode=$mode repetition=$repetition log=$log" >&2; return 39;
    }
  fi
  if [[ "$mode" == zero || "$mode" == zero-preimport || "$mode" == zero-reverse ]]; then
    rg -q 'road_route_configured_zero_copy=1 road_imported_fds=24 road_import_failures=0 road_fallback_logged=0' "$log" || {
      echo "zero_copy_route_gate_failed mode=$mode repetition=$repetition log=$log" >&2; return 40;
    }
  else
    rg -q 'road_route_configured_zero_copy=0 road_imported_fds=0 road_import_failures=0 road_fallback_logged=0' "$log" || {
      echo "copy_route_gate_failed mode=$mode repetition=$repetition log=$log" >&2; return 41;
    }
  fi
  if [[ "$mode" == zero-preimport ]]; then
    rg -q 'preimport_count=24' "$log" || { echo "preimport_gate_failed repetition=$repetition log=$log" >&2; return 42; }
  fi
  qroad_max=$(awk -F 'qroad_us=' '/^frame_latency/ { if (($2+0) > max) max=$2+0 } END { print max+0 }' "$log")
  (( qroad_max < 1000000 )) || { echo "qroad_stall_gate_failed max_us=$qroad_max log=$log" >&2; return 43; }
  mem=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)
  (( mem >= 1572864 )) || { echo "post_sample_mem_guard_failed_kb=$mem" >&2; return 44; }
  preflight
  printf 'complete phase=%s mode=%s repetition=%s qroad_max_us=%s log=%s\n' \
    "$PHASE" "$mode" "$repetition_id" "$qroad_max" "$log"
}

for ((repetition=1; repetition<=REPETITIONS; repetition++)); do
  start=$(( (repetition - 1) % ${#MODES[@]} ))
  for ((offset=0; offset<${#MODES[@]}; offset++)); do
    index=$(( (start + offset) % ${#MODES[@]} ))
    run_one "${MODES[$index]}" "$repetition"
  done
done
