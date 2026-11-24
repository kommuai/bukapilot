#!/usr/bin/env bash
set -euo pipefail

HARNESS=/data/openpilot/skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence/mpp_encoder_direct_class_harness_observed
EVIDENCE=/data/openpilot/skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence

if [[ $# -ne 2 || ! $1 =~ ^[0-9]+$ || ! $2 =~ ^[0-9]+$ ]]; then
  echo "usage: $0 first_pair last_pair" >&2
  exit 2
fi
first_pair=$1
last_pair=$2
if (( first_pair < 1 || last_pair > 10 || first_pair > last_pair )); then
  echo "pair range must be within 1..10" >&2
  exit 2
fi

run_one() {
  local pair=$1 mode=$2 log mem temp_bytes queue_bytes rc cooling_file cooling_state
  printf -v pair_id '%02d' "$pair"
  log="$EVIDENCE/class-repeat10-v1-pair-${pair_id}-${mode}.log"
  [[ ! -e "$log" ]] || { echo "refusing_to_overwrite=$log" >&2; return 30; }

  mem=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)
  (( mem >= 1572864 )) || { echo "guard_failed_mem_available_kb=$mem" >&2; return 31; }
  if [[ -n "$(fuser /dev/mpp_service /dev/dma_heap/system 2>/dev/null || true)" ]]; then
    echo "guard_failed_device_user" >&2
    return 32
  fi
  if [[ -n "$(pgrep -f '^/data/openpilot/skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence/mpp_encoder_direct_class_harness_observed' || true)" ]]; then
    echo "guard_failed_existing_harness" >&2
    return 33
  fi
  if ps -eo comm= | rg -q '^(encoderd|camerad|sensord)$'; then
    echo "guard_failed_live_camera_encoder_or_sensor_process" >&2
    return 34
  fi
  for cooling_file in /sys/class/thermal/cooling_device*/cur_state; do
    [[ -r "$cooling_file" ]] || continue
    cooling_state=$(cat "$cooling_file")
    [[ "$cooling_state" == 0 ]] || { echo "guard_failed_thermal_throttle $cooling_file=$cooling_state" >&2; return 35; }
  done
  temp_bytes=$(df -B1 --output=used /dev/shm | tail -1 | tr -d ' ')
  queue_bytes=$(du -sb /dev/shm/msgq_ka2bench | awk '{print $1}')
  (( temp_bytes <= 268435456 && queue_bytes <= 268435456 )) || {
    echo "guard_failed_temp_ceiling_bytes=$temp_bytes queue_bytes=$queue_bytes" >&2
    return 36
  }

  {
    printf 'repeat10_pair=%s mode=%s started_utc=%s mem_before_kb=%s shm_used_bytes=%s msgq_bytes=%s\n' \
      "$pair_id" "$mode" "$(date -u +%FT%TZ)" "$mem" "$temp_bytes" "$queue_bytes"
    for cooling_file in /sys/class/thermal/cooling_device*/cur_state; do
      [[ -r "$cooling_file" ]] && printf 'thermal_cooling_state %s=%s\n' "$cooling_file" "$(cat "$cooling_file")"
    done
    for freq_file in /sys/devices/system/cpu/cpufreq/policy*/scaling_cur_freq; do
      [[ -r "$freq_file" ]] && printf 'cpu_frequency %s=%s\n' "$freq_file" "$(cat "$freq_file")"
    done
  } >"$log"

  set +e
  env -u ZMQ -u LD_LIBRARY_PATH -u LD_PRELOAD \
    PARAMS_ROOT=/dev/shm/ka2-bench-params OPENPILOT_PREFIX=ka2bench \
    timeout --signal=TERM 15s "$HARNESS" "$mode" /dev/null >>"$log" 2>&1
  rc=$?
  set -e
  printf 'exit_code=%s completed_utc=%s mem_after_kb=%s\n' \
    "$rc" "$(date -u +%FT%TZ)" "$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)" >>"$log"

  [[ $rc -eq 0 ]] || { echo "harness_failed mode=$mode pair=$pair_id exit=$rc log=$log" >&2; return "$rc"; }
  local expected_zero=0 expected_imports=0
  if [[ "$mode" == zero ]]; then expected_zero=1; expected_imports=24; fi
  rg -q "road_route_configured_zero_copy=${expected_zero} road_imported_fds=${expected_imports} road_import_failures=0 road_fallback_logged=0" "$log" || {
    echo "route_status_gate_failed mode=$mode pair=$pair_id log=$log" >&2; return 37;
  }
  rg -q 'road_accepted=100 road_failures=0 road_packets=100' "$log" || {
    echo "road_output_gate_failed mode=$mode pair=$pair_id log=$log" >&2; return 38;
  }
  rg -q 'qroad_accepted=100 qroad_failures=0 qroad_packets=100' "$log" || {
    echo "qroad_output_gate_failed mode=$mode pair=$pair_id log=$log" >&2; return 39;
  }
  mem=$(awk '/MemAvailable:/ {print $2}' /proc/meminfo)
  (( mem >= 1572864 )) || { echo "post_sample_mem_guard_failed_kb=$mem" >&2; return 40; }
  grep -E 'road_route_configured|road_submit_us_median|qroad_submit_us_median|road_accepted|qroad_accepted|cpu_sample_ns|exit_code=' "$log" | tail -7
}

for ((pair=first_pair; pair<=last_pair; pair++)); do
  if (( pair % 2 == 1 )); then
    run_one "$pair" zero
    run_one "$pair" copy
  else
    run_one "$pair" copy
    run_one "$pair" zero
  fi
done
