#!/usr/bin/env bash
set -uo pipefail

test_binary="skillset-saves/runs/loggerd-ka2-efficiency-20261029/build/evidence/g0-mpp-encode-probe"
target_binary="/tmp/ka2-efficiency-r7-mergecheck/system/loggerd/encoderd"
expected_binary_sha="c30b920d13b072d271122408822192e6e427b8ecac4ed289e3cecd64b7d37b47"
expected_mpp_sha="ea762b7da71b63a73cdf14d053d84b091654aee5e5c4f26ceb1449b8fb8d92f9"
actual_binary_sha="$(sha256sum "$target_binary" | awk '{print $1}')"
actual_mpp_sha="$(sha256sum /lib/aarch64-linux-gnu/librockchip_mpp.so.1 | awk '{print $1}')"
if [ "$actual_binary_sha" != "$expected_binary_sha" ] || [ "$actual_mpp_sha" != "$expected_mpp_sha" ]; then
  printf 'fingerprint_mismatch binary=%s mpp=%s\n' "$actual_binary_sha" "$actual_mpp_sha"
  exit 2
fi
if fuser -s /dev/mpp_service /dev/dma_heap/system /dev/dri/renderD128 /dev/dri/renderD129; then
  printf 'abort_competing_mpp_dma_or_drm_device_user=true\n'
  exit 3
fi
competing_processes="$(ps -eo pid=,comm=,args= | awk -v self="$$" -v parent="$PPID" '
  $1 != self && $1 != parent {
    comm = tolower($2)
    if (comm == "bash" || comm == "sh" || comm == "ps" || comm == "awk" ||
        comm == "timeout" || comm == "env") next
    line = tolower($0)
    if ((line ~ /camerad|encoderd|sensord|loggerd|mpp/) &&
        line !~ /system\.loggerd\.deleter/ &&
        line !~ /system\.loggerd\.uploader/) print
  }')"
if [ -n "$competing_processes" ]; then
  printf 'abort_competing_processes=true\n%s\n' "$competing_processes"
  exit 3
fi
mem_available_kb="$(awk '/MemAvailable/ {print $2}' /proc/meminfo)"
data_free_kb="$(df -Pk /data | awk 'NR==2 {print $4}')"
if [ "$mem_available_kb" -lt 1536000 ] || [ "$data_free_kb" -lt 1048576 ]; then
  printf 'abort_resource_headroom mem_available_kb=%s data_free_kb=%s\n' "$mem_available_kb" "$data_free_kb"
  exit 4
fi
printf 'preflight_binary_sha=%s mpp_sha=%s mem_available_kb=%s data_free_kb=%s\n' \
  "$actual_binary_sha" "$actual_mpp_sha" "$mem_available_kb" "$data_free_kb"
stream_path="$(mktemp /dev/shm/ka2-r7-mpp-encode.XXXXXXXXXX)" || exit 1
count_path="${stream_path}.frames"

date -Is
printf 'temporary_stream=%s\n' "$stream_path"
timeout --signal=TERM --kill-after=2s 10s env -u LD_LIBRARY_PATH -u LD_PRELOAD "$test_binary" "$stream_path"
probe_rc=$?
printf 'mpp_probe_timeout_wrapper_exit=%d\n' "$probe_rc"

if [ -f "$stream_path" ]; then
  stream_size="$(stat -c '%s' "$stream_path")"
  printf 'stream_bytes=%s\n' "$stream_size"
  sha256sum "$stream_path"
  if [ "$probe_rc" -eq 0 ] && [ "$stream_size" -gt 0 ] && [ "$stream_size" -le 8388608 ]; then
    ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames \
      -of default=noprint_wrappers=1:nokey=1 -f hevc "$stream_path" > "$count_path"
    ffprobe_rc=$?
    frames="$(cat "$count_path" 2>/dev/null)"
    printf 'ffprobe_exit=%d decoded_frame_count=%s\n' "$ffprobe_rc" "$frames"
    timeout --signal=TERM --kill-after=1s 5s ffmpeg -nostdin -v error -xerror \
      -f hevc -i "$stream_path" -frames:v 1 -f null -
    ffmpeg_rc=$?
    printf 'ffmpeg_decode_exit=%d\n' "$ffmpeg_rc"
  else
    printf 'decode_skipped=true\n'
  fi
else
  printf 'temporary_stream_missing=true\n'
fi

if [ -e "$stream_path" ]; then rm -- "$stream_path"; fi
if [ -e "$count_path" ]; then rm -- "$count_path"; fi
printf 'temporary_stream_removed=true\n'

printf 'post_processes:\n'
ps -eo pid,comm,args | rg -i 'camera|encoderd|loggerd|sensord|mpp|rga' | rg -v 'rg -i|irq/'
printf 'post_device_users:\n'
fuser -v /dev/mpp_service /dev/dma_heap/system /dev/dri/renderD128 /dev/dri/renderD129 2>&1 || true
printf 'post_resources:\n'
awk '/MemAvailable|CmaFree|CmaTotal/ {print}' /proc/meminfo
df -Pk /data | tail -1
df -Pk /dev/shm | tail -1
