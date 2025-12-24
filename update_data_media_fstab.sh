#!/bin/sh
set -e

sed -i 's|^/dev/mmcblk1p1[[:space:]]\+/data/media[[:space:]]\+auto[[:space:]]\+discard,nosuid,nodev,nofail,x-systemd.device-timeout=5s[[:space:]]\+0[[:space:]]\+0$|/dev/mmcblk1p1 /data/media ext4 rw,nosuid,nodev,nofail,noatime,errors=remount-ro,commit=10,data=ordered,x-systemd.device-timeout=5s 0 2|' /etc/fstab
