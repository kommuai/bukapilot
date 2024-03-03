#!/bin/bash

cp ai.comma.plus.neossetup.apk /sdcard/
mount -o remount,rw /system && (mkdir -p /system/priv-app/plusneossetup && cp /sdcard/ai.comma.plus.neossetup.apk /system/priv-app/plusneossetup/neossetup.apk && cp /sdcard/ai.comma.plus.neossetup.apk /system/priv-app/NEOSSetup/NEOSSetup.apk && touch /system/priv-app/plusneossetup/apk_v1 && chmod 755 /system/priv-app//plusneossetup && chmod 644 /system/priv-app/plusneossetup/neossetup.apk); mount -o remount,ro /system
pm install -r -d /sdcard/ai.comma.plus.neossetup.apk
