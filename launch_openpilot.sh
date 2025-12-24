#!/usr/bin/bash

sudo ./patch_support_tunnel.sh
sudo ./update_data_media_fstab.sh
exec ./launch_chffrplus.sh
