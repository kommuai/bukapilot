## openpilot (KA2) agent quickstart

This repo runs on a **Kommu KA2** (Rockchip RK3588). Hardware selection is **runtime-flagged** by presence of `/KA2` (see `openpilot/system/hardware/__init__.py`).

### What starts what (on this KA2 device)

- **systemd**
  - `kommu.service` → launches tmux session running `/usr/kommu/kommu.sh`
  - `kommu-usb-recovery.service` → USB gadget (RNDIS+ACM) + loader trigger via `/usr/kommu/kommu-usb-recovery.sh`
- **boot script chain**
  - `/usr/kommu/kommu.sh` → if `/data/continue.sh` exists, `exec`s it
  - `/data/continue.sh` → `cd /data/openpilot && ./launch_openpilot.sh`
  - `launch_openpilot.sh` → `launch_chffrplus.sh`
  - `launch_chffrplus.sh` → sets env, handles **KA2/AGNOS** update logic, then runs:
    - `system/manager/build.py` (if not `prebuilt`)
    - `system/manager/manager.py`

### The runtime “hub” (read these first)

- **Process supervisor**: `openpilot/system/manager/manager.py`
- **Process list + gating**: `openpilot/system/manager/process_config.py`
  - KA2-only processes include `system.hardware.ka2.setapn`, `system.hardware.ka2.status_led.indicatord`, `system.hardware.ka2.formatdevice`
- **Device state / onroad-offroad logic**: `openpilot/system/hardware/hardwared.py`
- **Device paths (logs/params)**: `openpilot/system/hardware/hw.h` (C++ Path helpers used widely)

### KA2-specific code (only)

- **KA2 hardware implementation**: `openpilot/system/hardware/ka2/hardware.py`
  - modem bring-up uses `/usr/kommu/lte/wwan0-setup.sh`
  - SD card: `/dev/mmcblk1p1` formatting support
- **AGNOS updater for KA2**: `openpilot/system/hardware/ka2/agnos.py` + `agnos.json`
  - `launch_chffrplus.sh` picks KA2 manifest when `/KA2` exists
- **Status LED service**: `openpilot/system/hardware/ka2/status_led/indicatord.py`
- **APN override loop**: `openpilot/system/hardware/ka2/setapn.py`

### Repo top-level map (what’s where)

- **Core runtime**
  - `openpilot/` (python package; many “system/*” modules live here)
  - `system/` and `selfdrive/` (major subsystems; driving stack is mostly in `selfdrive/`)
- **IPC / schemas**: `cereal/` (capnp logs + messaging), `msgq_repo/` (msgq implementation; `msgq` symlink)
- **Vehicle interfaces**: `opendbc_repo/` (`opendbc` symlink)
- **Hardware safety / CAN**: `panda/`
- **Tools**: `tools/` (replay, plots, sim, etc.)
- **Build**: `SConstruct`, `site_scons/`, `compile_commands.json`

### On-device storage locations (KA2)

- **openpilot checkout**: `/data/openpilot`
- **params DB**: `/data/params` (also referenced by `Path::params()` in `openpilot/system/hardware/hw.h`)
- **logs / routes**: default `Path::log_root()` → `/data/media/0/realdata`
- **runtime logs**: `/data/log` (device-level); OS logs under `/var/log`
- **tmp**: `/data/tmp` (created by `/usr/kommu/kommu.sh`)
- **overlay/update staging**: `/data/safe_staging`, `/data/rootfs_overlay*`

### Fast “where is X handled?” pointers

- **Start/stop conditions**: `hardwared.py` publishes `deviceState.started`; `manager.py` uses it to gate processes.
- **Network/modem**: KA2 modem logic in `system/hardware/ka2/hardware.py`; APN policy in `system/hardware/ka2/setapn.py`.
- **Adding a new daemon**: add to `process_config.py` and ensure gating function reflects KA2 needs.

