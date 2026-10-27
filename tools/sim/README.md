openpilot in simulator
=====================

openpilot implements a [bridge](run_bridge.py) that allows it to run in the [MetaDrive simulator](https://github.com/metadriverse/metadrive).

## Launching openpilot
For a desktop MetaDrive run, use the complete recipe in the MetaDrive
section below. It sets the bukapilot virtualenv and the isolated params root
needed by this branch.

## Bridge usage
```
$ ./run_bridge.py -h
usage: run_bridge.py [-h] [--joystick] [--high_quality] [--dual_camera]
                     [--render | --headless]
Bridge between the simulator and openpilot.

options:
  -h, --help            show this help message and exit
  --joystick
  --high_quality
  --dual_camera
  --render        Show the MetaDrive simulator window
  --headless      Run without a MetaDrive window
```

#### Bridge Controls:
- To engage openpilot press 2, then press 1 to increase the speed and 2 to decrease.
- To disengage, press "S" (simulates a user brake)

#### All inputs:

```
| key  |   functionality       |
|------|-----------------------|
|  1   | Cruise Resume / Accel |
|  2   | Cruise Set    / Decel |
|  3   | Cruise Cancel         |
|  r   | Reset Simulation      |
|  i   | Toggle Ignition       |
|  q   | Exit all              |
| wasd | Control manually      |
```

## MetaDrive

### Launching Metadrive
On a desktop checkout, use bukapilot's virtualenv and an isolated params
directory. This branch otherwise tries to register with the Kommu backend
before starting the manager:

``` bash
export PATH="$PWD/.venv/bin:$PATH"
export PARAMS_ROOT=/tmp/bukapilot-metadrive-params
mkdir -p "$PARAMS_ROOT"

# Build the desktop Vision IPC, logger, MPC, and locationd artifacts once.
scons -u -j"$(nproc)" \
  msgq_repo/msgq/visionipc/visionipc_pyx.so \
  system/loggerd/bootlog \
  selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so \
  selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so \
  selfdrive/locationd/models/generated/libpose.so \
  selfdrive/locationd/models/generated/libcar.so

# launch_openpilot.sh seeds the offline simulator identity, selects the
# tinygrad CPU model, blocks desktop-incompatible services, and starts the
# openpilot UI when a display is available.
./tools/sim/launch_openpilot.sh
```

In a second terminal, from the same checkout, start the bridge:

``` bash
export PATH="$PWD/.venv/bin:$PATH"
export PARAMS_ROOT=/tmp/bukapilot-metadrive-params
./tools/sim/run_bridge.py --render
# ./tools/sim/run_bridge.py --headless  # explicit headless mode
```
