openpilot in simulator
=====================

openpilot implements a [bridge](run_bridge.py) that allows it to run in the [MetaDrive simulator](https://github.com/metadriverse/metadrive).

## Launching openpilot
For a desktop MetaDrive run, use the two-terminal commands in the MetaDrive
section below. The launch scripts select the checkout virtualenv and share an
isolated params root automatically.

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
|  z   | Toggle left blinker   |
|  x   | Toggle right blinker  |
|  q   | Exit all              |
| wasd | Control manually      |
```

### Lane changes

Press `z` to toggle the left signal, or `x` to toggle the right signal. Press
the same key again to turn it off; press the other key to switch direction.
Reset (`r`) clears the signal. These are turn-signal inputs, not direct lane
change commands: OpenPilot's normal lane-change logic decides if and when a
lane change can start. With Assisted Lane Change enabled, briefly tap `a` for
a left steering confirmation or `d` for a right confirmation while the
system is in `preLaneChange`. The usual guards still apply, including active
lateral control, minimum speed, blindspot and road-edge checks, lane/path
confidence, navigation state, and safety limits.

## MetaDrive

### Launching Metadrive
Start these from the repository root, in order.

**Terminal 1 — openpilot manager and UI**

```bash
./tools/sim/start_metadrive.sh
```

The first run builds desktop prerequisites and Tinygrad OpenCL model files.
Later runs reuse them. The manager's built-in `modeld` is blocked so it cannot
silently run the CPU model.

**Terminal 2 — MetaDrive and GPU modeld**

```bash
./tools/sim/start_metadrive_bridge.sh
```

This starts the rendered simulator when a display is available, then runs
Tinygrad on the OpenCL GPU. It fails with an error if the OpenCL platform has
no GPU device. To run without a window, pass `--headless` to the second
command. If standard input is not a terminal, the supervisor tries `/dev/tty`;
if no usable terminal is available, the simulator stays running without
keyboard controls and can be stopped with a shutdown signal.

The default MetaDrive road is a closed figure-eight. Each lobe is formed from
three connected 90-degree arcs with a 300-meter radius; the paths cross at
grade at the center, and navigation checkpoints repeat around the connected
cycle.
