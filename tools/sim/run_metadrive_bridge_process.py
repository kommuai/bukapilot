#!/usr/bin/env python3
"""Supervise MetaDrive so terminal shutdown also stops its worker process."""

import os
import signal
import sys
import termios
from multiprocessing import Queue

from openpilot.tools.sim.bridge.common import control_cmd_gen
from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge
from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread
from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread
from openpilot.tools.sim.run_bridge import parse_args


class BridgeProcessExited(Exception):
  pass


_simulator_process = None
_shutdown_requested = False


class SupervisedMetaDriveBridge(MetaDriveBridge):
  def _on_shutdown(self, signum, frame) -> None:
    global _shutdown_requested
    _shutdown_requested = True
    self.shutdown()

  def bridge_keep_alive(self, q: Queue, retries: int) -> None:
    # Isolate only the simulator workers. The parent stays attached to the
    # terminal so its keyboard reader remains in the foreground process group.
    signal.signal(signal.SIGTERM, self._on_shutdown)
    os.setpgrp()
    super().bridge_keep_alive(q, retries)


def _request_shutdown(signum, frame) -> None:
  global _shutdown_requested
  _shutdown_requested = True


def _stop_on_signal(signum, frame) -> None:
  signal.signal(signum, signal.SIG_IGN)
  raise KeyboardInterrupt


def _bridge_child_changed(signum, frame) -> None:
  if _simulator_process is not None and not _simulator_process.is_alive():
    raise BridgeProcessExited


def _is_tty(stream) -> bool:
  try:
    return stream.isatty()
  except (OSError, ValueError):
    return False


def _run_keyboard_controls(queue: Queue) -> bool:
  """Use stdin or the controlling terminal; return false if neither is usable."""
  stream = sys.stdin
  opened_stream = None

  if not _is_tty(stream):
    try:
      stream = open("/dev/tty", "r")
    except OSError as exc:
      print(
        "Keyboard controls unavailable: stdin is not a TTY and "
        f"/dev/tty could not be opened ({exc}).",
        file=sys.stderr, flush=True,
      )
      return False
    opened_stream = stream
    if not _is_tty(stream):
      stream.close()
      print("Keyboard controls unavailable: /dev/tty is not a TTY.", file=sys.stderr, flush=True)
      return False

  original_stdin = sys.stdin
  try:
    sys.stdin = stream
    try:
      termios.tcgetattr(stream.fileno())
      keyboard_poll_thread(queue)
      return True
    except (OSError, termios.error) as exc:
      print(
        f"Keyboard controls unavailable ({exc}); MetaDrive will keep running "
        "until its worker exits or the supervisor receives SIGINT/SIGTERM.",
        file=sys.stderr, flush=True,
      )
      return False
  finally:
    sys.stdin = original_stdin
    if opened_stream is not None:
      opened_stream.close()


def _monitor_simulator_worker(process) -> None:
  print(
    "MetaDrive is running without keyboard controls; waiting for the simulator "
    "worker or a shutdown signal.",
    file=sys.stderr, flush=True,
  )
  while process.is_alive():
    process.join(timeout=0.5)
  raise BridgeProcessExited


def main() -> None:
  global _simulator_process
  signal.signal(signal.SIGINT, _request_shutdown)
  signal.signal(signal.SIGTERM, _request_shutdown)
  args = parse_args()
  if _shutdown_requested:
    raise SystemExit(130)
  simulator_bridge = SupervisedMetaDriveBridge(args.dual_camera, args.high_quality, render=args.render)
  signal.signal(signal.SIGINT, _request_shutdown)
  signal.signal(signal.SIGTERM, _request_shutdown)
  if _shutdown_requested:
    raise SystemExit(130)
  queue = Queue()
  # MetaDriveBridge.__init__ installs its own SIGTERM handler; use a deferred
  # request until the fork completes so startup signals cannot orphan a worker.
  simulator_process = simulator_bridge.run(queue)

  _simulator_process = simulator_process
  bridge_exit_status = 0

  try:
    signal.signal(signal.SIGINT, _stop_on_signal)
    signal.signal(signal.SIGTERM, _stop_on_signal)
    signal.signal(signal.SIGCHLD, _bridge_child_changed)
    if _shutdown_requested:
      raise KeyboardInterrupt
    if not simulator_process.is_alive():
      raise BridgeProcessExited
    if args.joystick:
      wheel_poll_thread(queue)
    elif not _run_keyboard_controls(queue):
      _monitor_simulator_worker(simulator_process)
  except KeyboardInterrupt:
    print("\nStopping MetaDrive simulator...", flush=True)
  except BridgeProcessExited:
    bridge_exit_status = simulator_process.exitcode or 0
    print(f"MetaDrive bridge worker exited with status {simulator_process.exitcode}.", file=sys.stderr, flush=True)
  finally:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    signal.signal(signal.SIGCHLD, signal.SIG_DFL)
    queue.put(control_cmd_gen("quit"))
    simulator_bridge.shutdown()
    simulator_process.join(timeout=15)

    # A timed-out worker may still own a MetaDrive grandchild. Signal that
    # worker-only process group; it is separate from the terminal's input group.
    try:
      os.killpg(simulator_process.pid, signal.SIGTERM)
    except ProcessLookupError:
      pass

    simulator_process.join(timeout=5)
    try:
      os.killpg(simulator_process.pid, 0)
    except ProcessLookupError:
      pass
    else:
      os.killpg(simulator_process.pid, signal.SIGKILL)
    if simulator_process.is_alive():
      simulator_process.join()
    queue.close()
    queue.join_thread()

  if bridge_exit_status != 0:
    raise SystemExit(bridge_exit_status)


if __name__ == "__main__":
  main()
