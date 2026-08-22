#!/usr/bin/env python3
"""Live KA2 AE monitor — grey, target, integration lines, gain."""
import sys
import time

from cereal import messaging


def main() -> int:
    cams = ["wideRoadCameraState", "roadCameraState", "driverCameraState"]
    names = ["wide", "road", "driver"]
    sm = messaging.SubMaster(cams, poll="roadCameraState")
    interval = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    print("time_s  cam   grey  target  integ  gain   dc  frame")
    t0 = time.time()
    while True:
        sm.update(int(interval * 1000))
        for cam, name in zip(cams, names):
            if not sm.valid[cam]:
                continue
            m = sm[cam]
            print(
                f"{time.time()-t0:6.1f}  {name:5s}  {m.measuredGreyFraction:5.3f}  "
                f"{m.targetGreyFraction:5.3f}  {m.integLines:5d}  {m.gain:5.2f}  "
                f"{'Y' if m.highConversionGain else 'N':2s}  {m.frameId}",
                flush=True,
            )
        print(flush=True)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
