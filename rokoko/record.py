"""Step 2: record the Rokoko right-hand stream to .npz for offline retargeting work.

    python rokoko/record.py [--seconds 30] [--out rokoko/samples/hand_YYYYmmdd_HHMMSS.npz]

Saves ``positions`` (N, 21, 3) metres, ``rotations`` (N, 21, 4) xyzw quaternions, ``t`` (N,)
receive times and ``names`` (21,) joint names in the MediaPipe order (wrist, then
thumb/index/middle/ring/little as proximal, medial, distal, tip). Only the newest queued
packet is kept per loop iteration (non-blocking), like the live teleop.
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rokoko.stream import (  # noqa: E402
    DEFAULT_PORT,
    RIGHT_HAND_JOINTS,
    RokokoReceiver,
    frame_to_positions,
    frame_to_rotations,
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out = Path(args.out or f"rokoko/samples/hand_{datetime.now():%Y%m%d_%H%M%S}.npz")
    out.parent.mkdir(parents=True, exist_ok=True)

    rx = RokokoReceiver(args.port)
    positions, rotations, times = [], [], []
    print(f"recording {args.seconds:.0f}s on :{args.port} -> {out}")
    print("hold the hand open for the first second (retargeter calibration), then move")
    t_start = time.time()
    last_print = 0
    while time.time() - t_start < args.seconds:
        frame = rx.latest()
        if frame is None:
            time.sleep(0.002)
            continue
        pos, rot = frame_to_positions(frame), frame_to_rotations(frame)
        if pos is None or rot is None:
            continue
        positions.append(pos.astype(np.float32))
        rotations.append(rot.astype(np.float32))
        times.append(time.time())
        elapsed = int(time.time() - t_start)
        if elapsed != last_print and elapsed % 5 == 0:
            print(f"  {elapsed:2d}s  {len(times)} frames")
            last_print = elapsed

    if not times:
        print("no frames received; check with rokoko/listen.py")
        return
    np.savez(
        out,
        positions=np.stack(positions),
        rotations=np.stack(rotations),
        t=np.asarray(times),
        names=np.asarray(RIGHT_HAND_JOINTS),
    )
    dt = times[-1] - times[0]
    print(f"saved {len(times)} frames, {len(times) / max(dt, 1e-6):.1f} Hz -> {out}")


if __name__ == "__main__":
    main()
