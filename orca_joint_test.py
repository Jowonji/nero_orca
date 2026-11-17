#!/usr/bin/env python3
"""Move one real Orca joint to +deg / -deg and back, so its direction can be checked by eye.

Used to verify the abduction sign before running the glove on the real hand: the dataset
and retargeter convention (v1 URDF / orca_core) is ``+abd`` → fingertip toward the pinky,
``+thumb_abd`` → thumb away from the palm.

    ../orca_teleop/.venv/bin/python orca_joint_test.py                     # index_abd ±20
    ../orca_teleop/.venv/bin/python orca_joint_test.py --joint ring_abd --deg 15
    ../orca_teleop/.venv/bin/python orca_joint_test.py --joint index_abd --hold 3 --cycles 2

Every other joint is held at its neutral value. The hand returns to 0 and torque is
disabled on exit (also on Ctrl+C).
"""

from __future__ import annotations

import argparse
import logging
import time

from hardware import OrcaCoreHand

DEFAULT_MODEL = "config/orcahand_right/config.yaml"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--joint", default="index_abd")
    parser.add_argument("--deg", type=float, default=20.0, help="amplitude (clamped to the ROM)")
    parser.add_argument("--hold", type=float, default=2.0, help="seconds to hold each pose")
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--steps", type=int, default=40, help="interpolation steps per move")
    parser.add_argument("--model-path", default=DEFAULT_MODEL)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    hand = OrcaCoreHand(args.model_path)
    cfg = hand.config
    if args.joint not in cfg.joint_ids:
        raise SystemExit(f"unknown joint {args.joint!r}; choose from {cfg.joint_ids}")
    lo, hi = cfg.joint_roms_dict[args.joint]
    plus = min(args.deg, hi)
    minus = max(-args.deg, lo)
    base = {j: float(cfg.neutral_position.get(j, 0.0)) for j in cfg.joint_ids}

    def goto(value: float, label: str) -> None:
        target = dict(base)
        target[args.joint] = value
        hand.send_deg(target, num_steps=args.steps)
        time.sleep(0.3)
        read = hand.read_deg().get(args.joint)
        print(f"{args.joint} -> {label:>6} {value:+6.1f} deg   (reads {read:+6.1f})")
        time.sleep(args.hold)

    hand.connect()
    try:
        hand.enable()  # torque on, moves to neutral
        time.sleep(1.0)
        print(f"ROM {args.joint}: {lo}..{hi}; neutral {base[args.joint]:+.1f}")
        for _ in range(args.cycles):
            goto(base[args.joint], "neutral")
            goto(plus, "plus")
            goto(base[args.joint], "neutral")
            goto(minus, "minus")
        goto(base[args.joint], "neutral")
    except KeyboardInterrupt:
        pass
    finally:
        hand.close()  # zero position, torque off, disconnect


if __name__ == "__main__":
    main()
