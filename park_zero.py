#!/usr/bin/env python3
"""Park the real Nero arm and Orca hand at their zero positions (end of a session).

    ../orca_teleop/.venv/bin/python park_zero.py                 # arm + hand
    ../orca_teleop/.venv/bin/python park_zero.py --nero-speed 10 --no-orca
    ../orca_teleop/.venv/bin/python park_zero.py --no-nero       # hand only

Nero: enable at ``--nero-speed`` %, blocking joint move to all-zero (the straight-up pose),
then it is left ENABLED there, as everywhere else in this repo (disable it from the web
controller once it is resting). Orca: torque on, move to 0 on every joint, torque off,
disconnect. Clear the space above the arm first: from the teleop home it swings up.
"""

from __future__ import annotations

import argparse
import logging

import numpy as np

from hardware import NeroOutput, OrcaCoreHand, PyAgxNero

ORCA_CONFIG = "config/orcahand_right/config.yaml"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--nero-can", default="can0")
    parser.add_argument("--nero-speed", type=int, default=10, help="Nero speed percent")
    parser.add_argument("--no-nero", action="store_true")
    parser.add_argument("--no-orca", action="store_true")
    parser.add_argument("--model-path", default=ORCA_CONFIG)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    log = logging.getLogger("park")

    if not args.no_nero:
        nero = NeroOutput("real", PyAgxNero(channel=args.nero_can), args.nero_speed)
        nero.connect()
        q = nero.q
        log.info("Nero at %s deg; moving to 0 at %d%%", np.degrees(q).round(1).tolist(), args.nero_speed)
        nero.move_home(np.zeros(7))
        q = nero.refresh()
        log.info("Nero now at %s deg (left enabled)", np.degrees(q).round(1).tolist())
        nero.close()

    if not args.no_orca:
        hand = OrcaCoreHand(args.model_path)
        hand.connect()
        try:
            hand.enable()  # torque on; moves to the neutral pose first
            log.info("Orca hand at neutral; moving to 0 and releasing torque")
        finally:
            hand.close()  # zero position, torque off, disconnect
        log.info("Orca hand parked")


if __name__ == "__main__":
    main()
