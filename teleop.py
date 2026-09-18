#!/usr/bin/env python3
"""Webcam teleop for Nero+Orca without recording: sim, shadow, or the real robot.

MediaPipe (optionally with Orbbec depth) drives the Nero arm through wrist-position
IK and the Orca fingers through the retargeter, via ``CombinedNeroOrcaSink``. The
webcam window and the MuJoCo viewer stay open; nothing is written to disk.

Defaults are safe: the arm is ``shadow`` (reads the real arm, draws the command as
a ghost, sends nothing) and the hand is ``sim``. ``real`` asks for confirmation.

    export ORCAHAND_DESCRIPTION_DIR=$HOME/workspace/orcahand_description
    # CAN first: sudo ip link set can0 up type can bitrate 1000000

    ../orca_teleop/.venv/bin/python teleop.py --nero-output sim                 # sim only
    ../orca_teleop/.venv/bin/python teleop.py                                   # arm shadow
    ../orca_teleop/.venv/bin/python teleop.py --nero-output real --nero-speed 10
    ../orca_teleop/.venv/bin/python teleop.py --nero-output real --fake-hardware  # no robot

Stop with Ctrl+C in this terminal. The real Nero stays enabled at its last pose.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

from combined_sink import CombinedNeroOrcaSink, _orca_core_config_path
from hardware import OUTPUT_MODES

DEPTH_MODES = ("auto", "orbbec", "off", "webcam")


def _default_urdf() -> str | None:
    root = os.environ.get("ORCAHAND_DESCRIPTION_DIR")
    if not root:
        return None
    path = Path(root) / "v1" / "models" / "urdf" / "orcahand_right.urdf"
    return str(path) if path.exists() else None


def _confirm_real(args: argparse.Namespace) -> None:
    real = [name for name, mode in (("nero", args.nero_output), ("orca", args.orca_output)) if mode == "real"]
    if not real or args.fake_hardware or args.yes:
        return
    print(
        f"\nREAL OUTPUT for {', '.join(real)}: the robot will move "
        f"(Nero homes to rest first, speed {args.nero_speed}%). Keep the emergency stop in reach."
    )
    if not sys.stdin.isatty():
        raise SystemExit("real output needs an interactive confirmation (or pass --yes)")
    if input("Type 'yes' to continue: ").strip().lower() != "yes":
        raise SystemExit("aborted")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--nero-output", choices=OUTPUT_MODES, default="shadow")
    parser.add_argument("--orca-output", choices=OUTPUT_MODES, default="sim")
    parser.add_argument("--nero-can", default="can0", help="SocketCAN channel (default can0)")
    parser.add_argument("--nero-speed", type=int, default=10, help="Nero speed percent (default 10)")
    parser.add_argument("--model-path", default=None, help="orca_core hand config.yaml for hardware")
    parser.add_argument("--urdf-path", default=_default_urdf(), help="Orca hand URDF for retargeting")
    parser.add_argument("--depth", choices=DEPTH_MODES, default="auto", help="MediaPipe depth cue")
    parser.add_argument("--camera", type=int, default=None, help="OpenCV webcam index (depth=webcam/auto)")
    parser.add_argument("--hand", choices=["right"], default="right")
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--control-hz", type=float, default=15.0)
    parser.add_argument("--no-show-video", dest="show_video", action="store_false",
                        help="hide the webcam window")
    parser.add_argument("--no-viewer", dest="viewer", action="store_false",
                        help="hide the MuJoCo viewer")
    parser.add_argument("--fake-hardware", action="store_true",
                        help="use in-memory fake Nero/Orca for shadow/real (no robot)")
    parser.add_argument("--yes", action="store_true", help="skip the real-output confirmation")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(name)s | %(message)s", datefmt="%H:%M:%S"
    )
    if args.urdf_path is None:
        parser.error("--urdf-path is required (or export ORCAHAND_DESCRIPTION_DIR)")
    _confirm_real(args)

    nero_device = orca_device = None
    if args.fake_hardware:
        from orca_core import OrcaHandConfig

        from hardware import FakeNero, FakeOrca

        cfg = OrcaHandConfig.from_config_path(config_path=str(_orca_core_config_path("right")))
        nero_device = FakeNero() if args.nero_output != "sim" else None
        if args.orca_output != "sim":
            orca_device = FakeOrca(cfg, {jid: float(v) for jid, v in cfg.neutral_position.items()})

    sink = CombinedNeroOrcaSink(
        control_hz=args.control_hz,
        show_viewer=args.viewer,
        nero_output=args.nero_output,
        orca_output=args.orca_output,
        nero_can=args.nero_can,
        nero_speed_percent=args.nero_speed,
        orca_model_path=args.model_path,
        nero_device=nero_device,
        orca_device=orca_device,
    )

    from orca_teleop.pipeline import run_local

    # model_path=None lets the retargeter use the sink's hand config (sim v2 or the hardware one).
    run_local(
        model_path=None,
        urdf_path=args.urdf_path,
        port=args.port,
        handedness=args.hand,
        show_video=args.show_video,
        sink=sink,
        camera_index=args.camera,
        depth=args.depth,
    )


if __name__ == "__main__":
    main()
