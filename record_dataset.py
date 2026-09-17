#!/usr/bin/env python3
"""Record combined Nero+Orca sim episodes (Nero rest hold, Orca finger teleop).

Forwards to ``orca_teleop/scripts/record_dataset.py --backend combined``.

    conda activate orca
    export ORCAHAND_DESCRIPTION_DIR=/home/keti/workspace/orcahand_description
    python record_dataset.py --local --source mediapipe --show-video --overwrite --fps 15 \\
        --episode-end space --num-episodes 5 \\
        --urdf-path "$ORCAHAND_DESCRIPTION_DIR/v1/models/urdf/orcahand_right.urdf" \\
        --task "wave and flex fingers, arm at rest"

Arm cue comparison: add ``--depth off`` (Orbbec color, image-only palm-width cue) or
``--depth orbbec`` (metric RGB-D palm point). With an explicit ``--depth`` and no
``--root``/``--repo-id`` the dataset goes to ``datasets/nero-orca-sim-mediapipe-depth-<mode>``
so the runs stay side by side.

Real hardware (see ``hardware.py``; check joints with ``mirror_real.py`` first): add
``--nero-output shadow|real`` and/or ``--orca-output shadow|real``. ``shadow`` reads the
robot and shows the command as a ghost in the MuJoCo viewer without sending it.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

ORCA_TELEOP_RECORD = (
    Path(__file__).resolve().parent.parent / "orca_teleop" / "scripts" / "record_dataset.py"
)


def _arg_value(argv: list[str], flag: str) -> str | None:
    for i, arg in enumerate(argv):
        if arg == flag and i + 1 < len(argv):
            return argv[i + 1]
        if arg.startswith(flag + "="):
            return arg.split("=", 1)[1]
    return None


def main() -> None:
    if not ORCA_TELEOP_RECORD.exists():
        raise SystemExit(f"missing {ORCA_TELEOP_RECORD}")
    argv = sys.argv[1:]
    if "--backend" not in argv:
        argv = ["--backend", "combined", *argv]
    if "--root" not in argv and "--repo-id" not in argv:
        name = "nero-orca-sim-mediapipe"
        depth = _arg_value(argv, "--depth")
        if depth is not None:
            name = f"{name}-depth-{depth}"
        argv = [
            *argv,
            "--repo-id",
            f"keti/{name}",
            "--root",
            str(Path(__file__).resolve().parent / "datasets" / name),
        ]
    sys.argv = [str(ORCA_TELEOP_RECORD), *argv]
    runpy.run_path(str(ORCA_TELEOP_RECORD), run_name="__main__")


if __name__ == "__main__":
    main()
