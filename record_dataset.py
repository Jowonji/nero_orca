#!/usr/bin/env python3
"""Record combined Nero+Orca sim episodes (Nero rest hold, Orca finger teleop).

Forwards to ``orca_teleop/scripts/record_dataset.py --backend combined``.

    conda activate orca
    export ORCAHAND_DESCRIPTION_DIR=/home/keti/workspace/orcahand_description
    python record_dataset.py --local --source mediapipe --show-video --overwrite --fps 15 \\
        --episode-end space --num-episodes 5 \\
        --urdf-path "$ORCAHAND_DESCRIPTION_DIR/v1/models/urdf/orcahand_right.urdf" \\
        --task "wave and flex fingers, arm at rest"
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

ORCA_TELEOP_RECORD = (
    Path(__file__).resolve().parent.parent / "orca_teleop" / "scripts" / "record_dataset.py"
)


def main() -> None:
    if not ORCA_TELEOP_RECORD.exists():
        raise SystemExit(f"missing {ORCA_TELEOP_RECORD}")
    argv = sys.argv[1:]
    if "--backend" not in argv:
        argv = ["--backend", "combined", *argv]
    if "--root" not in argv and "--repo-id" not in argv:
        argv = [
            *argv,
            "--repo-id",
            "keti/nero-orca-sim-mediapipe",
            "--root",
            str(Path(__file__).resolve().parent / "datasets" / "nero-orca-sim-mediapipe"),
        ]
    sys.argv = [str(ORCA_TELEOP_RECORD), *argv]
    runpy.run_path(str(ORCA_TELEOP_RECORD), run_name="__main__")


if __name__ == "__main__":
    main()
