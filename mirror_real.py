#!/usr/bin/env python3
"""Mirror the real Nero arm / Orca hand in the MuJoCo viewer (read-only).

Nothing is enabled or commanded. Use it to check that model and robot agree:
move the arm by hand (or with its own tools) and compare the viewer. The
translucent ghost shows the rest pose, and the terminal prints real − rest per
joint. A joint that moves the wrong way needs ``NERO_JOINT_SIGN`` in
``hardware.py``; a constant gap at a known pose needs ``NERO_JOINT_OFFSET``.

    # CAN up first:  sudo ip link set can0 up type can bitrate 1000000
    ../orca_teleop/.venv/bin/python mirror_real.py                 # Nero + Orca
    ../orca_teleop/.venv/bin/python mirror_real.py --orca sim      # Nero only
    ../orca_teleop/.venv/bin/python mirror_real.py --nero sim      # Orca only
    ../orca_teleop/.venv/bin/python mirror_real.py --fake          # no hardware, demo motion
"""

from __future__ import annotations

import argparse
import logging
import math
import time

import numpy as np
from orca_core import OrcaHandConfig

from combined_sink import NERO_JOINT_IDS, CombinedNeroOrcaSink, _orca_core_config_path
from hardware import FakeNero, FakeOrca
from limits import REST_QPOS


class _WavingFakeNero(FakeNero):
    """Fake arm that sways joints 1 and 4 around rest so the mirror visibly moves."""

    def read_q(self):
        t = time.monotonic()
        q = np.array(REST_QPOS, dtype=np.float64)
        q[0] += 0.4 * math.sin(0.8 * t)
        q[3] += 0.3 * math.sin(0.5 * t)
        return q, t


def _print_table(sink: CombinedNeroOrcaSink, show_hand: bool) -> None:
    state = sink.joint_state()
    ids = sink.joint_ids
    rest = np.degrees(REST_QPOS)
    lines = ["joint        real(deg)  rest(deg)  real-rest"]
    for i, name in enumerate(NERO_JOINT_IDS):
        lines.append(f"{name:12s} {state[i]:9.2f} {rest[i]:10.2f} {state[i] - rest[i]:10.2f}")
    if show_hand:
        hand = "  ".join(f"{jid}={state[i]:.0f}" for i, jid in enumerate(ids) if i >= 7)
        lines.append(f"orca: {hand}")
    print("\n".join(lines) + "\n", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--nero", choices=["shadow", "sim"], default="shadow")
    parser.add_argument("--orca", choices=["shadow", "sim"], default="shadow")
    parser.add_argument("--nero-can", default="can0", help="SocketCAN channel (default can0)")
    parser.add_argument("--orca-model-path", default=None, help="orca_core hand model dir")
    parser.add_argument("--print-hz", type=float, default=2.0)
    parser.add_argument("--fake", action="store_true", help="fake devices, no hardware needed")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    nero_device = orca_device = None
    if args.fake:
        cfg = OrcaHandConfig.from_config_path(config_path=str(_orca_core_config_path("right")))
        nero_device = _WavingFakeNero()
        orca_device = FakeOrca(cfg, {jid: float(v) for jid, v in cfg.neutral_position.items()})

    sink = CombinedNeroOrcaSink(
        nero_output=args.nero,
        orca_output=args.orca,
        nero_can=args.nero_can,
        orca_model_path=args.orca_model_path,
        nero_device=nero_device if args.nero != "sim" else None,
        orca_device=orca_device if args.orca != "sim" else None,
        show_viewer=True,
    )
    sink.connect()
    if not sink.viewer_running:
        sink.close()
        raise SystemExit("MuJoCo viewer did not open (DISPLAY set?)")
    print("Mirroring real joints (read-only). Close the viewer or Ctrl+C to stop.\n")
    period = 1.0 / max(args.print_hz, 0.1)
    try:
        while sink.viewer_running:
            sink.refresh()
            _print_table(sink, show_hand=args.orca != "sim")
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        sink.close()


if __name__ == "__main__":
    main()
