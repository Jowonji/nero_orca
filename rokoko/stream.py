"""Rokoko Studio custom-stream receiver (UDP, JSON v3, no LZ4).

Studio sends one JSON frame per datagram (~10 KB, 30 Hz with the current Studio setting) to
this machine's port 14043. ``RokokoReceiver`` is non-blocking: ``latest()`` drains the socket
and returns only the newest frame, so a control loop never works on stale packets.

    rx = RokokoReceiver()            # binds 0.0.0.0:14043
    frame = rx.latest()              # dict or None (nothing new since the last call)
    pos = frame_to_positions(frame)  # (21, 3) metres, MediaPipe joint order
"""

from __future__ import annotations

import json
import socket
from typing import Any

import numpy as np

DEFAULT_PORT = 14043

FINGERS = ("Thumb", "Index", "Middle", "Ring", "Little")
SEGMENTS = ("Proximal", "Medial", "Distal", "Tip")
# Same order as MediaPipe's 21 hand landmarks: wrist, then per finger proximal..tip.
RIGHT_HAND_JOINTS = ["rightHand"] + [f"right{f}{s}" for f in FINGERS for s in SEGMENTS]


class RokokoReceiver:
    def __init__(self, port: int = DEFAULT_PORT, host: str = "0.0.0.0") -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.setblocking(False)
        self.port = port
        self.packets = 0
        self.dropped = 0  # older packets discarded by latest()

    def latest_raw(self) -> bytes | None:
        """Newest queued datagram, discarding older ones; None if nothing is queued."""
        latest = None
        while True:
            try:
                latest, _ = self._sock.recvfrom(65535)
            except BlockingIOError:
                return latest
            self.packets += 1
            if latest is not None:
                self.dropped += 1
        return latest

    def latest(self) -> dict[str, Any] | None:
        raw = self.latest_raw()
        if raw is None:
            return None
        self.dropped -= 1  # the one we return was counted as dropped above
        try:
            return json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError):
            return None

    def close(self) -> None:
        self._sock.close()


def _actor_body(frame: dict[str, Any], actor: int) -> dict[str, Any] | None:
    actors = frame.get("scene", {}).get("actors", [])
    if len(actors) <= actor:
        return None
    return actors[actor].get("body", {})


def frame_to_positions(frame: dict[str, Any], actor: int = 0) -> np.ndarray | None:
    """(21, 3) right-hand positions (metres, Rokoko frame) or None if the frame is incomplete."""
    body = _actor_body(frame, actor)
    if body is None:
        return None
    out = np.empty((21, 3), dtype=float)
    for i, name in enumerate(RIGHT_HAND_JOINTS):
        j = body.get(name)
        if j is None:
            return None
        p = j["position"]
        out[i] = (p["x"], p["y"], p["z"])
    return out


def frame_to_rotations(frame: dict[str, Any], actor: int = 0) -> np.ndarray | None:
    """(21, 4) right-hand rotations as xyzw quaternions, or None if incomplete."""
    body = _actor_body(frame, actor)
    if body is None:
        return None
    out = np.empty((21, 4), dtype=float)
    for i, name in enumerate(RIGHT_HAND_JOINTS):
        j = body.get(name)
        if j is None:
            return None
        r = j["rotation"]
        out[i] = (r["x"], r["y"], r["z"], r["w"])
    return out
