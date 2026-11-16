"""Step 1: check that the Rokoko Studio custom stream (UDP, JSON v3, no LZ4) reaches this machine.

    python rokoko/listen.py [--port 14043] [--seconds 10]

Prints the first packet's actors/joint names, the value ranges of the right-hand joints and the
receive rate.
"""

import argparse
import json
import socket
import time

FINGERS = ("Thumb", "Index", "Middle", "Ring", "Little")
SEGMENTS = ("Proximal", "Medial", "Distal", "Tip")
RIGHT_HAND = ["rightHand"] + [f"right{f}{s}" for f in FINGERS for s in SEGMENTS]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14043)
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(1.0)
    print(f"listening on 0.0.0.0:{args.port} for {args.seconds:.0f}s")

    first = None
    n = 0
    t0 = None
    lo, hi = {}, {}
    end = time.time() + args.seconds
    while time.time() < end:
        try:
            raw, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        now = time.time()
        if t0 is None:
            t0 = now
            print(f"first packet from {addr[0]}:{addr[1]}, {len(raw)} bytes")
        n += 1
        try:
            msg = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError):
            print(f"not JSON (LZ4 on?): {raw[:32]!r}")
            continue
        if first is None:
            first = msg
            print("top-level keys:", list(msg))
            print("version:", msg.get("version"))
            actors = msg.get("scene", {}).get("actors", [])
            print("actors:", [a.get("name") for a in actors])
            if actors:
                body = actors[0].get("body", {})
                print(f"body joints ({len(body)}):", sorted(body))
                missing = [j for j in RIGHT_HAND if j not in body]
                print("missing right-hand joints:", missing or "none")
        body = msg.get("scene", {}).get("actors", [{}])[0].get("body", {})
        for j in RIGHT_HAND:
            p = body.get(j, {}).get("position")
            if p:
                v = (p["x"], p["y"], p["z"])
                lo[j] = tuple(map(min, lo.get(j, v), v))
                hi[j] = tuple(map(max, hi.get(j, v), v))

    if not n:
        print("no packets. Check: sudo tcpdump -i any udp port", args.port, "-c 5")
        return
    dt = time.time() - t0
    print(f"\n{n} packets in {dt:.1f}s = {n / dt:.1f} Hz")
    print("position ranges (x, y, z):")
    for j in RIGHT_HAND:
        if j in lo:
            print(f"  {j:22s} " + "  ".join(f"[{a:8.3f},{b:8.3f}]" for a, b in zip(lo[j], hi[j])))
    if first:
        b = first["scene"]["actors"][0]["body"]
        print("\nsample rightHand:", b.get("rightHand"))
        print("sample rightIndexTip:", b.get("rightIndexTip"))


if __name__ == "__main__":
    main()
