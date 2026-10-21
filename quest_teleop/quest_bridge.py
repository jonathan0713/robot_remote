#!/usr/bin/env python3
"""Quest controllers over the network: CloudXR runs on the machine next to the headset, the teleop elsewhere.

  [headset side]  quest_bridge.py send --to <teleop host>:47710      (CloudXR + QuestController, ~60 Hz UDP)
  [teleop side]   NetworkQuest("0.0.0.0:47710")                       (same read_all() / buttons / close() as
                                                                       QuestController; --quest-source udp://...)

Each datagram is one JSON object: {"v": 1, "seq": n, "t": sender unix time, "hands": {hand: [tracked, [x, y, z],
[qx, qy, qz, qw], squeeze, trigger, primary, secondary]}}, poses in the operator frame (X forward, Y left, Z up).
The receiver keeps only the newest datagram (by seq); when none arrived for --stale-s it reports every hand as not
tracked and no button pressed, so a lost link releases the arms instead of freezing a grip. Use it over a private
network (Tailscale); --allow restricts the accepted sender address.
"""

import argparse
import json
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_PORT = 47710
SIDES = ("left", "right")
UNTRACKED = (False, None, None, 0.0, 0.0)


def parse_addr(text, default_host="0.0.0.0"):
    """'host:port', ':port', 'port' or 'udp://host:port' -> (host, port)."""
    text = text.removeprefix("udp://")
    host, _, port = text.rpartition(":")
    return (host or default_host), int(port or DEFAULT_PORT)


def encode(seq, readings, buttons) -> bytes:
    hands = {}
    for h, (ok, pos, quat, squeeze, trigger) in readings.items():
        prim, sec = buttons.get(h, (False, False))
        hands[h] = [bool(ok), None if pos is None else [float(v) for v in pos],
                    None if quat is None else [float(v) for v in quat], float(squeeze), float(trigger),
                    bool(prim), bool(sec)]
    return json.dumps({"v": 1, "seq": seq, "t": time.time(), "hands": hands}, separators=(",", ":")).encode()


def decode(data: bytes):
    """-> (seq, sender time, {hand: reading tuple}, {hand: (primary, secondary)}); raises ValueError on bad input."""
    msg = json.loads(data)
    if msg.get("v") != 1:
        raise ValueError(f"unknown bridge version {msg.get('v')!r}")
    readings, buttons = {}, {}
    for h, (ok, pos, quat, squeeze, trigger, prim, sec) in msg["hands"].items():
        if h not in SIDES:
            continue
        pos = None if pos is None else np.asarray(pos, dtype=float)
        quat = None if quat is None else np.asarray(quat, dtype=float)
        good = (ok and pos is not None and quat is not None and pos.shape == (3,) and quat.shape == (4,)
                and np.all(np.isfinite(pos)) and np.all(np.isfinite(quat)) and np.linalg.norm(quat) > 1e-6)
        readings[h] = (True, pos, quat / np.linalg.norm(quat), float(squeeze), float(trigger)) if good else UNTRACKED
        buttons[h] = (bool(prim), bool(sec))
    return int(msg["seq"]), float(msg["t"]), readings, buttons


class NetworkQuest:
    """Receiver with QuestController's interface. A background thread keeps the newest datagram."""

    def __init__(self, bind=f"0.0.0.0:{DEFAULT_PORT}", hands=SIDES, stale_s=0.2, allow=None, log=print):
        self.hands = list(hands)
        self.stale_s, self.allow, self.log = stale_s, allow, log
        self.buttons = {h: (False, False) for h in self.hands}
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(parse_addr(bind))
        self._sock.settimeout(0.2)
        self.address = self._sock.getsockname()
        self._lock = threading.Lock()
        self._latest = None          # (receive monotonic time, seq, sender time, readings, buttons)
        self.received = self.dropped = self.rejected = 0
        self.sender = None
        self._closed = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._closed.is_set():
            try:
                data, addr = self._sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if self.allow and addr[0] not in self.allow:
                self.rejected += 1
                continue
            try:
                seq, t_send, readings, buttons = decode(data)
            except (ValueError, KeyError, TypeError):
                self.rejected += 1
                continue
            with self._lock:
                if self._latest is not None and seq <= self._latest[1] and addr == self.sender:
                    continue  # reordered / duplicate
                if self.sender != addr:
                    self.log(f"[bridge] receiving controllers from {addr[0]}:{addr[1]}")
                    self.sender = addr
                elif self._latest is not None:
                    self.dropped += max(0, seq - self._latest[1] - 1)
                self.received += 1
                self._latest = (time.monotonic(), seq, t_send, readings, buttons)

    def age(self):
        """Seconds since the newest datagram arrived (inf before the first)."""
        with self._lock:
            return float("inf") if self._latest is None else time.monotonic() - self._latest[0]

    def read_all(self) -> dict:
        with self._lock:
            latest = self._latest
        if latest is None or time.monotonic() - latest[0] > self.stale_s:
            self.buttons = {h: (False, False) for h in self.hands}
            return {h: UNTRACKED for h in self.hands}
        _, _, _, readings, buttons = latest
        self.buttons = {h: buttons.get(h, (False, False)) for h in self.hands}
        return {h: readings.get(h, UNTRACKED) for h in self.hands}

    def read(self):
        return self.read_all()[self.hands[0]]

    def close(self):
        self._closed.set()
        self._sock.close()
        self._thread.join(timeout=1.0)


def make_quest(source, hands, app_name, log=print):
    """'cloudxr' (this machine runs CloudXR) or 'udp://host:port' (a quest_bridge.py sender elsewhere)."""
    if source == "cloudxr":
        from quest_input import QuestController
        return QuestController(hands, HERE / "cloudxr_quest3.env", app_name=app_name)
    if source.startswith("udp://"):
        q = NetworkQuest(source, hands=hands, log=log)
        log(f"[bridge] listening on udp {q.address[0]}:{q.address[1]} for a quest_bridge.py sender")
        return q
    raise ValueError(f"unknown quest source {source!r} (cloudxr or udp://host:port)")


def send(args):
    from quest_input import QuestController
    host, port = parse_addr(args.to, default_host="")
    if not host:
        sys.exit("--to needs the teleop host, e.g. --to 100.89.140.123:47710")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (socket.gethostbyname(host), port)
    xr = QuestController(list(SIDES), HERE / "cloudxr_quest3.env", app_name="QuestBridge")
    print(f"CloudXR up. Quest Browser -> https://nvidia.github.io/IsaacTeleop/client/v1.3.131/ , Server IP = THIS "
          f"machine's LAN IP, Connect. Sending controllers to udp {addr[0]}:{addr[1]} at {args.hz:.0f} Hz.", flush=True)
    seq, t_report, sent, errors = 0, time.monotonic(), 0, 0
    dt = 1.0 / args.hz
    try:
        while True:
            t0 = time.perf_counter()
            readings = xr.read_all()
            seq += 1
            try:
                sock.sendto(encode(seq, readings, xr.buttons), addr)
                sent += 1
            except OSError as exc:
                errors += 1
                if errors == 1 or errors % 100 == 0:
                    print(f"[bridge] send failed ({exc}) x{errors}", flush=True)
            now = time.monotonic()
            if now - t_report >= 5.0:
                state = ", ".join(f"{h} {'tracked' if readings[h][0] else 'NOT tracked'}" for h in SIDES)
                print(f"[bridge] {sent / (now - t_report):.0f} Hz sent | {state}", flush=True)
                t_report, sent = now, 0
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        xr.close()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("send", help="run CloudXR here and stream the controllers to the teleop host")
    s.add_argument("--to", required=True, help="teleop host:port, e.g. 100.89.140.123:47710 (its Tailscale IP)")
    s.add_argument("--hz", type=float, default=60.0)
    args = p.parse_args(argv)
    if args.cmd == "send":
        send(args)


if __name__ == "__main__":
    main()
