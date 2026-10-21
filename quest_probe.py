#!/usr/bin/env python3
"""Quest controller probe: connect through Isaac Teleop / CloudXR and print both controllers WITHOUT any robot or
ROS. Use it to check the headset <-> workstation link before driving an arm.

  .venv_quest/bin/python quest_probe.py            # Ctrl+C to quit
  .venv_quest/bin/python quest_probe.py --source udp://0.0.0.0:47710   # controllers from quest_bridge.py send

Per hand: grip position in the operator frame (x forward, y left, z up) and its change since tracking started
[cm], orientation (roll / pitch / yaw, deg), squeeze, trigger, buttons (A/X = primary, B/Y = secondary). Every 5 s a
[rate] line counts how often a fresh controller pose arrived (Hz) and the button presses seen so far.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "quest_teleop"))
from quest_bridge import make_quest  # noqa: E402

BUTTON_NAMES = {"left": ("X", "Y"), "right": ("A", "B")}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hands", nargs="+", choices=["left", "right"], default=["left", "right"])
    parser.add_argument("--hz", type=float, default=5.0, help="print rate")
    parser.add_argument("--read-hz", type=float, default=60.0, help="controller read rate")
    parser.add_argument("--source", default="cloudxr",
                        help="cloudxr = CloudXR on this machine; udp://0.0.0.0:47710 = a quest_bridge.py sender")
    args = parser.parse_args()

    xr = make_quest(args.source, args.hands, "QuestProbe")
    if args.source == "cloudxr":
        print("CloudXR up. Quest Browser -> https://nvidia.github.io/IsaacTeleop/client/v1.3.131/ , Server IP = this "
              "PC, accept the certificate, back, Connect.", flush=True)
    origin = {h: None for h in args.hands}
    last_pos = {h: None for h in args.hands}
    fresh = {h: 0 for h in args.hands}
    presses = {BUTTON_NAMES[h][i]: 0 for h in args.hands for i in (0, 1)}
    prev_buttons = {h: (False, False) for h in args.hands}
    t_print = t_rate = time.perf_counter()
    try:
        while True:
            t0 = time.perf_counter()
            readings = xr.read_all()
            for h in args.hands:
                ok, pos, _, _, _ = readings[h]
                for i, (now, was) in enumerate(zip(xr.buttons[h], prev_buttons[h])):
                    if now and not was:
                        presses[BUTTON_NAMES[h][i]] += 1
                prev_buttons[h] = xr.buttons[h]
                if not ok:
                    origin[h] = last_pos[h] = None
                    continue
                if origin[h] is None:
                    origin[h] = pos.copy()
                if last_pos[h] is None or np.any(pos != last_pos[h]):
                    fresh[h] += 1
                last_pos[h] = pos.copy()
            if t0 - t_print >= 1.0 / args.hz:
                t_print = t0
                lines = []
                for h in args.hands:
                    ok, pos, quat, squeeze, trigger = readings[h]
                    if not ok:
                        lines.append(f"{h:5s}: not tracked")
                        continue
                    rpy = Rotation.from_quat(quat).as_euler("xyz", degrees=True)
                    d = (pos - origin[h]) * 100
                    b = "".join(n if on else "-" for n, on in zip(BUTTON_NAMES[h], xr.buttons[h]))
                    lines.append(f"{h:5s}: pos {np.round(pos, 3)} d[cm] {np.round(d, 1)} rpy {np.round(rpy, 0)} "
                                 f"squeeze {squeeze:.2f} trigger {trigger:.2f} buttons {b}")
                print(" | ".join(lines), flush=True)
            if t0 - t_rate >= 5.0:
                hz = {h: fresh[h] / (t0 - t_rate) for h in args.hands}
                net = (f" | bridge: {xr.received} datagrams, {xr.dropped} lost, last {xr.age() * 1e3:.0f} ms ago"
                       if hasattr(xr, "received") else "")
                print("[rate] fresh poses " + ", ".join(f"{h} {v:.0f} Hz" for h, v in hz.items())
                      + f" | presses {presses}" + net, flush=True)
                fresh = {h: 0 for h in args.hands}
                t_rate = t0
            time.sleep(max(0.0, 1.0 / args.read_hz - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        xr.close()


if __name__ == "__main__":
    main()
