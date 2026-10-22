#!/usr/bin/env python3
"""Operator view for the Quest teleop: camera topics tiled in one window, with the teleop and pick status.

Defaults are the OpenArm held-tote ones (the operator's choice, 2026-10-21): the chest D435, the two operator-only
side cameras (ITRI run_tote_pick_scene.py --operator-cameras) and the right wrist (for the final alignment), and a
status bar
from /quest_teleop/state (arm engaged / tracked / lost, gripper opening, recording) and /held_tote_teleop/status
(item over the rim, hold time, and the gripper's depth: its lowest point over the tote floor, also as a bar at the
right edge of the cameras next to the item's top). Run it on the machine the operator watches (Moonlight shows its desktop), with the
chain's ROS environment.

  source /opt/ros/humble/setup.bash
  python3 quest_teleop/operator_view.py            # keys: l = left wrist in place of the right wrist, q = quit

A tile shows "no image" until its topic publishes, and turns grey with its age when it goes stale. With
--runner-status (the ITRI round runner passes its <out>/runner_status.json) the runner's phase heads the status bar,
and when the next attempt can start a green banner over the cameras says so (A on the Quest starts it).
"""

import argparse
import json
import time

import numpy as np

DEFAULT_TILES = [("chest", "/camera_center/rgb"), ("left side", "/operator/side_left/rgb"),
                 ("right side", "/operator/side_right/rgb"), ("right wrist", "/camera_right_hand/rgb")]
EXTRA_TILES = {ord("l"): ("left wrist", "/camera_left_hand/rgb")}


def image_to_bgr(msg) -> np.ndarray | None:
    """sensor_msgs/Image (rgb8 / bgr8 / rgba8 / bgra8) -> HxWx3 BGR uint8."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if channels is None:
        return None
    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
    a = a.reshape(msg.height, msg.width, channels)[..., :3]
    return a[..., ::-1] if msg.encoding.startswith("rgb") else a


READY_PHASES = ("ready",)
RUNNER_COLOURS = {"ready": (0, 255, 0), "restoring": (0, 255, 255), "writing": (0, 255, 255),
                  "teleop": (255, 255, 255), "practice": (255, 255, 255), "done": (200, 200, 200)}


def read_runner_status(path) -> dict | None:
    """The round runner's status file (ITRI run_held_tote_teleop.py), or None when absent, unreadable or its runner
    process is gone."""
    import os

    if path is None:
        return None
    try:
        with open(path) as f:
            status = json.loads(f.read())
        os.kill(int(status["pid"]), 0)
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return status


def runner_lines(runner: dict | None) -> list:
    """[(text, (b, g, r))]: what the round runner is doing between and around the attempts."""
    if runner is None:
        return []
    lines = [(runner.get("text", ""), RUNNER_COLOURS.get(runner.get("phase"), (255, 255, 255)))]
    if runner.get("last"):
        lines.append((f"last: {runner['last']}", (200, 200, 200)))
    return lines


def status_lines(state: dict | None, pick: dict | None, now: float, state_t: float, pick_t: float,
                 runner: dict | None = None) -> list:
    """[(text, (b, g, r))] for the status bar; the runner's line first. Between attempts (runner not in its teleop
    phase) no teleop runs, so its missing state is not an error then."""
    lines = runner_lines(runner)
    between = runner is not None and runner.get("phase") not in ("teleop", "practice")
    if state is None or now - state_t > 1.0:
        if not between:
            lines.append(("teleop: no /quest_teleop/state", (0, 0, 255)))
    else:
        arms = []
        for side in ("left", "right"):
            if side in state:
                s = state[side]
                grip = state.get("gripper", {}).get(side)
                arms.append(f"{side}: {s.upper()}" + (f" gripper {grip * 100:.0f}% open" if grip is not None else ""))
        recording = state.get("capturing")
        lines.append(("  ".join(arms) + ("   |   RECORDING" if recording else "   |   not recording (A = start)"),
                      (0, 0, 255) if recording else (255, 255, 255)))
        guard = state.get("guard") or {}
        if guard.get("holding"):
            lines.append((f"GUARD: arms {guard['clearance_m'] * 100:.1f} cm apart - move the right hand away from the "
                          "left arm (release and grip again)", (0, 0, 255)))
    if pick is not None and now - pick_t < 1.0 and pick.get("clearance_m") is not None:
        done = pick.get("succeeded")
        lines.append((f"item {pick['clearance_m'] * 100:+.1f} cm over the rim (need 2.0)   held "
                      f"{pick.get('hold_s', 0.0):.1f} / 2.0 s" + ("   DONE" if done else ""),
                      (0, 255, 0) if done else (0, 255, 255)))
        if pick.get("tip_above_floor_m") is not None:
            h = pick["tip_above_floor_m"]
            lines.append((f"gripper {h * 100:+.1f} cm over the floor   item top {pick['item_top_above_floor_m'] * 100:.1f} cm"
                          f"   TCP {pick['tcp_off_item_centre_m'] * 100:.1f} cm off the item centre",
                          (0, 0, 255) if h < 0.01 else (0, 255, 255) if h < 0.025 else (255, 255, 255)))
    return lines


def depth_gauge(canvas: np.ndarray, pick: dict | None, full_m: float = 0.20, tiles_h: int = 720) -> np.ndarray:
    """A depth bar at the right edge of the camera tiles: the tote floor at the bottom, the item's top (yellow) and the
    gripper's lowest point (green; yellow under 2.5 cm, red under 1 cm over the floor). The cameras give no depth."""
    import cv2

    if not pick or pick.get("tip_above_floor_m") is None:
        return canvas
    w = canvas.shape[1]
    x0, x1, top, bottom = w - 70, w - 40, 60, min(canvas.shape[0], tiles_h) - 60
    y = lambda m: int(bottom - (bottom - top) * min(max(m / full_m, 0.0), 1.0))
    cv2.rectangle(canvas, (x0 - 6, top - 30), (x1 + 34, bottom + 24), (0, 0, 0), -1)
    cv2.rectangle(canvas, (x0, top), (x1, bottom), (120, 120, 120), 1)
    cv2.line(canvas, (x0 - 4, bottom), (x1 + 4, bottom), (255, 255, 255), 2)
    cv2.putText(canvas, "floor", (x0 - 4, bottom + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    item = pick.get("item_top_above_floor_m")
    if item is not None:
        cv2.rectangle(canvas, (x0 + 4, y(item)), (x1 - 4, bottom), (0, 160, 160), -1)
        cv2.putText(canvas, "item", (x0 - 2, y(item) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    h = pick["tip_above_floor_m"]
    colour = (0, 0, 255) if h < 0.01 else (0, 255, 255) if h < 0.025 else (0, 255, 0)
    cv2.line(canvas, (x0 - 6, y(h)), (x1 + 6, y(h)), colour, 3)
    cv2.putText(canvas, f"{h * 100:.1f}", (x1 + 2, y(h) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
    cv2.putText(canvas, "cm", (x0 - 2, top - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    return canvas


def overlay(canvas: np.ndarray, text: str, colour=(0, 255, 0), tiles_h: int = 720) -> np.ndarray:
    """A banner across the middle of the camera tiles (the operator watches the cameras, not the status bar)."""
    import cv2

    w = canvas.shape[1]
    y = min(canvas.shape[0], tiles_h) // 2
    cv2.rectangle(canvas, (0, y - 45), (w, y + 45), (0, 0, 0), -1)
    cv2.rectangle(canvas, (0, y - 45), (w, y + 45), colour, 3)
    size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)[0]
    cv2.putText(canvas, text, (max(10, (w - size[0]) // 2), y + size[1] // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                colour, 3)
    return canvas


def compose(tiles: list, frames: dict, stamps: dict, lines: list, now: float, tile=(640, 360)) -> np.ndarray:
    import cv2

    w, h = tile
    canvas = np.zeros((h * 2 + 30 * max(1, len(lines)) + 10, w * 2, 3), dtype=np.uint8)
    for i, (label, topic) in enumerate(tiles):
        x, y = (i % 2) * w, (i // 2) * h
        if topic is None:  # free tile
            continue
        frame = frames.get(topic)
        if frame is None:
            cv2.putText(canvas, f"{label}: no image ({topic})", (x + 20, y + h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (120, 120, 120), 1)
            continue
        im = cv2.resize(frame, (w, h))
        age = now - stamps[topic]
        if age > 1.0:
            im = (im * 0.35).astype(np.uint8)
        canvas[y:y + h, x:x + w] = im
        cv2.rectangle(canvas, (x, y), (x + w, y + 24), (0, 0, 0), -1)
        cv2.putText(canvas, label + (f"  (stale {age:.0f} s)" if age > 1.0 else ""), (x + 8, y + 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
    for j, (text, colour) in enumerate(lines):
        cv2.putText(canvas, text, (12, h * 2 + 26 + 30 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
    return canvas


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topics", nargs="+", metavar="TOPIC", help="up to four image topics (top left, top right, "
                   "bottom left, bottom right) instead of the held-tote defaults")
    p.add_argument("--state-topic", default="/quest_teleop/state")
    p.add_argument("--pick-topic", default="/held_tote_teleop/status")
    p.add_argument("--hz", type=float, default=15.0)
    p.add_argument("--scale", type=float, default=1.0, help="window scale (1.0 = 1280 x ~800)")
    p.add_argument("--runner-status", help="the round runner's status JSON (phase, text, pid): its line in the "
                   "status bar, and a banner over the cameras when the next attempt can start (A)")
    args, ros_args = p.parse_known_args(argv)

    import cv2
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    tiles = list(DEFAULT_TILES) if not args.topics else [(t, t) for t in args.topics[:4]]
    rclpy.init(args=ros_args)
    node = rclpy.create_node("quest_operator_view")
    frames, stamps = {}, {}
    status = {"state": None, "state_t": 0.0, "pick": None, "pick_t": 0.0}

    def on_image(topic):
        def callback(msg):
            im = image_to_bgr(msg)
            if im is not None:
                frames[topic], stamps[topic] = im, time.monotonic()
        return callback

    for _, topic in tiles + list(EXTRA_TILES.values()):
        node.create_subscription(Image, topic, on_image(topic), qos_profile_sensor_data)

    def on_json(key):
        def callback(msg):
            try:
                status[key], status[key + "_t"] = json.loads(msg.data), time.monotonic()
            except ValueError:
                pass
        return callback
    node.create_subscription(String, args.state_topic, on_json("state"), 10)
    node.create_subscription(String, args.pick_topic, on_json("pick"), 10)

    window = "Quest teleop - operator view"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    extra = None  # key of the extra tile shown in the free fourth slot
    opened = time.monotonic()
    runner, runner_read = None, 0.0
    try:
        while rclpy.ok():
            t0 = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            for _ in range(20):  # drain queued callbacks without blocking the window
                rclpy.spin_once(node, timeout_sec=0.0)
            shown = (list(tiles) + [(None, None)] * 4)[:4]
            if extra is not None:
                shown[3] = EXTRA_TILES[extra]
            now = time.monotonic()
            if now - runner_read > 0.25:
                runner, runner_read = read_runner_status(args.runner_status), now
            canvas = compose(shown, frames, stamps, status_lines(status["state"], status["pick"], now,
                                                                 status["state_t"], status["pick_t"], runner), now)
            if runner is not None and runner.get("phase") in READY_PHASES:
                overlay(canvas, runner.get("banner") or runner.get("text", ""))
            elif status["pick"] is not None and now - status["pick_t"] < 1.0:
                depth_gauge(canvas, status["pick"])
            if args.scale != 1.0:
                canvas = cv2.resize(canvas, None, fx=args.scale, fy=args.scale)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(max(1, int(1000 / args.hz - (time.monotonic() - t0) * 1000))) & 0xFF
            # Closed by its title bar: VISIBLE is 0. OpenCV 4.5 GTK builds report -1 (not supported) all the time and a
            # GTK window reads 0 until it is mapped, so -1 never counts and 0 only after 2 s; q always quits.
            closed = time.monotonic() - opened > 2.0 and cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0
            if key == ord("q") or closed:
                break
            if key in EXTRA_TILES:
                extra = None if extra == key else key
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
