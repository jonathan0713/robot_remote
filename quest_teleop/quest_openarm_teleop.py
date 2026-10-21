#!/usr/bin/env python3
"""Meta Quest (Isaac Teleop / CloudXR) -> OpenArm arms in the ITRI_project Isaac Sim 5.1 scene.

  controller --clutch--> TCP target (openarm_body_link) --IK (7 joints + posture)--> openarm_<side>_joint1..7
  trigger    --> openarm_<side>_finger_joint1                                   -> /<side>_joint_command

Each driven arm publishes its joints + gripper on its own command topic (robot profile, default
config/robots/openarm_dual_gripper.json) every tick; the chain flow's joint_command_mux.py merges them into
Isaac's /joint_command. Arms that are not driven get no command at all (the left-arm tote teacher keeps them).

Per hand: hold SQUEEZE to move that arm, release to freeze it; TRIGGER closes its gripper. The gripper only follows
the trigger once the trigger has come within --grip-pickup of the current gripper command (no jump on start).

Held-tote chain (--run-dir <run_dir> of run_tote_chain_flow.sh with TOTE_CHAIN_ASSIST=1 TOTE_CHAIN_RECORD=1):
  A or X       start / end the pick capture (one byte on <run_dir>/capture.fifo each)
  B            discard: end the capture, write <run_dir>/teleop_discard.json, end the attempt
  Y            end the attempt: end the capture, touch <run_dir>/release (the chain pushes the tote back)
Without --run-dir the buttons are only published on /quest_teleop/events ("toggle", "discard", "end").

Run with the ROS environment of the chain flow (ROS_DOMAIN_ID 42, rmw_fastrtps_cpp + its FastDDS profile; NOT
robot_remote's ros_env.sh, which sets ROS_LOCALHOST_ONLY=1):
  source /opt/ros/humble/setup.bash
  .venv_quest/bin/python quest_teleop/quest_openarm_teleop.py --arms right --run-dir <run_dir>
"""

import argparse
import json
import os
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from arm_collision import clearance, link_capsules
from quest_input import QuestController
from robot_profile import load_profile
from teleop_core import ArmTeleop, TeleopLimits

HERE = Path(__file__).resolve().parent
SIDES = ("left", "right")
LATEST = QoSProfile(depth=1)


class CaptureControl:
    """The chain's capture control FIFO: one byte starts the pick capture, the next ends it. Opened on first use and
    kept open; closing it ends an open capture on the recorder side."""

    def __init__(self, fifo: Path | None):
        self.fifo, self.fd, self.capturing = fifo, None, False

    def toggle(self) -> bool:
        if self.fifo is None:
            return False
        if self.fd is None:
            self.fd = os.open(self.fifo, os.O_WRONLY | os.O_NONBLOCK)  # ENXIO: no reader (chain not ready)
        if os.write(self.fd, b" ") != 1:
            raise RuntimeError("capture control write failed")
        self.capturing = not self.capturing
        return True

    def end(self):
        if self.capturing:
            self.toggle()

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class OpenArmTeleop(Node):
    def __init__(self, args, profile):
        super().__init__("quest_openarm_teleop")
        self.args, self.profile = args, profile
        lim = profile.limits
        limits = TeleopLimits(rate=args.rate, max_joint_vel=args.max_joint_vel or lim["max_joint_vel"],
                              max_lin_vel=args.max_lin_vel or lim["max_lin_vel"],
                              max_rot_vel_deg=args.max_rot_vel or lim["max_rot_vel_deg"],
                              max_lead_m=lim["max_lead_m"], max_lead_deg=lim["max_lead_deg"],
                              max_reach=lim["max_reach"], scale=args.scale)
        gain = profile.ik["posture_gain"] if args.posture_gain is None else args.posture_gain
        self.sides = {s: profile.sides[s] for s in SIDES}
        self.arms = {s: ArmTeleop(profile.sides[s].chain, limits, yaw_deg=args.yaw_deg,
                                  ctrl_twist_axis=args.ctrl_twist_axis, tool_axis=profile.sides[s].tool_out_axis,
                                  rot_weight=profile.ik["rot_weight"], q_rest=profile.sides[s].rest, posture_gain=gain)
                     for s in args.arms}
        self.q_meas = {s: None for s in SIDES}   # measured joints of both arms (the guard needs the undriven one)
        self.grip_cmd = {s: None for s in args.arms}
        self.grip_meas = {}                             # measured gripper joint of the driven arms
        self.grip_live = {s: False for s in args.arms}  # gripper follows the trigger (after pickup)
        self.seed = {}                                  # last /joint_command values (initial command)
        self.cmd_pub = {s: self.create_publisher(JointState, profile.sides[s].command_topic, 10) for s in args.arms}
        self.event_pub = self.create_publisher(String, "/quest_teleop/events", 10)
        self.state_pub = self.create_publisher(String, "/quest_teleop/state", 10)
        self.create_subscription(JointState, profile.state_topic, self._on_state, LATEST)
        if args.seed_topic:
            self.create_subscription(JointState, args.seed_topic, self._on_seed, LATEST)

    def _on_state(self, msg):
        d = dict(zip(msg.name, msg.position))
        for s, side in self.sides.items():
            if all(j in d for j in side.joints):
                self.q_meas[s] = np.array([d[j] for j in side.joints])
            if s in self.grip_cmd and side.gripper_joints[0] in d:
                self.grip_meas[s] = float(d[side.gripper_joints[0]])

    def _on_seed(self, msg):
        self.seed.update(zip(msg.name, msg.position))

    def wait_ready(self, seed_wait_s=1.0):
        log = self.get_logger()
        while rclpy.ok() and any(self.q_meas[s] is None for s in self.arms):
            log.info(f"waiting for {self.profile.state_topic} (is the Isaac scene running?)", throttle_duration_sec=3.0)
            rclpy.spin_once(self, timeout_sec=0.2)
        end = time.monotonic() + (seed_wait_s if self.args.seed_topic else 0.0)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        for s, arm in self.arms.items():
            side = self.sides[s]
            # Start from the joint TARGETS Isaac currently holds (the chain's last command), not the measured
            # joints: commanding the measured pose would let the arm sag by its tracking error.
            if all(j in self.seed for j in side.joints):
                arm.q_cmd = np.array([self.seed[j] for j in side.joints])
                src = self.args.seed_topic
            else:
                arm.q_cmd = self.q_meas[s].copy()
                src = "measured joints"
            g = side.gripper_joints[0]
            self.grip_cmd[s] = float(self.seed.get(g, self.grip_meas.get(s, side.gripper_open)))
            log.info(f"[{s}] start command from {src}: joints {np.round(arm.q_cmd, 3).tolist()}, "
                     f"gripper {self.grip_cmd[s]:.4f} m")

    def send(self, side):
        sd, arm = self.sides[side], self.arms[side]
        names = list(sd.joints) + list(sd.gripper_joints)
        pos = [float(v) for v in arm.q_cmd] + [float(self.grip_cmd[side])] * len(sd.gripper_joints)
        msg = JointState(name=names, position=pos)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub[side].publish(msg)

    def update_gripper(self, side, trigger):
        """Soft pickup: hold the current command until the trigger reaches it, then follow the trigger."""
        sd = self.sides[side]
        want = sd.gripper_position(trigger)
        if self.args.no_gripper:
            return
        if not self.grip_live[side]:
            span = abs(sd.gripper_open - sd.gripper_closed)
            if abs(want - self.grip_cmd[side]) <= self.args.grip_pickup * span:
                self.grip_live[side] = True
                self.get_logger().info(f"[{side}] gripper follows the trigger")
            return
        self.grip_cmd[side] = want

    def guard(self, cand):
        """Inter-arm capsule guard: refuse steps that bring the arms closer than --min-arm-dist while reducing the
        clearance (moving apart is always allowed). The undriven arm is at its measured pose."""
        a = self.args
        def caps(s, q):
            return link_capsules(self.sides[s].chain, q, a.link_radius, a.tool_radius)
        now = {s: (self.arms[s].q_cmd if s in self.arms else self.q_meas[s]) for s in SIDES}
        if any(q is None for q in now.values()):
            return cand
        nxt = {s: cand.get(s, now[s]) for s in SIDES}
        d_now = clearance(caps("left", now["left"]), caps("right", now["right"]))
        d_next = clearance(caps("left", nxt["left"]), caps("right", nxt["right"]))
        if d_next < a.min_arm_dist and d_next < d_now:
            self.get_logger().warning(f"arms {d_next * 100:.1f} cm apart (capsule surfaces) - holding",
                                      throttle_duration_sec=1.0)
            return {}
        return cand


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--profile", default="openarm_dual_gripper", help="config/robots/<name>.json or a path")
    p.add_argument("--arms", nargs="+", choices=SIDES, default=["right"], help="arms to drive (default: right)")
    p.add_argument("--swap-hands", action="store_true", help="left controller drives the right arm and vice versa")
    p.add_argument("--yaw-deg", type=float, default=0.0,
                   help="rotation about Z from the operator frame (X fwd, Y left, Z up) to openarm_body_link; "
                        "0 = you face the way the robot faces (pushing forward reaches into the shelf)")
    p.add_argument("--scale", type=float, default=1.0, help="hand -> TCP translation scale")
    p.add_argument("--rate", type=float, default=30.0)
    p.add_argument("--max-joint-vel", type=float, help="rad/s (default: profile)")
    p.add_argument("--max-lin-vel", type=float, help="TCP speed [m/s] (default: profile)")
    p.add_argument("--max-rot-vel", type=float, help="TCP rotation speed [deg/s] (default: profile)")
    p.add_argument("--posture-gain", type=float, help="per-tick pull of the elbow toward the rest pose (default: profile)")
    p.add_argument("--rot-frame", choices=["tool", "base"], default="tool")
    p.add_argument("--ctrl-twist-axis", type=float, nargs=3, default=(0.0, -0.707, -0.707), metavar=("X", "Y", "Z"),
                   help="Quest handle long axis in the controller (OpenXR grip) frame")
    p.add_argument("--clutch", type=float, default=0.5, help="squeeze threshold")
    p.add_argument("--grip-pickup", type=float, default=0.1,
                   help="the gripper follows the trigger once it is this fraction of the stroke from the command")
    p.add_argument("--no-gripper", action="store_true", help="never change the gripper command")
    p.add_argument("--min-arm-dist", type=float, default=0.03, help="inter-arm guard: capsule surface clearance [m]")
    p.add_argument("--link-radius", type=float, default=0.05)
    p.add_argument("--tool-radius", type=float, default=0.045)
    p.add_argument("--seed-topic", default="/joint_command",
                   help="start from the targets last published here ('' = from the measured joints)")
    p.add_argument("--run-dir", type=Path, help="held-tote chain run dir: capture.fifo, release, teleop_discard.json")
    p.add_argument("--max-run-s", type=float, default=0.0, help="end the attempt after this long (0 = no limit)")
    args, ros_args = p.parse_known_args(argv)
    if len(set(args.arms)) != len(args.arms):
        p.error("--arms must not contain duplicates")
    for key in ("rate", "scale", "min_arm_dist", "link_radius", "tool_radius"):
        if not np.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            p.error(f"--{key.replace('_', '-')} must be finite and positive")
    return args, ros_args


def end_attempt(node, capture, args, reason, discard=False):
    """End the open capture, mark a discard, release the tote (chain flow)."""
    log = node.get_logger()
    capture.end()
    capture.close()
    if args.run_dir is not None:
        if discard:
            (args.run_dir / "teleop_discard.json").write_text(
                json.dumps({"reason": reason, "unix_s": time.time()}, indent=2) + "\n")
        (args.run_dir / "release").touch()
    log.warning(f"attempt ended ({reason})" + (" - discarded" if discard else ""))


def run(node, args, xr):
    log = node.get_logger()
    hand_of = {s: ({"left": "right", "right": "left"}[s] if args.swap_hands else s) for s in node.arms}
    capture = CaptureControl(args.run_dir / "capture.fifo" if args.run_dir else None)
    dt = 1.0 / args.rate
    last_buttons, tracked = {}, {s: False for s in node.arms}
    stop = {"requested": False}
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__("requested", True))
    t_start = time.monotonic()
    ended = None
    try:
        while rclpy.ok() and not stop["requested"] and ended is None:
            t0 = time.perf_counter()
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=0.0)
            readings = xr.read_all()
            cand = {}
            for side, arm in node.arms.items():
                ok, pos, quat, squeeze, trigger = readings[hand_of[side]]
                if ok != tracked[side]:
                    log.info(f"[{side} arm <- {hand_of[side]} hand] " + ("TRACKED" if ok else "tracking LOST"))
                    tracked[side] = ok
                if not ok:
                    arm.release()
                    continue
                if squeeze >= args.clutch and not arm.engaged:
                    arm.engage(arm.q_cmd, pos, quat, args.rot_frame)
                    log.info(f"[{side}] engaged")
                elif squeeze < args.clutch and arm.engaged:
                    arm.release()
                    log.info(f"[{side}] released")
                if arm.engaged:
                    q_prev = arm.q_cmd.copy()
                    cand[side] = arm.step(pos, quat)
                    arm.q_cmd = q_prev  # committed below, after the guard
                node.update_gripper(side, trigger)
            for side, q in node.guard(cand).items():
                node.arms[side].q_cmd = q
            for side in node.arms:
                node.send(side)
            node.state_pub.publish(String(data=json.dumps(
                {s: ("engaged" if a.engaged else "tracked" if tracked[s] else "lost") for s, a in node.arms.items()}
                | {"capturing": capture.capturing})))
            # buttons (rising edges): A/X capture toggle, B discard, Y end the attempt
            for h, (prim, sec) in xr.buttons.items():
                was_p, was_s = last_buttons.get(h, (False, False))
                if prim and not was_p:
                    try:
                        sent = capture.toggle()
                        log.warning(f"[{h} hand] capture {'START' if capture.capturing else 'END'}" if sent
                                    else f"[{h} hand] toggle (no --run-dir)")
                    except OSError as exc:
                        log.error(f"capture control not available ({exc}) - is the chain waiting for the pick?")
                    node.event_pub.publish(String(data="toggle"))
                if sec and not was_s:
                    ev = "end" if h == "left" else "discard"
                    node.event_pub.publish(String(data=ev))
                    if args.run_dir is not None:
                        ended = ev
                    log.warning(f"[{h} hand] {ev}")
                last_buttons[h] = (prim, sec)
            if args.max_run_s and time.monotonic() - t_start > args.max_run_s and ended is None:
                ended = "timeout"
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
    finally:
        if args.run_dir is not None:
            reason = ended or "stopped"
            end_attempt(node, capture, args, reason, discard=reason in ("discard", "stopped", "timeout"))
        else:
            capture.close()


def main(argv=None):
    args, ros_args = parse_args(argv)
    profile = load_profile(args.profile)
    rclpy.init(args=ros_args)
    signal.signal(signal.SIGTERM, signal.default_int_handler)  # abort during start-up
    node = OpenArmTeleop(args, profile)
    log = node.get_logger()
    xr = None
    try:
        node.wait_ready()
        # both hands always: the buttons (X/Y on the left, A/B on the right) are needed with one arm too
        xr = QuestController(list(SIDES), HERE / "cloudxr_quest3.env", app_name=profile.app_name)
        log.info("CloudXR up. Quest Browser -> https://nvidia.github.io/IsaacTeleop/client/v1.3.131/ , Server IP = "
                 "this PC, Connect. Hold SQUEEZE to move, TRIGGER = gripper, A/X = capture, B = discard, Y = end.")
        run(node, args, xr)
    finally:
        if xr is not None:
            xr.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
