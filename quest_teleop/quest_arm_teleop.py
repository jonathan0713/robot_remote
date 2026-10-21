#!/usr/bin/env python3
"""Meta Quest (Isaac Teleop / CloudXR) -> one igus ReBeL arm through ros2_control.

  Quest right controller grip pose --clutch--> EE target (arm base frame)
    --numpy DLS IK on /robot_description--> joint targets
    --> /rebel_arm_trajectory_controller/joint_trajectory   (same controller MoveIt executes on)
  Trigger -> XEG-32 gripper carriages on /dual_arm/isaac_joint_commands (Isaac sim only).

Controls: hold SQUEEZE (grip) to move, release to freeze; TRIGGER closes the gripper.
Re-squeezing re-latches, so you can reposition your hand while released.

Needs the ros2_control stack running (dual_arm_sim/start_right_arm_moveit.sh or
`ros2 launch igus_rebel rebel.launch.py hardware_protocol:=isaac_topic` + arm_relay.py).

  source /opt/ros/humble/setup.bash && source robot_remote/igus_ws/install/setup.bash
  robot_remote/.venv_quest/bin/python quest_arm_teleop.py --side right

(placo is not used: its bundled pinocchio clashes with /opt/ros/humble libs and aborts the process.)
"""

import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from kinematics import SerialChain, _origin, step_toward  # noqa: F401  (re-exported)
from quest_input import OPERATOR_T_ANCHOR, QuestController  # noqa: F401  (re-exported)

HERE = Path(__file__).resolve().parent
ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
# Natural working pose: tool tip ~0.42 m in front of / 0.21 m above base_link, gripper pitched 30 deg
# forward from straight down (like a hand reaching for a table), 0.44 m from the shoulder (~0.2 m reach
# margin). joint5 = 50 deg leaves ~44 deg of wrist pitch either way before its +-94 deg limit.
HOME_DEG = [0.0, 0.0, 65.0, 0.0, 50.0, 0.0]

# XEG-32 jaw centre in link6, measured from the Isaac USD (dual_arm_rig_v2, q = 0): midpoint of the two
# carriage joint origins (-30.2, -8.1) mm, 6 mm inside the fingertips (tips at z = 101 mm).
# USD link6 == URDF link6 was verified (same pose relative to base_link at q = 0).
GRIPPER_TCP_IN_LINK6 = (-0.0302, -0.0081, 0.095)
# Control-point presets (translation in link6). "eef" = the repo's own end effector xeg32_tool_tip
# (calibrated knife TCP, 94 mm to the side of the flange axis); "flange" = igus_tool0 on the flange axis.
TCP_PRESETS = {
    "eef": (-0.00594, -0.09412, 0.1834),
    "flange": (0.0, 0.0, 0.0527),
    "gripper": GRIPPER_TCP_IN_LINK6,
}

# ----------------------------------------------------------------------------- node
class QuestArmTeleop(Node):
    def __init__(self, args):
        super().__init__(f"quest_{args.side}_arm_teleop")
        self.args = args
        self.q_meas = None
        self.urdf = None
        # operator frame -> arm base frame (rotation about Z); see --yaw-deg help
        self.r_base_op = Rotation.from_euler("z", args.yaw_deg, degrees=True)
        self.traj_pub = self.create_publisher(JointTrajectory, args.controller_topic, 10)
        self.grip_pub = self.create_publisher(JointState, "/dual_arm/isaac_joint_commands", 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", lambda m: setattr(self, "urdf", m.data), qos)

    def _on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM_JOINTS):
            self.q_meas = np.array([d[j] for j in ARM_JOINTS])

    def wait_ready(self):
        while rclpy.ok() and (self.urdf is None or self.q_meas is None):
            self.get_logger().info("waiting for /robot_description and /joint_states ...", throttle_duration_sec=3.0)
            rclpy.spin_once(self, timeout_sec=0.2)

    def send(self, q):
        msg = JointTrajectory(joint_names=ARM_JOINTS)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=0, nanosec=int(self.args.lookahead * 1e9))
        msg.points = [pt]
        self.traj_pub.publish(msg)

    def go_home(self, q_home, vel=0.4, rate=30.0):
        """Stream a straight joint-space move to q_home at <= vel rad/s, then wait to settle."""
        q = self.q_meas.copy()
        steps = max(1, int(np.ceil(np.max(np.abs(q_home - q)) / vel * rate)))
        self.get_logger().info(f"moving to home pose {HOME_DEG} deg ({steps / rate:.1f} s)")
        for k in range(1, steps + 1):
            self.send(q + (q_home - q) * k / steps)
            end = time.perf_counter() + 1.0 / rate
            while time.perf_counter() < end:
                rclpy.spin_once(self, timeout_sec=0.005)
        end = time.perf_counter() + 1.0
        while time.perf_counter() < end:
            self.send(q_home)
            rclpy.spin_once(self, timeout_sec=0.05)

    def send_gripper(self, closedness):
        a = self.args
        pos = a.gripper_open + (a.gripper_closed - a.gripper_open) * closedness
        names = [f"arm_{a.side}_xeg32_left_carriage_joint", f"arm_{a.side}_xeg32_right_carriage_joint"]
        self.grip_pub.publish(JointState(name=names, position=[pos, pos]))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--side", choices=["left", "right"], default="right", help="rig arm (gripper topic names)")
    p.add_argument("--hand", choices=["left", "right"], default="right", help="Quest controller")
    p.add_argument("--yaw-deg", type=float, default=0.0,
                   help="rotation about Z from operator frame (X fwd, Y left, Z up) to arm base frame. "
                        "0 = you face the same way the arm reaches (pushing forward extends the arm); "
                        "180 = you stand facing the robot (mirror-like).")
    p.add_argument("--tcp-preset", choices=sorted(TCP_PRESETS), default="flange",
                   help="control point: flange = igus_tool0 on the flange (default), eef = repo knife tip xeg32_tool_tip, gripper = XEG-32 jaw centre")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--scale", type=float, default=1.0, help="hand->EE translation scale")
    p.add_argument("--rate", type=float, default=30.0)
    p.add_argument("--lookahead", type=float, default=0.1, help="time_from_start of each streamed point [s]")
    p.add_argument("--max-joint-vel", type=float, default=0.6, help="rad/s clamp (ReBeL limit 0.785)")
    p.add_argument("--rot-weight", type=float, default=1.0, help="orientation gain of the secondary IK task (0 = position only)")
    p.add_argument("--clutch", type=float, default=0.5, help="squeeze threshold")
    p.add_argument("--controller-topic", default="/rebel_arm_trajectory_controller/joint_trajectory")
    p.add_argument("--gripper-open", type=float, default=0.00483688, help="carriage pos at trigger 0 [m]")
    p.add_argument("--gripper-closed", type=float, default=-0.0116631, help="carriage pos at trigger 1 [m]")
    p.add_argument("--no-gripper", action="store_true")
    p.add_argument("--quiet", action="store_true", help="no per-0.5 s axis-check log while engaged")
    p.add_argument("--no-home", action="store_true", help="do not move to HOME_DEG at startup")
    p.add_argument("--max-reach", type=float, default=0.55,
                   help="clamp EE target to this distance from the shoulder [m] (ReBeL reach ~0.66) so IK never chases an unreachable point")
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = QuestArmTeleop(args)
    node.wait_ready()
    chain = SerialChain(node.urdf, args.base_frame, "link6", ARM_JOINTS, tcp_xyz=TCP_PRESETS[args.tcp_preset])
    shoulder = chain.joint_origin(1)
    if not args.no_home:
        node.go_home(np.deg2rad(HOME_DEG))
    q_cmd = node.q_meas.copy()
    t_home = chain.fk(q_cmd)
    log = node.get_logger()
    log.info(f"EE ({args.tcp_preset}) at {np.round(t_home[:3, 3], 3)} in {args.base_frame}")

    xr = QuestController(args.hand, HERE / "cloudxr_quest3.env")
    log.info("CloudXR up. Quest Browser -> https://nvidia.github.io/IsaacTeleop/client/v1.3.131/ , "
             "Server IP = this PC, Connect. Hold SQUEEZE to move, TRIGGER = gripper.")

    engaged = False
    origin_p = origin_r = None
    home_p, home_r = t_home[:3, 3].copy(), Rotation.from_matrix(t_home[:3, :3])
    dt = 1.0 / args.rate
    max_dq = args.max_joint_vel * dt
    last_trig = None
    was_tracked = False
    reach = args.max_reach
    try:
        while rclpy.ok():
            t0 = time.perf_counter()
            rclpy.spin_once(node, timeout_sec=0.0)
            tracked, pos, quat, squeeze, trigger = xr.read()
            if tracked != was_tracked:
                log.info("controller TRACKED - hold SQUEEZE to move" if tracked else "controller tracking LOST")
                was_tracked = tracked
            if not tracked:
                engaged = False
                log.info("waiting for controller tracking ...", throttle_duration_sec=3.0)
            else:
                p_b = node.r_base_op.apply(pos)  # controller position in arm base axes
                r_b = node.r_base_op * Rotation.from_quat(quat) * node.r_base_op.inv()
                if squeeze >= args.clutch and not engaged:
                    # latch: EE home = where the arm is now; origin = where the hand is now
                    t_now = chain.fk(node.q_meas)
                    home_p, home_r = t_now[:3, 3].copy(), Rotation.from_matrix(t_now[:3, :3])
                    origin_p, origin_r = p_b.copy(), r_b
                    q_cmd = node.q_meas.copy()
                    reach = max(args.max_reach, float(np.linalg.norm(home_p - shoulder)))  # no jump on engage
                    engaged = True
                    log.info("engaged")
                elif squeeze < args.clutch and engaged:
                    engaged = False
                    log.info("released")
                if engaged:
                    t_des = np.eye(4)
                    p_des = home_p + args.scale * (p_b - origin_p)
                    d = p_des - shoulder
                    if np.linalg.norm(d) > reach:  # stay inside the reachable sphere
                        p_des = shoulder + d * (reach / np.linalg.norm(d))
                    t_des[:3, 3] = p_des
                    t_des[:3, :3] = ((r_b * origin_r.inv()) * home_r).as_matrix()
                    q_ik = chain.ik(q_cmd, t_des, rot_weight=args.rot_weight)
                    q_cmd = step_toward(q_cmd, q_ik, max_dq)
                    node.send(q_cmd)
                    if not args.quiet:
                        # Axis check: hand delta in the operator frame (+x fwd, +y left, +z up) vs. the
                        # EE target / measured delta in the arm base frame. With --yaw-deg 0 all three
                        # should change along the same axis with the same sign.
                        hand_op = node.r_base_op.inv().apply(p_b - origin_p) * 100
                        ee_now = chain.fk(node.q_meas)[:3, 3]
                        log.info(
                            "hand[cm] fwd %+5.1f left %+5.1f up %+5.1f | target dEE[cm] x %+5.1f y %+5.1f z %+5.1f"
                            " | actual dEE[cm] x %+5.1f y %+5.1f z %+5.1f"
                            % (*hand_op, *((p_des - home_p) * 100), *((ee_now - home_p) * 100)),
                            throttle_duration_sec=0.5,
                        )
                if not args.no_gripper and (last_trig is None or abs(trigger - last_trig) > 0.01):
                    node.send_gripper(trigger)
                    last_trig = trigger
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        xr.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
