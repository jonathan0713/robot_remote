"""Per-arm Quest -> joint command math (numpy only, no ROS): clutch latch, hand -> tool mapping, Cartesian rate
limit, anti-windup, IK and joint rate limit. Same behaviour as the ReBeL dual-arm loop in quest_dual_arm_teleop.py
(run_session), for any SerialChain.

Frames: the hand pose comes in the operator frame (X forward, Y left, Z up) and is turned into the arm base frame
by a rotation about Z (yaw_deg; 0 = the operator faces the way the arm reaches).
"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from kinematics import SerialChain, step_toward


def tool_from_controller(r_tool: Rotation, twist_axis_ctrl, right_ctrl, right_base, tool_axis=(0.0, 0.0, 1.0)) -> Rotation:
    """Fixed-at-engage rotation M mapping controller-local axes onto tool-local axes:
       controller twist axis (Quest handle long axis)      -> tool_axis (outward tool axis; the ReBeL flange
                                                              axis is tool +Z): twist hand = spin tool
       'operator right' seen in the controller frame (at engage, orthogonalised)
                                                            -> tool axis pointing to base 'right'
                                                                                    : nod hand  = tilt tool
    Using the operator-right direction measured at engage (instead of a fixed controller +X) keeps the
    nod sense right for both hands, whatever the left/right OpenXR grip-frame conventions are."""
    def basis(a3, a1):
        e3 = np.asarray(a3, float) / np.linalg.norm(a3)
        e1 = np.asarray(a1, float) - np.dot(a1, e3) * e3
        if np.linalg.norm(e1) < 1e-3:  # degenerate: hand pointing sideways - fall back to any perpendicular
            e1 = np.cross(e3, [0.0, 0.0, 1.0]) if abs(e3[2]) < 0.9 else np.cross(e3, [1.0, 0.0, 0.0])
        e1 /= np.linalg.norm(e1)
        return np.column_stack([e1, np.cross(e3, e1), e3])

    e_basis = basis(twist_axis_ctrl, right_ctrl)
    f_basis = basis(tool_axis, r_tool.inv().apply(right_base))
    return Rotation.from_matrix(f_basis @ e_basis.T)


def scaled(r: Rotation, k: float) -> Rotation:
    """The same axis, k times the angle."""
    return r if k == 1.0 else Rotation.from_rotvec(k * r.as_rotvec())


@dataclass
class TeleopLimits:
    rate: float = 30.0            # control loop [Hz]
    max_joint_vel: float = 0.785  # [rad/s], uniform scaling of the joint step
    max_lin_vel: float = 0.5      # TCP command speed [m/s]
    max_rot_vel_deg: float = 90.0
    max_lead_m: float = 0.05      # anti-windup: command lead over FK(q_cmd)
    max_lead_deg: float = 15.0
    max_reach: float = 0.55       # target clamp around the shoulder [m]
    scale: float = 1.0            # hand -> TCP translation scale
    rot_scale: float = 1.0        # hand -> tool rotation scale (fraction of the hand's rotation angle)


class ArmTeleop:
    """Clutch + IK state of one arm. engage() latches on grip, step() returns the next joint command."""

    def __init__(self, chain: SerialChain, limits: TeleopLimits, *, shoulder=None, yaw_deg=0.0,
                 ctrl_twist_axis=(0.0, -0.707, -0.707), tool_axis=(0.0, 0.0, 1.0), rot_weight=1.0,
                 q_rest=None, posture_gain=0.0):
        self.chain, self.lim = chain, limits
        self.shoulder = chain.joint_origin(0) if shoulder is None else np.asarray(shoulder, float)
        self.r_base_op = Rotation.from_euler("z", yaw_deg, degrees=True)
        self.right_base = self.r_base_op.apply([0.0, -1.0, 0.0])  # operator 'right' in the arm base frame
        self.ctrl_twist_axis, self.tool_axis = ctrl_twist_axis, tool_axis
        self.rot_weight, self.q_rest, self.posture_gain = rot_weight, q_rest, posture_gain
        self.q_cmd = None
        self.engaged = False
        self.rot_frame = "tool"

    def hand_in_base(self, pos, quat):
        p_b = self.r_base_op.apply(pos)
        r_b = self.r_base_op * Rotation.from_quat(quat) * self.r_base_op.inv()
        return p_b, r_b

    def engage(self, q_start, pos, quat, rot_frame="tool"):
        """Grip pressed: the TCP at q_start becomes the reference, the hand pose the origin."""
        self.q_cmd = np.asarray(q_start, float).copy()
        t_now = self.chain.fk(self.q_cmd)
        self.home_p, self.home_r = t_now[:3, 3].copy(), Rotation.from_matrix(t_now[:3, :3])
        self.origin_p, self.origin_r = self.hand_in_base(pos, quat)
        self.origin_q = Rotation.from_quat(quat)  # controller orientation at engage (operator frame)
        self.rot_frame = rot_frame
        self.m_tool_ctrl = tool_from_controller(self.home_r, self.ctrl_twist_axis,
                                                self.origin_q.inv().apply([0.0, -1.0, 0.0]), self.right_base,
                                                self.tool_axis)
        self.cmd_p, self.cmd_r = self.home_p.copy(), self.home_r  # rate-limited Cartesian command
        # never clamp tighter than where the arm already is, or engaging would jump it inward
        self.reach = max(self.lim.max_reach, float(np.linalg.norm(self.home_p - self.shoulder)))
        self.engaged = True

    def release(self):
        self.engaged = False

    def target(self, pos, quat):
        """Desired TCP pose (p_des, r_des) for the hand pose, before rate limiting."""
        p_b, r_b = self.hand_in_base(pos, quat)
        p_des = self.home_p + self.lim.scale * (p_b - self.origin_p)
        d = p_des - self.shoulder
        if np.linalg.norm(d) > self.reach:
            p_des = self.shoulder + d * (self.reach / np.linalg.norm(d))
        if self.rot_frame == "tool":
            # rotation of the controller about its OWN axes since engage, re-expressed on the tool's own axes and
            # applied on the tool side (right-multiplied)
            d_local = scaled(self.origin_q.inv() * Rotation.from_quat(quat), self.lim.rot_scale)
            r_des = self.home_r * (self.m_tool_ctrl * d_local * self.m_tool_ctrl.inv())
        else:
            r_des = scaled(r_b * self.origin_r.inv(), self.lim.rot_scale) * self.home_r
        return p_des, r_des

    def step(self, pos, quat):
        """Next joint command (engaged arms only): rate-limited Cartesian command, anti-windup, IK, joint limit."""
        lim, dt = self.lim, 1.0 / self.lim.rate
        p_des, r_des = self.target(pos, quat)
        dp = p_des - self.cmd_p
        n = np.linalg.norm(dp)
        if n > lim.max_lin_vel * dt:
            dp *= lim.max_lin_vel * dt / n
        self.cmd_p = self.cmd_p + dp
        dr = (r_des * self.cmd_r.inv()).as_rotvec()
        a = np.linalg.norm(dr)
        w_max = np.deg2rad(lim.max_rot_vel_deg) * dt
        if a > w_max:
            dr *= w_max / a
        self.cmd_r = Rotation.from_rotvec(dr) * self.cmd_r
        # anti-windup: keep the command within max_lead_* of what the commanded joints reach
        t_ach = self.chain.fk(self.q_cmd)
        r_ach = Rotation.from_matrix(t_ach[:3, :3])
        e_r = (self.cmd_r * r_ach.inv()).as_rotvec()
        m = np.linalg.norm(e_r)
        lead_r = np.deg2rad(lim.max_lead_deg)
        if m > lead_r:
            self.cmd_r = Rotation.from_rotvec(e_r * (lead_r / m)) * r_ach
        e_p = self.cmd_p - t_ach[:3, 3]
        m = np.linalg.norm(e_p)
        if m > lim.max_lead_m:
            self.cmd_p = t_ach[:3, 3] + e_p * (lim.max_lead_m / m)
        t_des = np.eye(4)
        t_des[:3, 3] = self.cmd_p
        t_des[:3, :3] = self.cmd_r.as_matrix()
        q_ik = self.chain.ik(self.q_cmd, t_des, rot_weight=self.rot_weight, q_rest=self.q_rest,
                             posture_gain=self.posture_gain)
        self.q_cmd = step_toward(self.q_cmd, q_ik, lim.max_joint_vel * dt)
        return self.q_cmd
