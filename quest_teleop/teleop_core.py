"""Per-arm Quest -> joint command math (numpy only, no ROS): clutch latch, hand -> tool mapping, Cartesian rate
limit, anti-windup, IK and joint rate limit. Same behaviour as the ReBeL dual-arm loop in quest_dual_arm_teleop.py
(run_session), for any SerialChain.

Frames: the hand pose comes in the operator frame (X forward, Y left, Z up) and is turned into the arm base frame
by a rotation about Z (yaw_deg; 0 = the operator faces the way the arm reaches).
"""

import time
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
    e_basis = axes_basis(twist_axis_ctrl, right_ctrl)
    f_basis = axes_basis(tool_axis, r_tool.inv().apply(right_base))
    return Rotation.from_matrix(f_basis @ e_basis.T)


def axes_basis(a3, a1) -> np.ndarray:
    """Columns (e1, e2, e3): e3 along a3, e1 = a1 orthogonalised against it, e2 = e3 x e1. For the controller:
    e3 = the handle's twist axis, e1 = 'operator right' at engage (nod = rotation about e1), e2 = the hand's side
    swing axis (side = rotation about e2)."""
    e3 = np.asarray(a3, float) / np.linalg.norm(a3)
    e1 = np.asarray(a1, float) - np.dot(a1, e3) * e3
    if np.linalg.norm(e1) < 1e-3:  # degenerate: hand pointing sideways - fall back to any perpendicular
        e1 = np.cross(e3, [0.0, 0.0, 1.0]) if abs(e3[2]) < 0.9 else np.cross(e3, [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    return np.column_stack([e1, np.cross(e3, e1), e3])


def scaled(r: Rotation, k: float) -> Rotation:
    """The same axis, k times the angle."""
    return r if k == 1.0 else Rotation.from_rotvec(k * r.as_rotvec())


def scaled_axes(r: Rotation, basis: np.ndarray, twist: float, nod: float, side: float) -> Rotation:
    """Each component of r's rotation vector along the controller's axes (basis columns: nod, side, twist axis)
    scaled on its own. A held controller turns much further about some axes than others (forearm twist about
    +-80 deg, wrist nod about +-60, side swing about +-25), so one scale for all of them makes the side tilt feel
    stuck. Equal scales give scaled(r, k)."""
    v = basis.T @ r.as_rotvec()
    return Rotation.from_rotvec(basis @ (v * np.array([nod, side, twist])))


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
    rot_scale_axes: tuple | None = None  # (twist, nod, side) per controller axis, rot_frame tool; None = rot_scale


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
        right_ctrl = self.origin_q.inv().apply([0.0, -1.0, 0.0])
        self.m_tool_ctrl = tool_from_controller(self.home_r, self.ctrl_twist_axis, right_ctrl, self.right_base,
                                                self.tool_axis)
        self.ctrl_basis = axes_basis(self.ctrl_twist_axis, right_ctrl)
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
            d_local = self.origin_q.inv() * Rotation.from_quat(quat)
            d_local = (scaled_axes(d_local, self.ctrl_basis, *self.lim.rot_scale_axes) if self.lim.rot_scale_axes
                       else scaled(d_local, self.lim.rot_scale))
            r_des = self.home_r * (self.m_tool_ctrl * d_local * self.m_tool_ctrl.inv())
        else:
            r_des = scaled(r_b * self.origin_r.inv(), self.lim.rot_scale) * self.home_r
        return p_des, r_des

    def step(self, pos, quat):
        """Next joint command (engaged arms only): rate-limited Cartesian command, anti-windup, IK, joint limit."""
        lim, dt = self.lim, 1.0 / self.lim.rate
        p_des, r_des = self.target(pos, quat)
        self.last_target = (p_des, r_des)  # for the teleop trace: what the hand asked for, before any limit
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


class TeleopTrace:
    """Per tick and driven arm: the hand pose, what it asked for (target, before any limit), the rate-limited
    command, the joint command and the measured joints, engaged / tracked / capturing. Saved as
    <run_dir>/teleop_trace.npz to see afterwards where the tool did not follow the hand (rotation lag per axis,
    joints on a limit). Nothing here is used by the recorder."""

    def __init__(self, t0: float):
        self.t0, self.rows = t0, {}

    def add(self, node, tracked: dict, capturing: bool):
        nan = lambda n: np.full(n, np.nan)
        for side, arm in node.arms.items():
            hand = node.last_hand.get(side)
            engaged = bool(arm.engaged)
            target = getattr(arm, "last_target", None) if engaged else None
            q_meas = node.q_meas.get(side)
            row = np.concatenate([
                [time.monotonic() - self.t0, time.time(), engaged, bool(tracked.get(side)), bool(capturing)],
                (np.concatenate([hand[0], hand[1]]) if hand is not None else nan(7)),
                (np.concatenate([target[0], target[1].as_quat()]) if target is not None else nan(7)),
                (np.concatenate([arm.cmd_p, arm.cmd_r.as_quat()]) if engaged else nan(7)),
                (arm.q_cmd if arm.q_cmd is not None else nan(7)),
                (q_meas if q_meas is not None else nan(7))])
            self.rows.setdefault(side, []).append(row)

    COLUMNS = (["t_s", "unix_s", "engaged", "tracked", "capturing"] + [f"hand_{k}" for k in "xyz"]
               + [f"hand_q{k}" for k in "xyzw"] + [f"target_{k}" for k in "xyz"] + [f"target_q{k}" for k in "xyzw"]
               + [f"cmd_{k}" for k in "xyz"] + [f"cmd_q{k}" for k in "xyzw"]
               + [f"q_cmd{i}" for i in range(1, 8)] + [f"q_meas{i}" for i in range(1, 8)])

    def save(self, path, log=None):
        try:
            np.savez_compressed(path, columns=np.array(self.COLUMNS),
                                **{side: np.asarray(rows, dtype=float) for side, rows in self.rows.items()})
        except OSError as exc:  # never let the trace stop the attempt's end
            if log is not None:
                log.error(f"teleop trace not saved: {exc}")
