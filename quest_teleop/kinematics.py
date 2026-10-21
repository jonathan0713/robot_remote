"""Serial-chain FK + task-priority DLS IK from a URDF, for any number of revolute joints (numpy only, no ROS).

Used by the ReBeL teleops (6 joints) and the OpenArm teleop (7 joints). Revolute / continuous joints that are not
in `joint_names` are treated as fixed at 0: the OpenArm URDF exported from Isaac models its hand and TCP mounts as
zero-range revolute joints (`joints_openarm_<side>_hand`, `openarm_<side>_ee_tcp_joint`).
"""

import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def _origin(el) -> np.ndarray:
    t = np.eye(4)
    o = el.find("origin")
    if o is not None:
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
        t[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        t[:3, 3] = xyz
    return t


def step_toward(q_from, q_to, max_dq):
    """Move toward q_to by at most max_dq on the largest joint, scaling ALL joints uniformly.
    Per-joint clipping bends the joint-space direction and made the TCP swing up to 15 cm
    during fast wrist turns; uniform scaling keeps the IK's direction."""
    dq = q_to - q_from
    m = float(np.max(np.abs(dq)))
    return q_from + (dq * (max_dq / m) if m > max_dq else dq)


class SerialChain:
    """Revolute/fixed chain base -> tip parsed from a URDF string, with FK + DLS IK."""

    def __init__(self, urdf: str, base: str, tip: str, joint_names: list[str], tcp_xyz=None):
        """tcp_xyz: optional translation [m] appended after `tip` (expressed in the tip frame)."""
        root = ET.fromstring(urdf)
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"no path from {base} to {tip} in robot_description")
            j = by_child[link]
            chain.append(j)
            link = j.find("parent").get("link")
        chain.reverse()
        self.n = len(joint_names)
        self.joint_names = list(joint_names)
        self.elems = []  # (T_origin, axis or None, joint index)
        self.lower, self.upper = np.full(self.n, -np.pi), np.full(self.n, np.pi)
        found = set()
        for j in chain:
            name, typ = j.get("name"), j.get("type")
            if typ in ("revolute", "continuous") and name in joint_names:
                idx = joint_names.index(name)
                found.add(name)
                axis = np.array([float(v) for v in (j.find("axis").get("xyz") if j.find("axis") is not None else "1 0 0").split()])
                lim = j.find("limit")
                if typ == "revolute" and lim is not None:
                    self.lower[idx], self.upper[idx] = float(lim.get("lower")), float(lim.get("upper"))
                self.elems.append((_origin(j), axis / np.linalg.norm(axis), idx))
            elif typ in ("fixed", "revolute", "continuous"):  # revolute not driven here: fixed at 0
                self.elems.append((_origin(j), None, None))
            else:
                raise ValueError(f"unsupported joint type {typ} ({name})")
        missing = [n for n in joint_names if n not in found]
        if missing:
            raise ValueError(f"joints {missing} are not on the chain {base} -> {tip}")
        if tcp_xyz is not None:
            t_tcp = np.eye(4)
            t_tcp[:3, 3] = tcp_xyz
            self.elems.append((t_tcp, None, None))

    def joint_origin(self, n: int) -> np.ndarray:
        """Base-frame position of the n-th (0-based) revolute joint's origin at q = 0. Pose-independent for n = 0;
        for n = 1 only if joint 1's axis passes through it (true on the ReBeL, not on OpenArm)."""
        t, k = np.eye(4), 0
        for t_o, axis, _ in self.elems:
            t = t @ t_o
            if axis is not None:
                if k == n:
                    return t[:3, 3].copy()
                k += 1
        raise IndexError(n)

    def fk(self, q: np.ndarray, with_jacobian: bool = False):
        t = np.eye(4)
        axes, points = [], []
        for t_o, axis, idx in self.elems:
            t = t @ t_o
            if axis is not None:
                axes.append((idx, t[:3, :3] @ axis))
                points.append(t[:3, 3].copy())
                r = np.eye(4)
                r[:3, :3] = Rotation.from_rotvec(axis * q[idx]).as_matrix()
                t = t @ r
        if not with_jacobian:
            return t
        jac = np.zeros((6, self.n))
        for (idx, z), p in zip(axes, points):
            jac[:3, idx] = np.cross(z, t[:3, 3] - p)
            jac[3:, idx] = z
        return t, jac

    def ik(self, q0, t_des, iters=30, damping=0.02, rot_weight=1.0, tol=1e-4, q_rest=None, posture_gain=0.0):
        """Task-priority DLS IK: position is the primary task, orientation is solved in the
        null space of the position task. When the wrist cannot realise an orientation (e.g. a
        joint at its limit) the position is kept and only the orientation is left short.
        Joints sitting on a limit and pushed further into it are frozen for that iteration.

        Redundant arms (n > 6): with q_rest and posture_gain > 0 the FIRST iteration adds a third task that moves
        the joints a fraction posture_gain of the way toward q_rest inside the null space of both pose tasks; the
        remaining iterations then remove the small pose error that the damped null space lets through. The elbow's
        self-motion therefore does not drift with the seed. Called once per teleop tick, the gain is a per-tick rate
        (0.05 at 30 Hz: ~0.7 s time constant). With n <= 6 the null space of the full pose task is empty except at
        singular poses, where the task may move the joints along the singular self-motion (the pose is kept); the
        ReBeL leaves it off."""
        q = q0.copy()
        lam2 = damping**2
        n = self.n
        posture = q_rest is not None and posture_gain > 0.0
        for it in range(iters):
            first = posture and it == 0
            t, jac = self.fk(q, with_jacobian=True)
            e_p = t_des[:3, 3] - t[:3, 3]
            e_r = rot_weight * (Rotation.from_matrix(t_des[:3, :3]) * Rotation.from_matrix(t[:3, :3]).inv()).as_rotvec()
            if not first and np.linalg.norm(e_p) < tol and np.linalg.norm(e_r) < tol:
                break
            free = np.ones(n, dtype=bool)
            for _pass in range(2):
                jp, jr = jac[:3] * free, jac[3:] * free
                jp_pinv = jp.T @ np.linalg.inv(jp @ jp.T + lam2 * np.eye(3))
                dq1 = jp_pinv @ e_p
                null = np.eye(n) - jp_pinv @ jp
                jr_n = jr @ null
                jr_n_pinv = jr_n.T @ np.linalg.inv(jr_n @ jr_n.T + lam2 * np.eye(3))
                dq2 = null @ jr_n_pinv @ (e_r - jr @ dq1)
                dq = dq1 + dq2
                if first:
                    # null space of the stacked (position, then orientation-in-position-null-space) tasks
                    null2 = null @ (np.eye(n) - jr_n_pinv @ jr_n)
                    dq = dq + null2 @ (posture_gain * (q_rest - q) * free)
                dq = dq * free
                blocked = ((q <= self.lower + 1e-6) & (dq < 0)) | ((q >= self.upper - 1e-6) & (dq > 0))
                if not (blocked & free).any():
                    break
                free &= ~blocked
            q = np.clip(q + dq, self.lower, self.upper)
        return q
