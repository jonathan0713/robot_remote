"""Inter-arm collision model: each arm is a chain of capsules (segments with a radius) along its links and
tooling; the clearance between the two arms is the smallest surface-to-surface distance of any capsule pair.
capsules() is the ReBeL model (flange, XEG-32, knife); link_capsules() works for any SerialChain.

Arm bases are placed in one common frame: from the hand-eye calibrations (config/arm_calibration.json,
camera_link in each base_link -> world = camera_link) on the real arms, or from the simulation USD poses.
This is a coarse safety margin, not a full mesh collision check (stand, table and cameras are not modelled).
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIBRATION = ROOT / "config" / "arm_calibration.json"


def load_world_T_base(path=DEFAULT_CALIBRATION):
    """{side: 4x4 world_T_base} with world = the shared camera_link of the two hand-eye calibrations."""
    data = json.loads(Path(path).read_text())["camera_in_base"]
    out = {}
    for side, c in data.items():
        base_T_cam = np.eye(4)
        base_T_cam[:3, :3] = Rotation.from_quat(c["rotation_xyzw"]).as_matrix()
        base_T_cam[:3, 3] = c["translation"]
        out[side] = np.linalg.inv(base_T_cam)
    return out


def chain_points(chain, q):
    """Base-frame points along the arm: the six joint origins, then flange, XEG-32 jaw centre and knife tip."""
    from quest_arm_teleop import TCP_PRESETS  # ReBeL only; imported here so this module needs no ROS
    t, pts = np.eye(4), []
    last = max(i for i, (_, axis, _) in enumerate(chain.elems) if axis is not None)
    for t_o, axis, idx in chain.elems[:last + 1]:  # up to link6; a control-point offset may follow
        t = t @ t_o
        if axis is not None:
            pts.append(t[:3, 3].copy())
            r = np.eye(4)
            r[:3, :3] = Rotation.from_rotvec(axis * q[idx]).as_matrix()
            t = t @ r
    for name in ("flange", "gripper", "eef"):
        pts.append(t[:3, :3] @ np.array(TCP_PRESETS[name]) + t[:3, 3])
    return pts


def capsules(chain, q, world_T_base, arm_radius, tool_radius):
    """[(p0, p1, radius)] in the world frame: base column, upper arm, forearm, wrist, flange, gripper, knife."""
    j1, j2, j3, j4, j5, j6, flange, jaw, knife = (world_T_base[:3, :3] @ p + world_T_base[:3, 3]
                                                  for p in chain_points(chain, q))
    return [(j1, j2, arm_radius), (j2, j3, arm_radius), (j3, j4, arm_radius), (j4, j5, arm_radius),
            (j5, j6, arm_radius), (j6, flange, arm_radius), (flange, jaw, tool_radius), (flange, knife, tool_radius)]


def link_capsules(chain, q, link_radius, tool_radius, world_T_base=None):
    """[(p0, p1, radius)] in the world frame for any SerialChain: segments between consecutive joint origins
    (link_radius), then last joint origin -> chain tip (TCP; tool_radius)."""
    t, pts = np.eye(4), []
    for t_o, axis, idx in chain.elems:
        t = t @ t_o
        if axis is not None:
            pts.append(t[:3, 3].copy())
            r = np.eye(4)
            r[:3, :3] = Rotation.from_rotvec(axis * q[idx]).as_matrix()
            t = t @ r
    pts.append(t[:3, 3].copy())
    if world_T_base is not None:
        pts = [world_T_base[:3, :3] @ p + world_T_base[:3, 3] for p in pts]
    caps = [(a, b, link_radius) for a, b in zip(pts[:-2], pts[1:-1])]
    return caps + [(pts[-2], pts[-1], tool_radius)]


def segment_distance(p1, q1, p2, q2):
    """Closest distance between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection 5.1.9)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = d1 @ r
        if e <= eps:
            s, t = np.clip(-c / a, 0.0, 1.0), 0.0
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = np.clip((b * f - c * e) / den, 0.0, 1.0) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                s, t = np.clip(-c / a, 0.0, 1.0), 0.0
            elif t > 1.0:
                s, t = np.clip((b - c) / a, 0.0, 1.0), 1.0
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def clearance(caps_a, caps_b):
    """Smallest surface-to-surface distance between two capsule sets [m]; negative = overlapping."""
    return min(segment_distance(p0, p1, s0, s1) - ra - rb for p0, p1, ra in caps_a for s0, s1, rb in caps_b)
