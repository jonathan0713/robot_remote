"""Robot profiles (config/robots/<name>.json): everything robot-specific the Quest teleop needs.

A profile names the URDF and its base link, and per side: the IK tip link, the arm joints, the outward tool axis
(twisting the controller spins the tool about it), the gripper joints with their open / closed positions, the
command topic and the posture-task rest pose. The URDF path may use ${VAR:-default} and ~.
"""

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from kinematics import SerialChain

PROFILES = Path(__file__).resolve().parents[1] / "config" / "robots"


def expand(path: str) -> Path:
    """${VAR:-default} / ${VAR} / $VAR and ~, like the shell."""
    def sub(m):
        name, default = m.group(1) or m.group(3), m.group(2)
        return os.environ.get(name) or (default if default is not None else "")
    return Path(os.path.expanduser(re.sub(r"\$\{(\w+)(?::-([^}]*))?\}|\$(\w+)", sub, path))).expanduser()


@dataclass
class Side:
    name: str
    tip: str
    joints: list
    tool_out_axis: np.ndarray
    gripper_joints: list
    gripper_open: float
    gripper_closed: float
    command_topic: str
    rest: np.ndarray
    chain: SerialChain

    def gripper_position(self, closedness: float) -> float:
        """Trigger 0 (open) .. 1 (closed) -> gripper joint position."""
        c = float(np.clip(closedness, 0.0, 1.0))
        return self.gripper_open + (self.gripper_closed - self.gripper_open) * c


@dataclass
class RobotProfile:
    name: str
    robot_type: str
    urdf_path: Path
    base_link: str
    state_topic: str
    app_name: str
    limits: dict
    ik: dict
    sides: dict
    operator: dict = None  # operator defaults (scale, rot_scale, gripper_mode); CLI flags override


def load_profile(name_or_path) -> RobotProfile:
    path = Path(name_or_path)
    if not path.suffix:
        path = PROFILES / f"{name_or_path}.json"
    data = json.loads(path.read_text())
    urdf_path = expand(data["urdf"])
    if not urdf_path.is_file():
        raise FileNotFoundError(f"profile {data['name']}: URDF {urdf_path} not found (set ITRI_PROJECT?)")
    urdf = urdf_path.read_text()
    sides = {}
    for side, s in data["sides"].items():
        chain = SerialChain(urdf, data["base_link"], s["tip"], s["joints"])
        rest = np.asarray(s["rest"], dtype=float)
        if rest.shape != (chain.n,) or np.any(rest < chain.lower) or np.any(rest > chain.upper):
            raise ValueError(f"profile {data['name']}: {side} rest pose is not {chain.n} joints inside the limits")
        g = s["gripper"]
        sides[side] = Side(side, s["tip"], list(s["joints"]), np.asarray(s["tool_out_axis"], dtype=float),
                           list(g["joints"]), float(g["open"]), float(g["closed"]), s["command_topic"], rest, chain)
    return RobotProfile(data["name"], data["robot_type"], urdf_path, data["base_link"], data["state_topic"],
                        data.get("cloudxr_app_name", "QuestTeleop"), dict(data["limits"]), dict(data["ik"]), sides,
                        dict(data.get("operator", {})))
