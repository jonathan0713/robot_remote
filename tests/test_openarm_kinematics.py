"""OpenArm robot profile, n-joint IK and the per-arm teleop math (numpy only: no ROS, no Quest, no Isaac).

Needs the ITRI_project checkout for the OpenArm URDF (ITRI_PROJECT, default ~/Documents/ITRI_project); the FK
cross-check also needs its TurboVLA tree. Skipped when they are missing.
"""

import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "quest_teleop"))

from kinematics import SerialChain, step_toward  # noqa: E402
from robot_profile import expand, load_profile  # noqa: E402
from teleop_core import ArmTeleop, TeleopLimits  # noqa: E402

ITRI = expand("${ITRI_PROJECT:-~/Documents/ITRI_project}")
URDF = ITRI / "ros2_ws/src/openarm_dual_gripper_description/urdf/openarm_dual_gripper.urdf"
HAVE_OPENARM = URDF.is_file()
# held_tote_start_v5.json: centre-pick reference joints of the right arm (a grasp inside the held tote)
CENTRE_PICK = np.array([-1.109407, 3.237999, 0.679287, 1.165461, -0.201422, 0.659359, 0.901764])


def pose_error(chain, q, t):
    a = chain.fk(q)
    return (np.linalg.norm(a[:3, 3] - t[:3, 3]),
            np.linalg.norm((Rotation.from_matrix(a[:3, :3]) * Rotation.from_matrix(t[:3, :3]).inv()).as_rotvec()))


class ReBeLChainTest(unittest.TestCase):
    """The 6-joint behaviour the ReBeL teleops rely on (bundled URDF, no igus_ws needed)."""

    def setUp(self):
        urdf = (ROOT / "igus_rebel_sim/rebel_6dof.urdf").read_text()
        self.chain = SerialChain(urdf, "base_link", "tcp", [f"joint{i}" for i in range(1, 7)], tcp_xyz=(0, 0, 0.05))

    def test_ik_reaches_nearby_poses_and_posture_keeps_the_solution(self):
        c, rng = self.chain, np.random.default_rng(0)
        for _ in range(50):
            q0 = rng.uniform(c.lower * 0.7, c.upper * 0.7)
            t = c.fk(np.clip(q0 + rng.normal(0, 0.15, 6), c.lower, c.upper))
            q = c.ik(q0, t, iters=60)
            self.assertLess(pose_error(c, q, t)[0], 1e-3)
            # 6 joints: the null space of the full pose task is empty away from singular poses, so the posture
            # task keeps the solution; at a singular pose it may move along the self-motion, never off the pose
            qp = c.ik(q0, t, iters=60, q_rest=np.zeros(6), posture_gain=0.05)
            self.assertLess(pose_error(c, qp, t)[0], 1e-3)
            if np.linalg.svd(c.fk(q, with_jacobian=True)[1], compute_uv=False).min() > 0.01:
                self.assertLess(np.rad2deg(np.abs(qp - q).max()), 0.5)


@unittest.skipUnless(HAVE_OPENARM, f"OpenArm URDF not found at {URDF}")
class OpenArmProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = load_profile("openarm_dual_gripper")

    def test_chains_have_seven_joints_and_skip_the_zero_range_mounts(self):
        for s, side in self.profile.sides.items():
            self.assertEqual(side.chain.n, 7)
            self.assertEqual(sum(axis is not None for _, axis, _ in side.chain.elems), 7)
            self.assertTrue(np.all(side.chain.upper > side.chain.lower))

    def test_unknown_joint_names_are_rejected(self):
        with self.assertRaises(ValueError):
            SerialChain(URDF.read_text(), "openarm_body_link", "openarm_right_ee_tcp",
                        [f"openarm_left_joint{i}" for i in range(1, 8)])

    @unittest.skipUnless((ITRI / "TurboVLA/experiments/openarm/motion/arm_kinematics.py").is_file(), "no TurboVLA")
    def test_fk_matches_turbovla_arm_kinematics(self):
        sys.path.insert(0, str(ITRI / "TurboVLA"))
        from experiments.openarm.motion.arm_kinematics import ArmKinematics
        rng = np.random.default_rng(1)
        for s, side in self.profile.sides.items():
            ref = ArmKinematics(str(URDF), s)
            for _ in range(200):
                q = rng.uniform(side.chain.lower, side.chain.upper)
                np.testing.assert_allclose(side.chain.fk(q), ref.fk(q), atol=1e-9)

    def test_tool_out_axis_points_from_the_wrist_to_the_tcp(self):
        urdf = URDF.read_text()
        rng = np.random.default_rng(2)
        for s, side in self.profile.sides.items():
            wrist = SerialChain(urdf, "openarm_body_link", f"openarm_{s}_link7", side.joints)
            for _ in range(20):
                q = rng.uniform(side.chain.lower, side.chain.upper)
                t, t7 = side.chain.fk(q), wrist.fk(q)
                out = t[:3, :3].T @ (t[:3, 3] - t7[:3, 3])
                np.testing.assert_allclose(out / np.linalg.norm(out), side.tool_out_axis, atol=1e-6)

    def test_gripper_mapping(self):
        right = self.profile.sides["right"]
        self.assertAlmostEqual(right.gripper_position(0.0), 0.040)
        self.assertAlmostEqual(right.gripper_position(1.0), 0.0)
        self.assertAlmostEqual(right.gripper_position(2.0), 0.0)


@unittest.skipUnless(HAVE_OPENARM, f"OpenArm URDF not found at {URDF}")
class OpenArmIKTest(unittest.TestCase):
    """Teleop-like Cartesian paths ready pose -> inside the held tote -> ready pose, warm-started every tick."""

    @classmethod
    def setUpClass(cls):
        cls.side = load_profile("openarm_dual_gripper").sides["right"]
        c, rng = cls.side.chain, np.random.default_rng(2)
        cls.targets = [c.fk(CENTRE_PICK)] + [c.fk(np.clip(CENTRE_PICK + rng.normal(0, 0.25, 7), c.lower, c.upper))
                                             for _ in range(5)]

    def run_path(self, poses, q, **kw):
        c = self.side.chain
        for t in poses:
            q = step_toward(q, c.ik(q, t, **kw), 1.0 / 30)  # 1 rad/s at 30 Hz
        return q

    def line(self, ta, tb, hold=150):
        n = max(2, int(np.ceil(np.linalg.norm(tb[:3, 3] - ta[:3, 3]) / (0.25 / 30))))
        slerp = Slerp([0, 1], Rotation.from_matrix([ta[:3, :3], tb[:3, :3]]))
        out = []
        for s in np.linspace(0, 1, n):
            t = np.eye(4)
            t[:3, 3] = ta[:3, 3] + s * (tb[:3, 3] - ta[:3, 3])
            t[:3, :3] = slerp(s).as_matrix()
            out.append(t)
        return out + [tb] * hold

    def test_reaches_the_tote_and_comes_back_to_the_same_joints(self):
        rest, c = self.side.rest, self.side.chain
        t_ready = c.fk(rest)
        drift_plain = []
        for t in self.targets:
            q = self.run_path(self.line(t_ready, t), rest, q_rest=rest, posture_gain=0.05)
            e_p, e_r = pose_error(c, q, t)
            self.assertLess(e_p, 1e-3)
            self.assertLess(np.rad2deg(e_r), 0.1)
            q = self.run_path(self.line(t, t_ready), q, q_rest=rest, posture_gain=0.05)
            self.assertLess(np.rad2deg(np.abs(q - rest).max()), 1.0, "elbow drifted with the posture task")
            q = self.run_path(self.line(t, t_ready), self.run_path(self.line(t_ready, t), rest))
            drift_plain.append(np.rad2deg(np.abs(q - rest).max()))
        # why the posture task exists: without it the 7th joint's self-motion does not come back
        self.assertGreater(max(drift_plain), 5.0)


@unittest.skipUnless(HAVE_OPENARM, f"OpenArm URDF not found at {URDF}")
class ArmTeleopTest(unittest.TestCase):
    def setUp(self):
        self.side = load_profile("openarm_dual_gripper").sides["right"]
        self.arm = ArmTeleop(self.side.chain, TeleopLimits(max_joint_vel=1.0, max_lin_vel=0.25, max_reach=0.62),
                             tool_axis=self.side.tool_out_axis, q_rest=self.side.rest, posture_gain=0.05)
        self.quat = Rotation.from_euler("xyz", [10, -20, 30], degrees=True).as_quat()
        self.pos = np.array([0.3, -0.2, 1.1])
        self.arm.engage(self.side.rest, self.pos, self.quat)
        self.t0 = self.side.chain.fk(self.side.rest)

    def run_ticks(self, pos, quat, n=90):
        for _ in range(n):
            q = self.arm.step(pos, quat)
        return self.side.chain.fk(q)

    def test_still_hand_holds_the_arm(self):
        t = self.run_ticks(self.pos, self.quat, 30)
        self.assertLess(np.linalg.norm(t[:3, 3] - self.t0[:3, 3]), 1e-4)

    def test_hand_forward_moves_the_tcp_forward(self):
        t = self.run_ticks(self.pos + [0.05, 0.0, 0.0], self.quat)
        np.testing.assert_allclose(t[:3, 3] - self.t0[:3, 3], [0.05, 0.0, 0.0], atol=1e-3)
        r = Rotation.from_matrix(self.t0[:3, :3]).inv() * Rotation.from_matrix(t[:3, :3])
        self.assertLess(np.rad2deg(r.magnitude()), 0.2)

    def test_twisting_the_handle_spins_the_tool_about_its_outward_axis(self):
        twist = np.array(self.arm.ctrl_twist_axis) / np.linalg.norm(self.arm.ctrl_twist_axis)
        quat = (Rotation.from_quat(self.quat) * Rotation.from_rotvec(np.deg2rad(20) * twist)).as_quat()
        t = self.run_ticks(self.pos, quat)
        self.assertLess(np.linalg.norm(t[:3, 3] - self.t0[:3, 3]), 1e-3)
        r = Rotation.from_matrix(self.t0[:3, :3]).inv() * Rotation.from_matrix(t[:3, :3])  # in the tool frame
        self.assertAlmostEqual(np.rad2deg(r.magnitude()), 20.0, delta=0.5)
        np.testing.assert_allclose(r.as_rotvec() / r.magnitude(), self.side.tool_out_axis, atol=0.02)

    def test_rot_scale_turns_the_tool_a_fraction_of_the_hand(self):
        arm = ArmTeleop(self.side.chain, TeleopLimits(max_joint_vel=1.0, max_lin_vel=0.25, max_reach=0.62, rot_scale=0.5),
                        tool_axis=self.side.tool_out_axis, q_rest=self.side.rest, posture_gain=0.05)
        arm.engage(self.side.rest, self.pos, self.quat)
        twist = np.array(arm.ctrl_twist_axis) / np.linalg.norm(arm.ctrl_twist_axis)
        quat = (Rotation.from_quat(self.quat) * Rotation.from_rotvec(np.deg2rad(20) * twist)).as_quat()
        for _ in range(90):
            q = arm.step(self.pos, quat)
        t = self.side.chain.fk(q)
        r = Rotation.from_matrix(self.t0[:3, :3]).inv() * Rotation.from_matrix(t[:3, :3])
        self.assertAlmostEqual(np.rad2deg(r.magnitude()), 10.0, delta=0.5)


    def test_rotation_scales_per_controller_axis(self):
        """twist / nod / side x0.5 / 0.75 / 1.0: a held controller swings little to the side, so that axis is not cut."""
        from teleop_core import axes_basis
        lim = TeleopLimits(max_joint_vel=1.0, max_lin_vel=0.25, max_reach=0.62, rot_scale=0.5,
                           rot_scale_axes=(0.5, 0.75, 1.0))
        for name, column, scale in (("twist", 2, 0.5), ("nod", 0, 0.75), ("side", 1, 1.0)):
            with self.subTest(axis=name):
                arm = ArmTeleop(self.side.chain, lim, tool_axis=self.side.tool_out_axis, q_rest=self.side.rest,
                                posture_gain=0.05)
                arm.engage(self.side.rest, self.pos, self.quat)
                axis = arm.ctrl_basis[:, column]
                quat = (Rotation.from_quat(self.quat) * Rotation.from_rotvec(np.deg2rad(16) * axis)).as_quat()
                p_des, r_des = arm.target(self.pos, quat)
                r = Rotation.from_matrix(self.t0[:3, :3]).inv() * r_des  # in the tool frame
                self.assertAlmostEqual(np.rad2deg(r.magnitude()), 16.0 * scale, delta=0.05)
                np.testing.assert_allclose(p_des, self.t0[:3, 3], atol=1e-9)
        # equal scales are the single rot_scale
        r0 = Rotation.from_rotvec([0.2, -0.1, 0.3])
        basis = axes_basis((0.0, -0.707, -0.707), (1.0, 0.2, 0.0))
        from teleop_core import scaled, scaled_axes
        np.testing.assert_allclose(scaled_axes(r0, basis, 0.6, 0.6, 0.6).as_rotvec(), scaled(r0, 0.6).as_rotvec(),
                                   atol=1e-12)

    def test_the_profile_scales_rotation_per_axis(self):
        from robot_profile import rot_scale_axes
        self.assertEqual(rot_scale_axes(load_profile("openarm_dual_gripper").operator), (0.5, 0.75, 1.0))
        self.assertEqual(rot_scale_axes({"rot_scale_axes": [1, 2, 3]}), (1.0, 2.0, 3.0))
        self.assertIsNone(rot_scale_axes({"rot_scale": 0.5}))
        with self.assertRaises(ValueError):
            rot_scale_axes({"rot_scale_axes": {"twist": 0.5, "nod": -1, "side": 1}})

    def test_the_trace_keeps_what_the_hand_asked_for_and_what_the_arm_did(self):
        import tempfile
        from types import SimpleNamespace
        from teleop_core import TeleopTrace
        arm = ArmTeleop(self.side.chain, TeleopLimits(max_joint_vel=1.0, max_lin_vel=0.25, max_reach=0.62),
                        tool_axis=self.side.tool_out_axis, q_rest=self.side.rest, posture_gain=0.05)
        arm.q_cmd = np.array(self.side.rest)
        node = SimpleNamespace(arms={"right": arm}, last_hand={}, q_meas={"right": None})
        trace = TeleopTrace(0.0)
        trace.add(node, {"right": False}, False)                       # not tracked, not engaged: NaN blocks
        arm.engage(self.side.rest, self.pos, self.quat)
        node.last_hand["right"] = (self.pos, self.quat)
        node.q_meas["right"] = np.array(self.side.rest)
        arm.step(self.pos + [0.01, 0, 0], self.quat)
        trace.add(node, {"right": True}, True)
        with tempfile.TemporaryDirectory() as d:
            trace.save(Path(d) / "teleop_trace.npz")
            z = np.load(Path(d) / "teleop_trace.npz")
            cols = list(z["columns"])
            rows = z["right"]
        self.assertEqual(rows.shape, (2, len(cols)))
        self.assertTrue(np.isnan(rows[0, cols.index("target_x")]) and rows[1, cols.index("engaged")] == 1.0)
        self.assertAlmostEqual(rows[1, cols.index("target_x")] - self.t0[0, 3], 0.01, places=9)  # scale 1
        np.testing.assert_allclose(rows[1, cols.index("q_meas1"):cols.index("q_meas7") + 1], self.side.rest)


    def test_the_trace_report_finds_a_rotation_the_arm_did_not_follow(self):
        import trace_report
        from teleop_core import TeleopTrace
        c, rest = self.side.chain, np.array(self.side.rest)
        t_rest = c.fk(rest)
        cols = list(TeleopTrace.COLUMNS)
        rows = np.full((20, len(cols)), np.nan)
        rows[:, cols.index("engaged")] = 1.0
        target_r = Rotation.from_matrix(t_rest[:3, :3]) * Rotation.from_rotvec([0.0, 0.0, np.deg2rad(20)])  # spin
        rows[:, cols.index("target_x"):cols.index("target_z") + 1] = t_rest[:3, 3] + [0.003, 0, 0]
        rows[:, cols.index("target_qx"):cols.index("target_qw") + 1] = target_r.as_quat()
        rows[:, cols.index("q_meas1"):cols.index("q_meas7") + 1] = rest  # the arm stayed where it was
        r = trace_report.lag(rows, cols, c)
        self.assertEqual(r["engaged_ticks"], 20)
        self.assertAlmostEqual(r["position_lag_mm"]["median"], 3.0, places=6)
        self.assertAlmostEqual(r["rotation_lag_deg"]["median"], 20.0, places=6)
        axes = r["rotation_lag_rms_deg_per_tool_axis"]
        self.assertAlmostEqual(axes["z"], 20.0, places=1)
        self.assertLess(max(axes["x"], axes["y"]), 1e-6)
        rows[:, cols.index("engaged")] = 0.0
        self.assertIsNone(trace_report.lag(rows, cols, c))


if __name__ == "__main__":
    unittest.main()
