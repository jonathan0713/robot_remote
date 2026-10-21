"""quest_openarm_teleop.py end to end with a fake Quest and a fake Isaac (no headset, no simulator).

The fake Isaac follows /right_joint_command and /left_joint_command at once, publishes /isaac_joint_states and, like
the chain's joint_command_mux.py, the merged targets on /joint_command. The fake Quest grips, moves the hand 10 cm
forward, starts the capture (A), closes the trigger and ends the attempt (Y). A FIFO stands in for the chain's
capture control. Needs ROS 2 Humble's rclpy (source /opt/ros/humble/setup.bash) and the OpenArm URDF.

Runs on its own ROS domain, localhost only.
"""

import os
import random
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "quest_teleop"))

try:
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(150, 200))
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    import quest_openarm_teleop as teleop
    from robot_profile import load_profile
    HAVE = True
    WHY = ""
except Exception as exc:  # no ROS / no URDF
    HAVE, WHY = False, f"needs rclpy + the OpenArm profile ({exc})"


class FakeIsaac:
    """Joint targets follow the commands instantly; states at ~60 Hz, merged commands at ~30 Hz."""

    def __init__(self, profile):
        self.ctx = Context()
        rclpy.init(context=self.ctx)
        self.node = rclpy.create_node("fake_isaac", context=self.ctx)
        self.q = {}
        for s, side in profile.sides.items():
            self.q.update(zip(side.joints, side.rest))
            self.q[side.gripper_joints[0]] = side.gripper_open
        self.q["openarm_left_finger_joint1"] = 0.0105  # the left jaw holds the tote
        self.received = {"/left_joint_command": [], "/right_joint_command": []}
        for topic in self.received:
            self.node.create_subscription(JointState, topic, lambda m, t=topic: self.on_cmd(t, m), 10)
        self.state_pub = self.node.create_publisher(JointState, "/isaac_joint_states", 10)
        self.cmd_pub = self.node.create_publisher(JointState, "/joint_command", 10)
        self.node.create_timer(1 / 60, self.tick)
        self.node.create_timer(1 / 30, lambda: self.cmd_pub.publish(self.msg()))
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()

    def msg(self):
        return JointState(name=list(self.q), position=[float(v) for v in self.q.values()])

    def on_cmd(self, topic, msg):
        self.received[topic].append(dict(zip(msg.name, msg.position)))
        self.q.update(zip(msg.name, msg.position))

    def tick(self):
        m = self.msg()
        m.header.stamp = self.node.get_clock().now().to_msg()
        self.state_pub.publish(m)

    def close(self):
        self.ex.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.ctx)


class ScriptedQuest:
    """Both hands; the right hand drives. Tick-based script, see the module docstring."""

    def __init__(self):
        self.k = 0
        self.quat = Rotation.from_euler("xyz", [10, -20, 30], degrees=True).as_quat()
        self.buttons = {"left": (False, False), "right": (False, False)}

    def read_all(self):
        k = self.k = self.k + 1
        squeeze = 1.0 if 20 <= k < 160 else 0.0
        x = 0.3 + 0.1 * np.clip((k - 30) / 30, 0, 1)
        trigger = float(np.clip((k - 110) / 30, 0, 1))
        self.buttons = {"left": (False, 170 <= k < 175), "right": (100 <= k < 104, False)}
        right = (True, np.array([x, -0.2, 1.1]), self.quat, squeeze, trigger)
        left = (True, np.array([0.3, 0.2, 1.1]), self.quat, 0.0, 0.0)
        return {"left": left, "right": right}

    def close(self):
        pass


class UdpQuestSender:
    """Plays a ScriptedQuest through quest_bridge datagrams, like `quest_bridge.py send` next to the headset."""

    def __init__(self, port, hz=60.0):
        from quest_bridge import encode
        import socket
        self.quest, self.stop = ScriptedQuest(), threading.Event()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        def run():
            seq = 0
            while not self.stop.is_set():
                seq += 1
                sock.sendto(encode(seq, self.quest.read_all(), self.quest.buttons), ("127.0.0.1", port))
                time.sleep(1.0 / hz)
            sock.close()
        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.0)


@unittest.skipUnless(HAVE, WHY)
class OpenArmTeleopRosTest(unittest.TestCase):
    def test_right_arm_session_with_capture_and_end(self):
        self.run_session(quest_over_udp=False)

    def test_same_session_with_the_controllers_over_udp(self):
        self.run_session(quest_over_udp=True)

    def run_session(self, quest_over_udp):
        profile = load_profile("openarm_dual_gripper")
        right = profile.sides["right"]
        isaac = FakeIsaac(profile)
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            fifo = run_dir / "capture.fifo"
            os.mkfifo(fifo)
            got = bytearray()

            def reader():  # like the chain's `cat capture.fifo`, until the writer closes it
                with open(fifo, "rb", buffering=0) as f:
                    while b := f.read(1):
                        got.extend(b)
            th = threading.Thread(target=reader, daemon=True)
            th.start()
            argv = ["--arms", "right", "--run-dir", str(run_dir), "--max-run-s", "30"]
            sender = None
            try:
                if quest_over_udp:
                    port = random.randint(47720, 47790)
                    real = teleop.NetworkQuest

                    def receiver_then_sender(*a, **kw):  # start the script once the teleop listens
                        nonlocal sender
                        rx = real(*a, **kw)
                        sender = UdpQuestSender(port)
                        return rx
                    with patch.object(teleop, "NetworkQuest", side_effect=receiver_then_sender):
                        teleop.main(argv + ["--quest-source", f"udp://127.0.0.1:{port}"])
                else:
                    with patch.object(teleop, "QuestController", return_value=ScriptedQuest()):
                        teleop.main(argv)
            finally:
                if sender:
                    sender.close()
                isaac.close()
            th.join(timeout=2)
            self.assertFalse(th.is_alive(), "the teleop did not close the capture FIFO")
            self.assertEqual(bytes(got), b"  ", "capture start (A) + end (on Y)")
            self.assertTrue((run_dir / "release").exists())
            self.assertFalse((run_dir / "teleop_discard.json").exists())

        # never a command for the left arm (the tote teacher owns it)
        self.assertEqual(isaac.received["/left_joint_command"], [])
        cmds = isaac.received["/right_joint_command"]
        self.assertGreater(len(cmds), 50)  # ~85 when the script runs at the 60 Hz sender rate
        self.assertEqual(set(cmds[0]), set(right.joints) | set(right.gripper_joints))
        # the first command is the pose Isaac already holds (no jump)
        q0 = np.array([cmds[0][j] for j in right.joints])
        np.testing.assert_allclose(q0, right.rest, atol=1e-6)
        # 10 cm forward with the hand -> 10 cm forward with the TCP (openarm_body_link +x)
        q_end = np.array([cmds[-1][j] for j in right.joints])
        d = right.chain.fk(q_end)[:3, 3] - right.chain.fk(right.rest)[:3, 3]
        np.testing.assert_allclose(d, [0.1, 0.0, 0.0], atol=2e-3)
        # trigger 0 -> 1 closed the gripper from 0.040 to 0.0
        grip = [c[right.gripper_joints[0]] for c in cmds]
        self.assertAlmostEqual(grip[0], right.gripper_open)
        self.assertAlmostEqual(grip[-1], right.gripper_closed, places=4)


if __name__ == "__main__":
    unittest.main()
