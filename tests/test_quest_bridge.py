"""quest_bridge.py: controller datagrams over localhost UDP (no CloudXR, no ROS)."""

import socket
import sys
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "quest_teleop"))

from quest_bridge import NetworkQuest, decode, encode, parse_addr  # noqa: E402

QUAT = np.array([0.1, 0.2, 0.3, 0.927])
QUAT = QUAT / np.linalg.norm(QUAT)


def readings(x=0.3, squeeze=1.0, trigger=0.25, tracked_left=False):
    return {"right": (True, np.array([x, -0.2, 1.1]), QUAT, squeeze, trigger),
            "left": (tracked_left, None, None, 0.0, 0.0)}


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.rx = NetworkQuest("127.0.0.1:0", stale_s=0.2, log=lambda m: None)
        self.addr = self.rx.address
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.rx.close()
        self.tx.close()

    def send(self, seq, r, buttons=None, settle=0.05):
        self.tx.sendto(encode(seq, r, buttons or {}), self.addr)
        time.sleep(settle)

    def test_parse_addr(self):
        self.assertEqual(parse_addr("udp://100.89.140.123:47710"), ("100.89.140.123", 47710))
        self.assertEqual(parse_addr(":5000"), ("0.0.0.0", 5000))
        self.assertEqual(parse_addr("udp://0.0.0.0:"), ("0.0.0.0", 47710))

    def test_round_trip_matches_quest_controller_shape(self):
        seq, _, r, b = decode(encode(7, readings(), {"right": (True, False), "left": (False, True)}))
        self.assertEqual(seq, 7)
        ok, pos, quat, squeeze, trigger = r["right"]
        self.assertTrue(ok)
        np.testing.assert_allclose(pos, [0.3, -0.2, 1.1])
        np.testing.assert_allclose(quat, QUAT)
        self.assertEqual((squeeze, trigger), (1.0, 0.25))
        self.assertEqual(r["left"][0], False)
        self.assertEqual(b, {"right": (True, False), "left": (False, True)})

    def test_untracked_before_the_first_datagram(self):
        r = self.rx.read_all()
        self.assertEqual({h: v[0] for h, v in r.items()}, {"left": False, "right": False})
        self.assertEqual(self.rx.age(), float("inf"))

    def test_newest_datagram_wins_and_reordered_ones_are_ignored(self):
        self.send(1, readings(x=0.30))
        self.send(3, readings(x=0.33), {"right": (True, False)})
        self.send(2, readings(x=0.32))  # late duplicate of an older pose
        r = self.rx.read_all()
        self.assertAlmostEqual(r["right"][1][0], 0.33)
        self.assertEqual(self.rx.buttons["right"], (True, False))
        self.assertEqual(self.rx.dropped, 1)  # seq 2 counted as lost when 3 arrived first

    def test_link_loss_releases_the_hands(self):
        self.send(1, readings(), {"right": (True, True)})
        self.assertTrue(self.rx.read_all()["right"][0])
        time.sleep(0.3)  # > stale_s
        r = self.rx.read_all()
        self.assertFalse(r["right"][0])
        self.assertEqual(self.rx.buttons["right"], (False, False))

    def test_garbage_and_non_finite_poses_are_not_tracked(self):
        self.tx.sendto(b"not json", self.addr)
        bad = readings()
        bad["right"] = (True, np.array([np.nan, 0, 0]), QUAT, 1.0, 0.0)
        self.send(1, bad)
        self.assertEqual(self.rx.rejected, 1)
        self.assertFalse(self.rx.read_all()["right"][0])

    def test_allow_list(self):
        rx = NetworkQuest("127.0.0.1:0", allow={"10.9.9.9"}, log=lambda m: None)
        try:
            self.tx.sendto(encode(1, readings(), {}), rx.address)
            time.sleep(0.05)
            self.assertEqual(rx.rejected, 1)
            self.assertFalse(rx.read_all()["right"][0])
        finally:
            rx.close()


if __name__ == "__main__":
    unittest.main()
