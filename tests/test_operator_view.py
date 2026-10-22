"""operator_view.py's image decoding, status bar and tiling (no window, no ROS)."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "quest_teleop"))

import operator_view as view  # noqa: E402

try:
    import cv2  # noqa: F401
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False


def image(encoding, rgb, pad=0):
    h, w, _ = rgb.shape
    channels = 4 if encoding.endswith("a8") else 3
    px = rgb if encoding.startswith("rgb") else rgb[..., ::-1]
    if channels == 4:
        px = np.concatenate([px, np.full((h, w, 1), 255, np.uint8)], axis=2)
    rows = np.concatenate([px.reshape(h, w * channels), np.zeros((h, pad), np.uint8)], axis=1)
    return SimpleNamespace(encoding=encoding, height=h, width=w, step=w * channels + pad, data=rows.tobytes())


class OperatorViewTest(unittest.TestCase):
    def test_image_decoding_gives_bgr_for_every_encoding_and_row_padding(self):
        rgb = np.zeros((2, 3, 3), np.uint8)
        rgb[..., 0] = 200  # red
        for enc in ("rgb8", "bgr8", "rgba8", "bgra8"):
            bgr = view.image_to_bgr(image(enc, rgb, pad=4))
            self.assertEqual(bgr.shape, (2, 3, 3), enc)
            self.assertTrue(np.all(bgr[..., 2] == 200) and np.all(bgr[..., 0] == 0), enc)
        self.assertIsNone(view.image_to_bgr(SimpleNamespace(encoding="32FC1")))

    def test_status_shows_arms_gripper_recording_and_the_pick(self):
        state = {"right": "engaged", "capturing": True, "gripper": {"right": 0.25}}
        pick = {"clearance_m": 0.031, "hold_s": 1.4, "succeeded": False}
        lines = view.status_lines(state, pick, now=10.0, state_t=9.9, pick_t=9.9)
        self.assertIn("right: ENGAGED gripper 25% open", lines[0][0])
        self.assertIn("RECORDING", lines[0][0])
        self.assertIn("+3.1 cm over the rim", lines[1][0])
        self.assertIn("1.4 / 2.0 s", lines[1][0])
        held = view.status_lines(dict(state, guard={"clearance_m": 0.008, "holding": True}), None, 10.0, 9.9, 0.0)
        self.assertIn("GUARD: arms 0.8 cm apart", held[1][0])
        stale = view.status_lines(state, pick, now=10.0, state_t=5.0, pick_t=5.0)
        self.assertEqual(len(stale), 1)
        self.assertIn("no /quest_teleop/state", stale[0][0])

    @unittest.skipUnless(HAVE_CV2, "no OpenCV")
    def test_tiles_are_placed_and_a_stale_one_is_dimmed(self):
        tiles = [("a", "/a"), ("b", "/b"), ("c", "/c"), ("d", "/d")]
        white = np.full((90, 160, 3), 255, np.uint8)
        frames = {"/a": white, "/b": white, "/c": white}
        stamps = {"/a": 10.0, "/b": 10.0, "/c": 5.0}
        canvas = view.compose(tiles, frames, stamps, [("x", (255, 255, 255))], now=10.0, tile=(160, 90))
        self.assertEqual(canvas.shape, (2 * 90 + 40, 320, 3))
        self.assertEqual(canvas[60, 80].tolist(), [255, 255, 255])        # fresh tile
        self.assertLess(canvas[150, 80].max(), 100)                       # stale tile (bottom left), dimmed
        self.assertLess(canvas[150, 240].max(), 200)                      # no image yet (bottom right)


    def test_the_round_runner_heads_the_status_bar_and_between_attempts_no_teleop_is_fine(self):
        import json as _json
        import os
        import tempfile
        state = {"right": "engaged", "capturing": False}
        ready = {"phase": "ready", "text": "READY - A = attempt 1: green tea pack", "last": "attempt 0: admitted",
                 "pid": os.getpid()}
        lines = view.status_lines(None, None, now=10.0, state_t=0.0, pick_t=0.0, runner=ready)
        self.assertEqual(lines[0], ("READY - A = attempt 1: green tea pack", (0, 255, 0)))
        self.assertEqual(lines[1][0], "last: attempt 0: admitted")
        self.assertFalse(any("no /quest_teleop/state" in t for t, _ in lines))  # no teleop between attempts
        teleop = dict(ready, phase="teleop", text="attempt 1: green tea pack")
        self.assertTrue(any("no /quest_teleop/state" in t for t, _ in view.status_lines(None, None, 10.0, 0.0, 0.0,
                                                                                         teleop)))
        self.assertIn("RECORDING", view.status_lines(dict(state, capturing=True), None, 10.0, 9.9, 0.0, teleop)[2][0])
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "runner_status.json"
            self.assertIsNone(view.read_runner_status(path))       # no runner yet
            path.write_text(_json.dumps(ready))
            self.assertEqual(view.read_runner_status(path)["phase"], "ready")
            path.write_text(_json.dumps(dict(ready, pid=2 ** 22 + 12345)))
            self.assertIsNone(view.read_runner_status(path))       # its runner is gone: a stale file shows nothing
        self.assertIsNone(view.read_runner_status(None))

    @unittest.skipUnless(HAVE_CV2, "no OpenCV")
    def test_the_ready_banner_crosses_the_camera_tiles(self):
        canvas = np.zeros((760, 1280, 3), np.uint8)
        view.overlay(canvas, "READY - press A")
        self.assertEqual(canvas[360 - 45, 640].tolist(), [0, 255, 0])  # its green frame, mid-height of the tiles
        self.assertEqual(canvas[100, 640].tolist(), [0, 0, 0])        # the tiles above stay untouched


    def test_the_gripper_depth_line_warns_near_the_floor(self):
        pick = {"clearance_m": -0.1, "hold_s": 0.0, "succeeded": False, "tip_above_floor_m": 0.032,
                "item_top_above_floor_m": 0.105, "tcp_off_item_centre_m": 0.014}
        lines = view.status_lines({"right": "engaged", "capturing": True}, pick, 10.0, 9.9, 9.9)
        self.assertEqual(lines[-1][0], "gripper +3.2 cm over the floor   item top 10.5 cm   TCP 1.4 cm off the item centre")
        self.assertEqual(lines[-1][1], (255, 255, 255))
        self.assertEqual(view.status_lines(None, dict(pick, tip_above_floor_m=0.008), 10.0, 0.0, 9.9)[-1][1], (0, 0, 255))
        self.assertEqual(view.status_lines(None, dict(pick, tip_above_floor_m=0.02), 10.0, 0.0, 9.9)[-1][1], (0, 255, 255))

    @unittest.skipUnless(HAVE_CV2, "no OpenCV")
    def test_the_depth_bar_marks_the_gripper_over_the_floor(self):
        canvas = np.zeros((760, 1280, 3), np.uint8)
        view.depth_gauge(canvas, {"tip_above_floor_m": 0.005, "item_top_above_floor_m": 0.10})
        bottom, top = 720 - 60, 60
        y = int(bottom - (bottom - top) * 0.005 / 0.20)
        self.assertEqual(canvas[y, 1280 - 55].tolist(), [0, 0, 255])  # red: under 1 cm
        self.assertGreater(canvas[int(bottom - (bottom - top) * 0.05 / 0.2), 1280 - 55, 1], 100)  # item column
        before = canvas.copy()
        view.depth_gauge(canvas, {"clearance_m": 0.0})  # no depth (no kinematics): nothing drawn
        self.assertTrue((canvas == before).all())


if __name__ == "__main__":
    unittest.main()
