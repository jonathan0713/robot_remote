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


if __name__ == "__main__":
    unittest.main()
