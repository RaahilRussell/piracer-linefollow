"""Run: python3 -m unittest -v tests.test_lane_tracker   (from the project root)

What the lane tracker (linetrack/detector.py) must get right, on synthetic scenes with
known geometry and on the recorded frames where the earlier detector failed.

tests/frames/carpet/ is from run 20261109-193803 (Full auto on the carpet venue), with the
part of the image above the floor blacked out:
  000296-000345  left-hand bend: only the right line in view, its mask broken up by glare.
                 The old detector reported LOST for almost all of it.
  001124-001149  the car is on top of the right line. The old detector called it the LEFT
                 line and steered hard right, off the track.
"""
import glob
import os
import unittest

import cv2
import numpy as np

from linetrack.config import Config
from linetrack.detector import LeftLineDetector
from tests.test_linetrack import H, MAT, ORANGE, W, scene, stripe

HERE = os.path.dirname(__file__)
CARPET = sorted(glob.glob(os.path.join(HERE, "frames", "carpet", "*.jpg")))


def carpet(lo, hi):
    return [f for f in CARPET if lo <= int(os.path.basename(f)[:6]) <= hi]


class TestMask(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.det = LeftLineDetector(self.cfg)

    @staticmethod
    def bgr(h, s, v):
        return cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0]

    def test_nearly_orange_hole_in_the_tape_is_filled(self):
        img = scene(left=False)
        strict = int((self.det.detect(img).mask > 0).sum())
        holed = img.copy()
        holed[150:156, 196:202] = self.bgr(12, 150, 230)      # glare: hue just past the strict limit
        self.assertEqual(int((LeftLineDetector(self.cfg).detect(holed).mask > 0).sum()), strict)
        self.cfg.mask_grow_px = 0
        self.assertLess(int((LeftLineDetector(self.cfg).detect(holed).mask > 0).sum()), strict)

    def test_nearly_orange_alone_is_not_tape(self):
        img = np.full((H, W, 3), MAT, np.uint8)
        img[100:190, 20:60] = self.bgr(12, 150, 230)
        self.assertEqual(int((self.det.detect(img).mask > 0).sum()), 0)

    def test_growth_does_not_flood_a_nearly_orange_surface(self):
        img = scene(left=False)
        img[90:192, 0:120] = self.bgr(12, 100, 170)            # big wood-coloured area, not touching the tape
        d = self.det.detect(img)
        self.assertEqual(int((d.mask[:, :120] > 0).sum()), 0)


class TestCandidates(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.det = LeftLineDetector(self.cfg)

    def test_broken_line_is_one_candidate(self):
        img = scene()
        for y in (120, 140, 165):                               # three gaps across both lines
            img[y:y + 5] = MAT
        d = self.det.detect(img)
        self.assertEqual(len(d.stripes), 2, [s.bbox for s in d.stripes])
        self.assertTrue(d.found)
        self.assertEqual(d.source, "left")
        self.assertGreater(np.ptp(d.edge_points[:, 1]), 0.4 * H, "the fit should span the gaps")

    def test_speckled_stripe_keeps_its_true_edges(self):
        clean = self.det.detect(scene())
        img = scene()
        rng = np.random.default_rng(0)
        ys, xs = np.nonzero((img == ORANGE).all(2))
        pick = rng.choice(len(ys), len(ys) // 12, replace=False)
        img[ys[pick], xs[pick]] = MAT                           # 8% of tape pixels knocked out
        d = LeftLineDetector(self.cfg).detect(img)
        self.assertTrue(d.found)
        self.assertAlmostEqual(d.x_ratio, clean.x_ratio, delta=0.02)
        self.assertAlmostEqual(d.right_x_ratio, clean.right_x_ratio, delta=0.02)

    def test_shallow_diagonal_tape_is_not_mistaken_for_a_fat_blob(self):
        # a right line crossing the view at a shallow angle covers many pixels per row
        img = np.full((H, W, 3), MAT, np.uint8)
        stripe(img, lambda y: 20 + 2.6 * (y - 90), lambda y: 55, 90)
        d = self.det.detect(img)
        self.assertEqual(len(d.stripes), 1)
        self.assertTrue(d.found, d.reason)
        self.assertEqual(d.source, "right")

    def test_ragged_blob_is_dropped(self):
        img = scene(right=False)
        rng = np.random.default_rng(1)
        for y in range(100, 190):                               # tape-coloured, but with torn edges
            a = 170 + int(rng.integers(-12, 12))
            img[y, a:a + 18 + int(rng.integers(-8, 8))] = ORANGE
        d = self.det.detect(img)
        self.assertFalse([s for s in d.stripes if s.near_x > 0.5 * W and s.substantial], "the blob is not a line")
        self.assertIsNone(d.right_selected)
        self.assertEqual(d.source, "left")


class TestIdentity(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.cfg.lane_width_ratio = 0.30                        # the synthetic lane is ~0.27 W wide
        self.det = LeftLineDetector(self.cfg)

    @staticmethod
    def right_only(shift):
        """The synthetic right boundary moved `shift` (fraction of width) to the left."""
        img = np.full((H, W, 3), MAT, np.uint8)
        yt = 0.42 * H
        u = lambda y: (H - y) / (H - yt)                        # noqa: E731
        stripe(img, lambda y: (0.92 - shift) * W - 80 * u(y), lambda y: 26 - 14 * u(y), yt)
        return img

    def test_right_line_stays_right_when_it_drifts_across_the_view(self):
        """The car drifts onto the right line: it slides to the centre of the image and beyond.
        It is still the RIGHT line, so the output must keep falling (steer left), never jump to
        'this is the left line, steer right'."""
        for _ in range(3):
            d = self.det.detect(scene())
        self.assertEqual(d.source, "left")
        xs = []
        for k in range(1, 40):
            d = self.det.detect(self.right_only(0.015 * k))    # ends 0.58 W to the left
            self.assertTrue(d.found, f"step {k}: {d.reason}")
            self.assertEqual(d.source, "right", f"step {k}")
            self.assertIsNone(d.selected, f"step {k}: right line taken for the left one")
            xs.append(d.x_ratio)
        self.assertLess(xs[-1], xs[0] - 0.4)
        self.assertLess(max(np.diff(xs)), 0.01, "must fall steadily")

    def test_same_line_seen_fresh_in_the_middle_is_ambiguous(self):
        # without history, a single near-vertical line under the car could be either boundary
        img = np.full((H, W, 3), MAT, np.uint8)
        stripe(img, lambda y: 0.47 * W, lambda y: 14, 0.5 * H)
        d = self.det.detect(img)
        self.assertFalse(d.found)

    def test_boundary_survives_a_short_dropout_without_renaming(self):
        for _ in range(3):
            self.det.detect(scene())
        blank = np.full((H, W, 3), MAT, np.uint8)
        for _ in range(self.cfg.track_memory_frames - 1):
            self.assertFalse(self.det.detect(blank).found)
        d = self.det.detect(self.right_only(0.30))             # reappears well left of where it was
        self.assertEqual(d.source, "right")

    def test_wrong_start_is_corrected_when_the_other_line_appears(self):
        """Started on the middle line with something orange to its right (looks like a lane);
        then the real left line comes into view: the middle line must become RIGHT."""
        def view(with_left):
            img = scene(left=with_left)
            yt = 0.42 * H
            u = lambda y: (H - y) / (H - yt)                    # noqa: E731
            img[:, 150:] = MAT                                  # move the right line to the middle ...
            stripe(img, lambda y: 0.52 * W - 10 * u(y), lambda y: 20 - 12 * u(y), yt)
            stripe(img, lambda y: 0.95 * W - 60 * u(y), lambda y: 16 - 10 * u(y), yt)   # ... junk far right
            return img
        self.cfg.lane_width_ratio = 0.25
        for _ in range(3):
            d = self.det.detect(view(False))
        if d.selected is not None:                              # it did start on the wrong line
            self.assertGreater(d.selected.near_x, 0.45 * W)
        for _ in range(self.cfg.flip_confirm_frames + 2):
            d = self.det.detect(view(True))
        self.assertEqual(d.source, "left")
        self.assertLess(d.selected.near_x, 0.45 * W)
        self.assertAlmostEqual(d.right_selected.near_x, 0.52 * W + 10, delta=8)

    def test_sure_only_after_both_lines_were_seen_together(self):
        d = self.det.detect(self.right_only(0.0))
        self.assertTrue(d.found)
        self.assertFalse(d.sure, "a line picked up alone could be either boundary")
        self.det.reset()
        for _ in range(2):
            d = self.det.detect(scene())
        self.assertTrue(d.sure)
        for _ in range(30):                                     # left line leaves the view in a bend
            d = self.det.detect(scene(left=False))
        self.assertEqual(d.source, "right")
        self.assertTrue(d.sure, "tracked without a break since both were visible")


class TestCarpetRun(unittest.TestCase):
    """Recorded frames where the earlier detector failed (see the module docstring)."""

    def _run(self, files, warm=()):
        det = LeftLineDetector(Config())
        for f in warm:
            det.detect(cv2.imread(f))
        return [det.detect(cv2.imread(f)) for f in files]

    def test_fixture_present(self):
        self.assertEqual(len(carpet(296, 345)), 50)
        self.assertEqual(len(carpet(1124, 1149)), 26)

    def test_glare_broken_right_line_is_followed_through_the_bend(self):
        res = self._run(carpet(296, 345))
        self.assertGreaterEqual(sum(d.found for d in res), len(res) - 1)
        for d in res:
            if d.found:
                self.assertLess(d.x_ratio, 0.12, "left bend: the lane is on the left of the view")
                if d.right_selected is not None:
                    self.assertGreater(d.right_selected.near_x, 0.6 * W)
        xs = [d.x_ratio for d in res if d.found]
        self.assertLess(float(np.max(np.abs(np.diff(xs)))), 0.08, "no teleporting")

    def test_right_line_under_the_car_is_not_taken_for_the_left_line(self):
        files = carpet(1124, 1149)
        res = self._run(files)
        for f, d in zip(files, res):
            if d.found:
                # steering error must say "the lane is to the left", never "to the right"
                self.assertLess(d.x_ratio, 0.20, os.path.basename(f))
            if d.selected is not None:
                self.assertLess(d.selected.near_x, 0.25 * W, os.path.basename(f))


if __name__ == "__main__":
    unittest.main()
