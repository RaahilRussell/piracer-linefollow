"""Tests for pi/donkey_part.py (the on-car Donkeycar parts). Runs on the Mac."""
import glob
import os
import sys
import types
import unittest

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "pi"))
sys.path.insert(0, ROOT)

import line_config  # noqa: E402
from donkey_part import LineDetectorPart, LineFollowerPilot, load_line_config  # noqa: E402
from linetrack.detector import LeftLineDetector  # noqa: E402

STAGE = sorted(glob.glob(os.path.join(ROOT, "tests", "frames", "stage", "*.jpg")))
LIVE = sorted(glob.glob(os.path.join(ROOT, "tests", "frames", "live_*.jpg")))


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def rgb(path):
    return cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2RGB)


class TestLoadConfig(unittest.TestCase):
    def test_defaults_from_line_config(self):
        cfg, extras = load_line_config(line_config)
        self.assertAlmostEqual(cfg.target_x_ratio, 0.20)
        self.assertAlmostEqual(cfg.throttle, min(line_config.LINE_THROTTLE, line_config.MAX_THROTTLE))
        self.assertTrue(extras["SHOW_OVERLAY"])

    def test_throttle_capped(self):
        m = types.SimpleNamespace(LINE_THROTTLE=0.9, MAX_THROTTLE=0.3, KP="1.5")
        cfg, _ = load_line_config(m)
        self.assertAlmostEqual(cfg.throttle, 0.3)
        self.assertAlmostEqual(cfg.kp, 1.5)


class TestParts(unittest.TestCase):
    def setUp(self):
        self.cfg, _ = load_line_config(line_config)
        self.clock = Clock()
        self.det = LineDetectorPart(self.cfg, clock=self.clock)
        self.pilot = LineFollowerPilot(self.cfg, clock=self.clock)
        self.det.pilot = self.pilot

    def step(self, img, mode, dt=0.05):
        self.clock.t += dt
        x, found, age, overlay, motion, both = self.det.run(img)
        return (x, found, age, overlay), self.pilot.run(mode, x, found, age, motion, both)

    def test_rgb_input_tracks_real_frames(self):
        # camera delivers RGB; feeding it as-is must still find orange
        (x, found, _, overlay), _ = self.step(rgb(LIVE[0]), "local_angle")
        self.assertTrue(found)
        self.assertLess(x, 0.15)  # the small left fragment, not the centre stripe at ~0.42
        self.assertEqual(overlay.shape, rgb(LIVE[0]).shape)

    def test_throttle_only_in_local_while_tracking(self):
        img = rgb(LIVE[0])
        _, (_, thr) = self.step(img, "local_angle")
        self.assertEqual(thr, 0.0)
        self.clock.t += 0.01
        _, (_, thr) = self.step(np.roll(img, 1, axis=0), "local")
        self.assertAlmostEqual(thr, self.cfg.throttle)
        blank = np.full_like(img, 120)
        _, (_, thr) = self.step(blank, "local")
        self.assertEqual(thr, 0.0)

    def test_stale_frame_zeroes_throttle(self):
        img = rgb(LIVE[0])
        _, (_, thr) = self.step(img, "local")
        self.assertGreater(thr, 0.0)
        # same frame repeated (camera stalled) for longer than max_frame_age_s
        for _ in range(10):
            (_, _, age, _), (_, thr) = self.step(img, "local")
        self.assertGreater(age, self.cfg.max_frame_age_s)
        self.assertEqual(thr, 0.0)
        self.assertEqual(self.pilot.last_cmd.status, "STALE")

    def test_none_image_is_safe(self):
        (x, found, age, ov), (angle, thr) = self.step(None, "local")
        self.assertFalse(found)
        self.assertEqual(thr, 0.0)
        self.assertEqual(angle, 0.0)

    def test_reset_after_mode_gap(self):
        img = rgb(LIVE[0])
        for i in range(5):
            self.step(np.roll(img, i, axis=1), "local_angle")
        self.assertNotEqual(self.pilot.ctrl.angle, 0.0)
        self.clock.t += 2.0  # pilot not called (user mode)
        self.assertIsNone(self.pilot.active_cmd())
        blank = np.full_like(img, 120)
        _, (angle, _) = self.step(blank, "local_angle")
        self.assertEqual(angle, 0.0)  # fresh state: no stale steering held

    def test_stage_tracking_rate(self):
        n = 0
        for f in STAGE:
            (_, found, _, _), _ = self.step(rgb(f), "local_angle", dt=0.2)
            n += found
        self.assertGreaterEqual(n / len(STAGE), 0.85)


class TestThrottleCapAndRecorder(unittest.TestCase):
    def test_cap_limits_forward_only(self):
        from donkey_part import ThrottleCap
        cap = ThrottleCap(line_config.TOP_THROTTLE)
        self.assertAlmostEqual(cap.run(1.0), line_config.TOP_THROTTLE)
        self.assertAlmostEqual(cap.run(0.1), 0.1)
        self.assertEqual(cap.run(-1.0), -1.0)   # E-stop braking pulse untouched
        self.assertEqual(cap.run(None), 0.0)

    def test_turn_slowdown(self):
        from donkey_part import ThrottleCap
        cap = ThrottleCap(0.35, 0.6, 0.4, 0.8)
        self.assertAlmostEqual(cap.run(1.0, 0.0, "local_angle"), 0.35)       # straight: just the cap
        self.assertAlmostEqual(cap.run(1.0, 0.4, "local_angle"), 0.35 * 0.7)  # half turn
        self.assertAlmostEqual(cap.run(1.0, -0.8, "local"), 0.35 * 0.4)       # full turn, either side
        self.assertAlmostEqual(cap.run(0.2, 0.8, "local"), 0.2 * 0.4)         # pilot throttle scaled too
        self.assertAlmostEqual(cap.run(1.0, 0.8, "user"), 0.35)               # user mode: cap only
        self.assertEqual(cap.run(-1.0, 0.8, "local"), -1.0)                   # E-stop brake untouched

    def test_recorder_writes_frames_and_csv(self):
        import tempfile
        from donkey_part import LineRecorder
        cfg, _ = load_line_config(line_config)
        det = LineDetectorPart(cfg)
        img = rgb(LIVE[0])
        x, found, age, _, _, _ = det.run(img)
        with tempfile.TemporaryDirectory() as d:
            rec = LineRecorder(d, det)
            rec.run(img, "local_angle", x, found, age, 0.3, 0.1)
            rec.run(img, "local_angle", None, False, age, 0.0, 0.0)
            rec.shutdown()
            sub = os.path.join(d, os.listdir(d)[0])
            self.assertEqual(sorted(os.listdir(sub)), ["000000.jpg", "000001.jpg", "log.csv"])
            lines = open(os.path.join(sub, "log.csv")).read().splitlines()
            self.assertEqual(len(lines), 3)
            self.assertIn(",left,", lines[1])
            back = cv2.imread(os.path.join(sub, "000000.jpg"))
            # saved as a normal BGR jpg: replays through the detector the same way
            self.assertTrue(LeftLineDetector(cfg).detect(back).found)


class TestAutoCrawlAndSingleLine(unittest.TestCase):
    def test_crawl_ramps_until_moving_then_holds(self):
        from donkey_part import AutoCrawl
        c = AutoCrawl(start=0.40, lo=0.35, hi=0.60, ramp_up=0.10, move_thresh=2.5, fast_thresh=6.0)
        for _ in range(20):                 # 1 s still -> +0.09 (needs 3 readings first)
            t = c.update(0.9, 0.05)
        self.assertAlmostEqual(t, 0.49, places=2)
        for _ in range(20):                 # rolling slowly -> hold
            t2 = c.update(4.0, 0.05)
        self.assertAlmostEqual(t2, t, delta=0.011)  # median needs 1-2 frames to see the change
        for _ in range(200):                # too fast -> eases off, never below lo
            t3 = c.update(15.0, 0.05)
        self.assertAlmostEqual(t3, 0.35)
        for _ in range(400):                # never above hi
            t4 = c.update(0.9, 0.05)
        self.assertAlmostEqual(t4, 0.60)

    def _pilot(self, clock, stop_s=2.5):
        from donkey_part import AutoCrawl
        cfg, _ = load_line_config(line_config)
        return LineFollowerPilot(cfg, clock=clock, crawl=AutoCrawl(), single_line_stop_s=stop_s)

    def test_single_line_parked_still_ramps_up(self):
        clock = Clock(); p = self._pilot(clock)
        thrs = []
        for _ in range(80):                 # 4 s parked, one line: must still try to get going
            clock.t += 0.05; _, thr = p.run("local", 0.2, True, 0.0, 0.9, False)
            thrs.append(thr)
        self.assertGreater(thrs[-1], thrs[5])
        self.assertGreater(thrs[-1], 0.0)   # timer doesn't run while parked

    def test_single_line_while_rolling_holds_then_stops(self):
        clock = Clock(); p = self._pilot(clock)
        for _ in range(10):                 # rolling with both lines
            clock.t += 0.05; _, thr = p.run("local", 0.2, True, 0.0, 4.0, True)
        held = thr
        for _ in range(20):                 # 1 s rolling on a single line: no speed-up
            clock.t += 0.05; _, thr = p.run("local", 0.2, True, 0.0, 4.0, False)
        self.assertAlmostEqual(thr, held, places=3)
        for _ in range(40):                 # > 2.5 s rolling on a single line: stop
            clock.t += 0.05; _, thr = p.run("local", 0.2, True, 0.0, 4.0, False)
        self.assertEqual(thr, 0.0)
        clock.t += 0.05; _, thr = p.run("local", 0.2, True, 0.0, 4.0, True)
        self.assertGreater(thr, 0.0)        # both lines again -> resumes

    def test_local_angle_never_pilot_throttle(self):
        clock = Clock(); p = self._pilot(clock)
        for _ in range(10):
            clock.t += 0.05; _, thr = p.run("local_angle", 0.2, True, 0.0, 0.9, True)
            self.assertEqual(thr, 0.0)


class TestBacktestedFixes(unittest.TestCase):
    """The two changes the backtest showed help, on the real frames where they matter."""

    def _picks(self, swap):
        from linetrack.detector import LeftLineDetector
        cfg, _ = load_line_config(line_config)
        cfg.left_lean_swap = swap
        det = LeftLineDetector(cfg)
        picks = {}
        for f in sorted(glob.glob(os.path.join(ROOT, "tests", "frames", "wrongline", "*.jpg"))):
            d = det.detect(cv2.imread(f))
            if d.selected is not None:  # the stripe it believes is the left line (even if it then
                picks[os.path.basename(f)] = d.selected.near_x / (d.width / d.scale)  # steers off the right line)
        return picks

    def test_left_lean_swap_fixes_wrong_line_lock(self):
        # run 20261108-185226: from ~1495 the old tracker locked onto the right boundary (x~0.4)
        # while the real left line sat at the far left (x<0.05). Hand-labelled frames:
        for name, truth in {"001500.jpg": 0.021, "001520.jpg": 0.010, "001540.jpg": 0.034}.items():
            self.assertGreater(abs(self._picks(False).get(name, 9) - truth), 0.07)   # old: wrong
            self.assertLess(abs(self._picks(True)[name] - truth), 0.07)              # new: right

    def test_coast_then_slow_then_stop_holding_heading(self):
        from donkey_part import AutoCrawl
        cfg, _ = load_line_config(line_config)
        cfg.lost_hold_heading = True
        clock = Clock()
        p = LineFollowerPilot(cfg, clock=clock, crawl=AutoCrawl(), coast_s=0.5, slow_s=0.5)
        for _ in range(20):                                   # tracking, line to the right -> steer right
            clock.t += 0.05; angle, thr = p.run("local", 0.35, True, 0.0, 4.0, True)
        self.assertGreater(thr, 0); self.assertGreater(angle, 0)
        held, seen = angle, []
        for _ in range(30):                                   # 1.5 s blind
            clock.t += 0.05; a, t = p.run("local", None, False, 0.0, 4.0, False)
            seen.append((a, t))
        self.assertAlmostEqual(seen[5][1], thr, places=3)     # 0.3 s: still coasting at speed
        self.assertTrue(0 < seen[13][1] < thr)                # 0.7 s: slowing
        self.assertEqual(seen[-1][1], 0.0)                    # 1.5 s: stopped
        self.assertTrue(all(abs(a - held) < 1e-6 for a, _ in seen))  # heading held, never re-centred

    def test_camera_freeze_still_cuts_throttle_at_once(self):
        from donkey_part import AutoCrawl
        cfg, _ = load_line_config(line_config)
        clock = Clock()
        p = LineFollowerPilot(cfg, clock=clock, crawl=AutoCrawl(), coast_s=0.5, slow_s=0.5)
        for _ in range(10):
            clock.t += 0.05; p.run("local", 0.2, True, 0.0, 4.0, True)
        clock.t += 0.05
        _, thr = p.run("local", 0.2, True, 5.0, 4.0, True)   # frame 5 s old = STALE
        self.assertEqual(thr, 0.0)


class TestKillSwitch(unittest.TestCase):
    def test_x_zeroes_then_quits(self):
        from donkey_part import KillSwitch
        clock = Clock()
        k = KillSwitch(hold_s=0.5, clock=clock)
        self.assertEqual(k.run(0.3, 0.2), (0.3, 0.2))
        k.trigger()
        self.assertEqual(k.run(0.3, 0.2), (0.0, 0.0))
        clock.t += 0.6
        with self.assertRaises(KeyboardInterrupt):
            k.run(0.3, 0.2)


if __name__ == "__main__":
    unittest.main()
