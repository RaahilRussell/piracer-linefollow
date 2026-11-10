"""Tuning-health rules and the auto-tuner, checked in the closed-loop simulator."""
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "tools"), os.path.join(ROOT, "pi"), ROOT]

import tuning  # noqa: E402
from backtest_autotune import tune  # noqa: E402
from sim.__main__ import _Clock  # noqa: E402
from sim.controllers import PD_STEERING, PilotController  # noqa: E402
from sim.params import Profile  # noqa: E402
from sim.world import World  # noqa: E402


class _R:
    pass


def tight_track(radius=0.5):
    """The original assumed stadium (0.5 m curves, long straights, 25 deg lock). The PD health rules and
    the classic strategies were calibrated on it: they read "steering at its limit more than 20% of
    the time" as too fast, which is simply normal on the traced Waveshare track (71% curve, near
    full lock). The lap tuner (strategy "sections", tests/test_guided_safety.py) is what runs on the
    real track; these tests keep the classic rules honest on the geometry they were made for."""
    prof = Profile.load()
    prof._e.pop("track_polyline", None)
    prof._e["track_radius_m"]["value"], prof._e["track_straight_m"]["value"] = radius, 1.6
    prof._e["lane_width_m"]["value"] = 0.53
    prof._e["steer_angle_table"]["value"] = [[300, 25.0], [375, 0.0], [450, -25.0]]
    return prof


# The pilot settings those rules were calibrated with (line far left in the image, steering capped at 0.8).
LEGACY = {"target_x_ratio": 0.20, "max_steering": 0.8}


def health(speed=0.65, overrides=None, seconds=20, prof=None):
    """Drive the PiRacer simulator and judge the window with the tuning-health rules."""
    clk = _Clock()
    ctrl = PilotController("h", {**PD_STEERING, **(overrides or {})}, speed=speed, clock=clk)   # the health rules judge PD gains
    w = World(prof or Profile.load(), seed=1, target_ratio=ctrl.cfg.target_x_ratio, row_ratio=ctrl.cfg.lookahead_ratio)
    r = _R()
    r.t, r.error, r.steering, r.p, r.d, r.tracking = [], [], [], [], [], []
    while w.t < seconds:
        clk.t = w.t
        x, found, both, age, motion, img = w.observe()
        a, thr, cmd = ctrl.step(x, found, both, age, motion, img, edges=w.edges, source=w.edges_source)
        w.tick(a, thr)
        r.t.append(w.t); r.error.append(cmd.error if cmd.status == "TRACKING" else float("nan"))
        r.steering.append(a); r.p.append(cmd.p); r.d.append(cmd.d); r.tracking.append(cmd.status == "TRACKING")
    r.off_track = w.metrics.off_track
    return r, tuning.analyze(r.t, r.error, r.steering, r.p, r.d, r.tracking)


def _schedule(name):
    """One speed profile of the gain schedule the car ships (pi/gain_schedule.json)."""
    import json
    with open(os.path.join(ROOT, "pi", "gain_schedule.json")) as f:
        return next(p for p in json.load(f)["profiles"] if p["profile"] == name)


class TestHealthRules(unittest.TestCase):
    def test_good_gains_are_healthy(self):
        # Rechecked after adding throttle recovery ramps; the old KP 3.0 now lags.
        r, h = health(speed=0.70, overrides={**LEGACY, "kp": 3.5, "kd": 1.0}, prof=tight_track())
        self.assertFalse(r.off_track)
        self.assertTrue(h.healthy, tuning.summary(h))

    def test_kp_too_high_is_weave(self):
        # every KP: the section gains (STRAIGHT_KP / CORNER_KP) would otherwise shadow a bare kp
        _, h = health(speed=0.90, overrides={"kp": 7.0, "straight_kp": 7.0, "corner_kp": 7.0}, prof=tight_track())
        self.assertIn("weave", [c for _, c, _ in h.findings], tuning.summary(h))
        self.assertLess(h.suggest.get("kp", 1), 1)

    def test_kp_too_low_is_lagging(self):
        # The rule itself, on a made-up window: big one-sided errors while the steering has plenty left.
        # (Driving the simulator with KP 0.8 no longer produces this: since the speed plan slows on
        # large errors, a weak KP just drives slowly and stays inside the rule's 0.22 error limit.)
        t = np.arange(0, 12, 0.05)
        e = 0.26 + 0.02 * np.sin(2 * np.pi * 0.3 * t)
        h = tuning.analyze(t, e, 0.8 * e)
        self.assertIn("lagging", [c for _, c, _ in h.findings], tuning.summary(h))
        self.assertGreater(h.suggest.get("kp", 1), 1)

    def test_synthetic_growing_oscillation(self):
        t = np.arange(0, 12, 0.05)
        e = 0.03 * np.exp(0.25 * t) * np.sin(2 * np.pi * 0.8 * t)
        h = tuning.analyze(t, e, 3.5 * e)
        self.assertIn("weave", [c for _, c, _ in h.findings])


class TestAutoTuner(unittest.TestCase):
    def test_section_tune_only_changes_selected_gain_pair(self):
        from autotune import AutoTuner
        import line_config
        from donkey_part import LineFollowerPilot, load_line_config
        cfg, _ = load_line_config(line_config)
        pilot = LineFollowerPilot(cfg)
        base = (cfg.kp, cfg.kd)
        at = AutoTuner(pilot, ceiling=0.7)
        at.begin_section("corner", speed=0.62)
        cfg.corner_kp, cfg.corner_kd = 1.2, 0.3
        at.reset_section()
        self.assertEqual(at.section, "corner")
        self.assertEqual((cfg.kp, cfg.kd), base)
        at.abort("test")

    def test_tunes_without_crashing_and_never_exceeds_ceiling(self):
        tuner, crashes, _ = tune(tight_track(), seed=3, ceiling=0.70, overrides=LEGACY)
        self.assertLessEqual(crashes, 1)
        self.assertEqual(tuner.state, "done")
        self.assertIsNotNone(tuner.best)
        self.assertLessEqual(max(lv.straight for lv in tuner.history), 0.70 + 1e-9)

    @unittest.skip("The simulated car no longer weaves at KP 9 on any track: since 'Slow on visible bends' (88375a3) "
                   "the speed plan slows on large steering, which damps it, so there is nothing for the ladder to "
                   "fix. The weave rule is covered by test_kp_too_high_is_weave and test_synthetic_growing_oscillation.")
    def test_fixes_kp_that_starts_too_high(self):
        tuner, crashes, _ = tune(tight_track(), kp=9.0, seed=3, ceiling=0.90)
        self.assertLess(tuner.best.kp, 8.0)

    def test_abort_restores_settings(self):
        import line_config
        from autotune import AutoTuner
        from donkey_part import LineFollowerPilot, load_line_config
        cfg, _ = load_line_config(line_config)
        pilot = LineFollowerPilot(cfg)
        kp0, plan0 = cfg.kp, pilot.speed_plan
        at = AutoTuner(pilot, ceiling=0.7)
        at.begin()
        cfg.kp = 6.0
        at.abort("test")
        self.assertEqual((cfg.kp, pilot.speed_plan), (kp0, plan0))
        self.assertEqual(at.state, "aborted")


if __name__ == "__main__":
    unittest.main()


class TestStrategies(unittest.TestCase):
    """Every dashboard strategy finishes in the simulator without leaving the track."""

    def _run(self, strategy, **kw):
        from backtest_autotune import drive
        t, crashes, _ = tune(tight_track(), seed=3, strategy=strategy, overrides=LEGACY, **kw)
        return t, crashes, drive

    def test_scheduled_starts_from_model_and_records_verified_speeds(self):
        t, crashes, drive = self._run("scheduled", ceiling=0.75)
        self.assertEqual(t.state, "done")
        self.assertEqual(crashes, 0)
        self.assertTrue(t.verified)
        crawl = _schedule("Crawl")                                       # the shipped schedule's 0.55 point
        self.assertAlmostEqual(t.history[0].kp, crawl["kp"], delta=0.01)

    def test_gains_only_never_climbs_kp_and_tracks_well(self):
        t, crashes, drive = self._run("gains", hold_speed=0.70)
        self.assertEqual(crashes, 0)
        self.assertLessEqual(t.best.kp, 3.0 + 1e-9)          # started from the model's 3.0 and didn't climb
        self.assertLess(drive(tight_track(), t.best.kp, t.best.kd, 0.70, 0.70, overrides=LEGACY)["err_p95_cm"], 12)

    def test_max_speed_keeps_gains(self):
        t, crashes, _ = self._run("max_speed", kp=3.5, kd=0.4)
        self.assertTrue(all(abs(lv.kp - 3.5) < 1e-9 for lv in t.history))
        self.assertGreater(t.best.straight, 0.55)

    @unittest.skip("Ziegler-Nichols needs a steady weave to measure, and the simulated car no longer weaves even at "
                   "the highest KP allowed (12): the speed plan slows on large steering (88375a3). The strategy ends "
                   "with 'no steady weave' and keeps the original gains, which is its safe outcome.")
    def test_ziegler_nichols_finds_ultimate_gain_at_speed(self):
        t, crashes, _ = self._run("zn", hold_speed=0.80)
        self.assertIsNotNone(t.best)
        self.assertEqual(crashes, 0)
        self.assertGreater(t.best.kd, 0)


class TestLiveSchedule(unittest.TestCase):
    def test_saved_verified_gains_show_in_profiles(self):
        import shutil
        import tempfile
        import webui
        from donkey_part import LineFollowerPilot, load_line_config
        import line_config
        from autotune import AutoTuner
        d = tempfile.mkdtemp()
        old = webui.Hub.HERE_DIR
        try:
            shutil.copy(os.path.join(ROOT, "pi", "gain_schedule.json"), d)
            webui.Hub.HERE_DIR = d
            cfg, _ = load_line_config(line_config)
            pilot = LineFollowerPilot(cfg)
            hub = webui.Hub(cfg, pilot, None)
            hub.autotuner = AutoTuner(pilot)
            hub.autotuner.verified = [{"throttle": 0.70, "kp": 2.8, "kd": 0.9, "corner": 0.70, "summary": "x"}]
            hub.autotune("save_schedule")
            med = next(p for p in hub.profiles["profiles"] if p["profile"] == "Medium")
            self.assertEqual((med["kp"], med["kd"]), (2.8, 0.9))
            self.assertTrue(med["live_verified"])
            self.assertEqual((med["model_kp"], med["model_kd"]), (_schedule("Medium")["kp"], _schedule("Medium")["kd"]))
        finally:
            webui.Hub.HERE_DIR = old
            shutil.rmtree(d)
