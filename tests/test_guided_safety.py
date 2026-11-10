"""Regression tests for guided trials, geometry failure and manual authority."""
import os
import sys
import unittest
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "pi"), os.path.join(ROOT, "tools"), ROOT]
from autotune import AutoTuner
from donkey_part import LineFollowerPilot, SpeedPlan
from linetrack.config import Config
from linetrack.geometry import Bend
from webui import ManualOverridePart, AutoMotionGuard
import tuning


class GuidedTrials(unittest.TestCase):
    def setUp(self):
        self.t = 0.0
        self.pilot = LineFollowerPilot(Config(max_throttle=.8), clock=lambda: self.t,
                                       speed_plan=SpeedPlan(.75, .55, transition=.61))
        self.at = AutoTuner(self.pilot, clock=lambda: self.t, ceiling=.8)
        self.cmd = SimpleNamespace(error=0.0, angle=0.0, p=0.0, d=0.0)

    def samples(self, section, n=45):
        for _ in range(n):
            self.t += .05
            self.at.update("local", self.cmd, True, section)

    def test_too_few_samples_never_healthy(self):
        self.assertFalse(tuning.analyze([0, 10], [0, 0], [0, 0]).healthy)

    def test_single_boundary_pass_stops_pilot(self):
        self.at.begin_section("straight", .6)
        self.samples("straight")
        self.samples("corner", 1)
        self.assertEqual(self.at.result, "passed")
        self.assertEqual(self.pilot.run("local", .2, True, 0)[1], 0)

    def test_short_boundary_is_insufficient_and_stops(self):
        self.at.begin_section("straight", .6)
        self.samples("straight", 6)
        self.samples("corner", 1)
        self.assertEqual(self.at.result, "insufficient")
        self.assertTrue(self.pilot.motion_stopped)

    def test_both_preserves_plan_and_accepts_separate_segments(self):
        self.at.begin_sections()
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner,
                          self.pilot.speed_plan.transition), (.75, .55, .61))
        self.samples("straight")
        self.samples("corner")
        self.samples("straight", 1)
        self.assertEqual(self.at.result, "passed")
        self.assertTrue(self.pilot.motion_stopped)
        shared = (self.pilot.cfg.kp, self.pilot.cfg.kd)
        self.at.apply_best()
        self.assertEqual(shared, (self.pilot.cfg.kp, self.pilot.cfg.kd))

    def test_sparse_data_cannot_pass(self):
        self.at.begin_sections()
        self.samples("straight", 1)
        self.t += 11
        self.samples("straight", 1)
        self.samples("corner", 1)
        self.assertNotEqual(self.at.result, "passed")

    def test_tracking_loss_in_either_section_stops(self):
        self.at.begin_sections()
        self.at.update("local", self.cmd, False, "unknown")
        self.assertEqual(self.at.result, "failed")
        self.assertTrue(self.pilot.motion_stopped)

    def test_manual_end_stops_without_claiming_pass(self):
        self.at.begin_section("corner", .55)
        self.at.finish_section()
        self.assertEqual(self.at.result, "insufficient")
        self.assertTrue(self.pilot.motion_stopped)

    def test_unknown_geometry_caps_speed_and_is_not_straight(self):
        from donkey_part import AutoCrawl
        self.pilot.crawl = AutoCrawl(start=.7, hi=.8)
        _, thr = self.pilot.run("local", .2, True, 0)
        self.assertEqual(self.pilot.last_cmd.section, "unknown")
        self.assertLessEqual(thr, .55)

    def test_section_requires_stable_evidence(self):
        p = self.pilot
        self.assertEqual(p._confirmed_section(0, 0), "unknown")
        self.assertEqual(p._confirmed_section(0, .2), "straight")
        self.assertEqual(p._confirmed_section(2, .25), "straight")
        self.assertEqual(p._confirmed_section(0, .3), "straight")
        self.assertEqual(p._confirmed_section(None, .4), "unknown")

    def test_gamepad_reverse_overrides_auto_and_stays_manual(self):
        controller = SimpleNamespace(mode="local")
        part = ManualOverridePart(self.pilot, controller)
        self.assertEqual(part.run("local", .2, -.6), ("user", .2, -.6))
        self.assertEqual(controller.mode, "user")
        self.assertEqual(part.run("user", 0, 0)[0], "user")

    def test_completion_is_latched_until_explicit_rearm(self):
        controller = SimpleNamespace(mode="local")
        part = ManualOverridePart(self.pilot, controller)
        part.run("local", 0, 0)
        self.pilot.stop_motion()
        self.assertEqual(part.run("local", 0, 0)[0], "user")
        part.run("user", 0, 0)
        part.run("local", 0, 0)
        self.assertFalse(self.pilot.motion_stopped)

    def test_final_gate_zeroes_same_loop_and_allows_manual_reverse(self):
        guard = AutoMotionGuard(self.pilot)
        self.pilot.stop_motion()
        self.assertEqual(guard.run("local", .7), 0.0)
        self.assertEqual(guard.run("user", -.5), -.5)
        part = ManualOverridePart(self.pilot, SimpleNamespace(mode="user"))
        self.assertEqual(part.run("user", 0, .7), ("user", 0., 0.))
        part.run("user", 0, 0)
        self.assertEqual(part.run("user", 0, -.5), ("user", 0., -.5))

    def test_unknown_geometry_does_not_pass_single_section(self):
        self.at.begin_section("straight", .6)
        self.samples("straight")
        self.samples("unknown", 1)
        self.assertEqual(self.at.result, "insufficient")

    def test_attempt_limit_stops_continuous_retuning(self):
        from unittest.mock import patch
        bad = tuning.Health(sufficient=True, n=40, seconds=2.,
                            findings=[("warn", "weave", "weave")], suggest={"kp": .9})
        self.at.begin_sections()
        self.at._segment = "straight"
        self.at._t = [0., 1.]
        with patch("tuning.analyze", return_value=bad):
            for _ in range(4):
                self.at._guided_judge()
        self.assertEqual(self.at.result, "failed")
        self.assertTrue(self.pilot.motion_stopped)

    def test_gain_change_invalidates_previous_section_pass(self):
        from unittest.mock import patch
        self.at.begin_sections()
        self.samples("straight")
        self.samples("corner", 1)
        self.assertEqual(self.at._section_results["straight"], "passed")
        bad = tuning.Health(sufficient=True, n=40, seconds=2.,
                            findings=[("warn", "weave", "weave")], suggest={"kp": .9})
        with patch("tuning.analyze", return_value=bad):
            self.at._guided_judge()
        self.assertEqual(self.at._section_results["straight"], "pending")

    def test_replay_wires_geometry_and_plan(self):
        from backtest import build
        cfg, clk, det, pilot = build({"speed_straight": .6, "speed_corner": .55})
        det.bend = Bend(1, 1, .15, .4, 1., 10, 0.)
        self.assertIs(pilot.bend_source(), det.bend)
        self.assertEqual(pilot.speed_plan.corner, .55)

    def test_small_straight_corrections_leave_speed_available(self):
        p = SpeedPlan(.75, .55, transition=.65)
        self.assertEqual(p.target(.08, .8, 0., .02), .75)

    def test_large_straight_recovery_requests_corner_speed(self):
        p = SpeedPlan(.75, .55, transition=.65)
        self.assertAlmostEqual(p.target(.7, .8, 0., 0.), .55)
        self.assertAlmostEqual(p.target(.0, .8, 0., .25), .55)

    def test_corner_target_cannot_exceed_straight(self):
        p = SpeedPlan(.6, .8, transition=.9)
        self.assertEqual((p.straight, p.transition, p.corner), (.6, .6, .6))

    def test_simulator_uses_same_recovery_speed_guard(self):
        from sim.controllers import PilotController
        ctrl = PilotController(speed=(.75, .55), clock=lambda: self.t,
                               overrides={"speed_transition": .6, "speed_ramp_up": .2,
                                          "auto_crawl": False, "pilot_coast_s": 0.})
        self.assertIsInstance(ctrl.pilot, LineFollowerPilot)
        self.assertEqual(ctrl.pilot.speed_plan.transition, .6)
        self.assertEqual(ctrl.pilot.speed_plan.ramp_up, .2)
        self.assertIsNone(ctrl.pilot.crawl)
        self.assertEqual(ctrl.pilot.coast_s, 0.)
        ctrl.pilot.run("local", .5, True, 0., bend=Bend(0., 0., .15, .4, 1., 10, 0.))
        self.assertAlmostEqual(ctrl.pilot.speed_target, .55)


if __name__ == "__main__":
    unittest.main()
