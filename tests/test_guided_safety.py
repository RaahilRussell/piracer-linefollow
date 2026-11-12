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
        self.at = AutoTuner(self.pilot, clock=lambda: self.t, ceiling=.8, confirm_laps=1, margin_steps=0)
        self.cmd = SimpleNamespace(error=0.0, angle=0.0, p=0.0, d=0.0)

    def lose(self, seconds=0.4):
        """The line disappears for longer than the tuner's grace period."""
        for _ in range(int(seconds / .05) + 1):
            self.t += .05
            self.at.update("local", self.cmd, False, "unknown")

    def samples(self, section, n=45):
        for _ in range(n):
            self.t += .05
            self.at.update("local", self.cmd, True, section)

    def test_curve_trial_waits_for_exit_and_restores_split_speeds(self):
        original = self.pilot.speed_plan
        self.at.window_s = 1
        self.at.begin_section("corner")
        self.assertEqual(self.pilot.speed_plan.straight, original.corner)
        self.samples("corner", 56)
        self.assertEqual(self.at.state, "running")
        self.samples("straight", 1)
        self.assertEqual(self.at.result, "passed")
        self.assertIs(self.pilot.speed_plan, original)

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

    def _pass_both_sections(self):
        self.samples("straight")
        self.samples("corner")
        self.samples("straight", 1)

    def test_both_preserves_plan_and_climbs_curve_at_half_step(self):
        self.at.begin_sections(step=.05)
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner,
                          self.pilot.speed_plan.transition), (.75, .55, .61))
        self._pass_both_sections()
        self.assertEqual(self.at.state, "running")
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.8, .58))
        self.assertEqual(self.at._section_results, {"straight": "pending", "corner": "pending"})
        for expected_corner in (.61, .64, .67, .7, .73, .76, .79, .8):
            self._pass_both_sections()
            self.assertEqual(self.at.state, "running")
            self.assertAlmostEqual(self.pilot.speed_plan.straight, .8)
            self.assertAlmostEqual(self.pilot.speed_plan.corner, expected_corner)
        self._pass_both_sections()
        self.assertEqual(self.at.result, "passed")
        self.assertTrue(self.pilot.motion_stopped)
        shared = (self.pilot.cfg.kp, self.pilot.cfg.kd)
        self.at.apply_best()
        self.assertEqual(shared, (self.pilot.cfg.kp, self.pilot.cfg.kd))

    def test_step_failed_button_keeps_last_proven_throttle_and_stops(self):
        self.at.begin_sections(step=.05)
        self._pass_both_sections()
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.8, .58))
        self.at.fail_sections()
        self.assertEqual(self.at.result, "passed")
        self.assertTrue(self.pilot.motion_stopped)
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.75, .55))
        self.assertIn("Driver marked this throttle step failed", self.at.message)

    def test_curve_too_fast_holds_curve_and_keeps_raising_straight(self):
        self.at.ceiling = 1.0
        self.at.begin_sections(step=.05)
        self._pass_both_sections()                          # now .80 / .58
        self.at.fail_sections("corner")
        self.assertEqual(self.at.state, "running")
        self.assertFalse(self.pilot.motion_stopped)
        self.assertEqual(self.at._frozen, {"straight": False, "corner": True})
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.8, .55))
        self._pass_both_sections()
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.85, .55))
        self.at.fail_sections("straight")                   # both held: finished at last pass
        self.assertEqual(self.at.result, "passed")
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.8, .55))
        self.assertTrue(self.pilot.motion_stopped)

    def test_line_lost_in_curve_holds_curve_and_pauses_for_reset(self):
        self.at.ceiling = 1.0
        self.at.begin_sections(step=.05)
        self._pass_both_sections()                          # now .80 / .58
        self.samples("corner", 5)
        self.lose()
        self.assertEqual(self.at.state, "paused")
        self.assertTrue(self.pilot.motion_stopped)
        self.assertTrue(self.at._frozen["corner"])
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.8, .55))
        self.at.update("user", self.cmd, False, "unknown")  # latched to manual: waits, no abort
        self.assertEqual(self.at.state, "paused")
        self.pilot.resume_motion()                          # driver re-selects Full Auto
        self.at.update("local", self.cmd, True, "straight")
        self.assertEqual(self.at.state, "running")
        self.assertFalse(self.pilot.motion_stopped)

    def test_off_track_after_curve_holds_curve(self):
        self.at.ceiling = 1.0
        self.at.begin_sections(step=.05)
        self._pass_both_sections()                          # now .80 / .58, last segment straight
        self.samples("corner", 5)
        self.at.report_off_track()
        self.assertTrue(self.at._frozen["corner"])
        self.assertEqual(self.at.state, "running")
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.8, .55))

    def test_exit_hold_keeps_curve_while_still_turning(self):
        p = self.pilot
        p.cfg.corner_exit_steer = .45
        p.section_now = "corner"
        p.last_cmd = SimpleNamespace(angle=.8)
        self.assertEqual(p._confirmed_section(0.0, 1.0), "corner")
        p.last_cmd = SimpleNamespace(angle=.2)
        p._confirmed_section(0.0, 2.0)
        self.assertEqual(p._confirmed_section(0.0, 2.3), "straight")

    def test_first_lap_line_loss_steps_down_and_pauses(self):
        self.at.begin_sections(step=.05)
        self.samples("corner", 5)
        self.lose()
        self.assertEqual(self.at.state, "paused")
        self.assertTrue(self.pilot.motion_stopped)
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.75, .52))
        self.assertEqual(self.at.status()["laps"][-1]["result"], "curve limit")

    def test_speed_needs_two_clean_laps_before_stepping_up(self):
        self.at.confirm_laps = 2
        self.at.begin_sections(step=.05)
        self._pass_both_sections()
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.75, .55))
        self.assertIn("Clean 1 of 2", self.at.status()["laps"][-1]["change"])
        self._pass_both_sections()
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.8, .58))

    def test_limit_races_one_step_below_last_clean_speed(self):
        self.at.ceiling, self.at.margin_steps = 1.0, 1
        self.at.begin_sections(step=.05)
        self._pass_both_sections()                          # clean at .75/.55, now trying .80/.58
        self.at.fail_sections("corner")
        lim = self.at.status()["limits"]["corner"]
        self.assertEqual((lim["failed_at"], lim["last_clean"], lim["kept"]), (.58, .55, .52))
        self.assertEqual(self.pilot.speed_plan.corner, .52)
        self._pass_both_sections()
        self.assertEqual(self.pilot.speed_plan.corner, .52)     # held with its margin

    def test_single_dropped_frame_is_not_a_line_failure(self):
        self.at.begin_sections(step=.05)
        self.samples("corner", 5)
        self.t += .05
        self.at.update("local", self.cmd, False, "unknown")
        self.samples("corner", 5)
        self.assertEqual(self.at.state, "running")
        self.assertFalse(self.pilot.motion_stopped)

    def test_first_lap_gives_up_after_three_retries(self):
        self.at.begin_sections(step=.05)
        for _ in range(4):
            self.pilot.resume_motion()
            self.at.update("local", self.cmd, True, "corner")
            self.samples("corner", 3)
            self.lose()
        self.assertEqual(self.at.result, "failed")
        self.assertTrue(self.pilot.motion_stopped)
        self.assertEqual((self.pilot.speed_plan.straight, self.pilot.speed_plan.corner), (.75, .55))

    def test_sparse_data_cannot_pass(self):
        self.at.begin_sections()
        self.samples("straight", 1)
        self.t += 11
        self.samples("straight", 1)
        self.samples("corner", 1)
        self.assertNotEqual(self.at.result, "passed")

    def test_tracking_loss_in_either_section_stops(self):
        self.at.begin_sections()
        self.lose()
        self.assertEqual(self.at.state, "paused")
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
        # Gains never settled before any pass: the straight steps down and the trial carries on.
        self.assertEqual(self.at.state, "running")
        self.assertLess(self.pilot.speed_plan.straight, .75)
        self.assertEqual(self.at.status()["laps"][-1]["result"], "straight limit")

    def test_curve_lagging_never_raises_curve_kp(self):
        from unittest.mock import patch
        lag = tuning.Health(sufficient=True, n=60, seconds=3., tracked=1.0,
                            findings=[("warn", "lagging", "lagging")], suggest={"kp": 1.25})
        self.pilot.cfg.corner_kp = 3.5
        self.at.begin_sections()
        self.at._segment = "corner"
        self.at._t = [0., 3.]
        with patch("tuning.analyze", return_value=lag):
            self.at._guided_judge()
        self.assertEqual(self.pilot.cfg.corner_kp, 3.5)
        self.assertEqual(self.at.state, "running")

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
        ctrl.pilot.run("local", ctrl.cfg.target_x_ratio + .25, True, 0., bend=Bend(0., 0., .15, .4, 1., 10, 0.))
        self.assertAlmostEqual(ctrl.pilot.speed_target, .55)


if __name__ == "__main__":
    unittest.main()


class ConfidenceSlowdown(unittest.TestCase):
    """racer-v2: a shaky detection blends throttle down to curve speed (CONF_SLOW_BELOW, off by default)."""

    def drive(self, conf, slow_below):
        t = [0.0]
        pilot = LineFollowerPilot(Config(max_throttle=1.0, conf_slow_below=slow_below, target_x_ratio=0.40),
                                  clock=lambda: t[0], speed_plan=SpeedPlan(.9, .6, transition=.75))
        pilot.bend_source = lambda: Bend(0.0, 0.0, .15, .4, 1., 10, 0.)      # a straight in view
        pilot.conf_source = lambda: conf
        thr = 0.0
        for _ in range(120):                                                  # 6 s: the plan ramps to straight speed
            t[0] += .05
            _, thr = pilot.run("local", 0.40, True, 0.0, both=True)
        return thr, pilot

    def test_off_by_default(self):
        self.assertEqual(Config().conf_slow_below, 0.0)
        self.assertAlmostEqual(self.drive(0.36, 0.0)[0], self.drive(0.85, 0.0)[0])

    def test_confident_detection_keeps_straight_speed(self):
        thr, pilot = self.drive(0.85, 0.6)
        self.assertGreater(thr, .6 + .1)
        self.assertIsNone(pilot.conf_cap)

    def test_shaky_detection_drops_to_curve_speed_never_below(self):
        full = self.drive(0.85, 0.6)[0]
        low, pilot = self.drive(0.35, 0.6)
        self.assertAlmostEqual(low, .6, places=3)                             # at min_confidence: curve speed
        mid = self.drive(0.475, 0.6)[0]                                       # half way: half way
        self.assertAlmostEqual(mid, (.6 + full) / 2, places=2)


class TunerRulesPersist(unittest.TestCase):
    def test_rules_saved_on_car_survive_restart(self):
        import tempfile
        import line_config
        import webui
        from donkey_part import load_line_config
        path = os.path.join(tempfile.mkdtemp(), "tuning.json")

        def boot():
            cfg, _ = load_line_config(line_config)
            pilot = LineFollowerPilot(cfg)
            hub = webui.Hub(cfg, pilot, None, tuning_path=path)
            hub.load_saved()                       # drive_line.py order: saved file, then the tuner
            hub.attach_autotuner(AutoTuner(pilot))
            return hub
        hub = boot()
        hub.apply({"tune_margin_steps": 2, "tune_confirm_laps": 3, "tune_curve_sat": 0.9})
        hub.save()
        at = boot().autotuner
        self.assertEqual((at.margin_steps, at.confirm_laps, at.curve_sat), (2, 3, 0.9))


class LapTimerCounts(unittest.TestCase):
    LAP = ["straight"] * 20 + ["corner"] * 20 + ["straight"] * 20 + ["corner"] * 20

    def test_four_section_changes_make_a_lap_and_losses_are_counted(self):
        from lap_tuning import LapTimer
        timer, t = LapTimer(sections_per_lap=4, race_laps=3), 0.0
        for n in range(5):
            for i, sec in enumerate(self.LAP):
                t += .05
                timer.update(t, "local", sec, not (n == 2 and i == 30))
        st = timer.status(t)
        self.assertEqual([l["seconds"] for l in st["laps"]], [4.0] * 4)
        self.assertEqual([l["lost"] for l in st["laps"]], [0, 0, 1, 0])
        self.assertEqual(st["race_total"], 12.0)
        self.assertEqual(st["race_total_with_penalties"], 17.0)       # one line loss = one off-track = +5 s

    def test_leaving_full_auto_does_not_count_a_lap(self):
        from lap_tuning import LapTimer
        timer = LapTimer()
        timer.update(0, "local", "straight", True)
        timer.update(1, "user", "corner", True)
        self.assertIsNone(timer.status(1)["current"])

    def test_brief_flicker_is_not_a_section_change(self):
        from lap_tuning import LapTimer
        timer, t = LapTimer(sections_per_lap=4, debounce_s=.25), 0.0
        lap = list(self.LAP)
        lap[30:33] = ["straight"] * 3            # 0.15 s of "straight" in the middle of a curve
        lap[50:52] = ["corner"] * 2              # and 0.10 s of "curve" on a straight
        for _ in range(5):
            for sec in lap:
                t += .05
                timer.update(t, "local", sec, True)
        self.assertEqual([l["seconds"] for l in timer.laps], [4.0] * 4)

    def test_a_curve_read_as_unknown_still_counts(self):
        # Real Nov 10 night: with the inner line out of view a whole curve read "unknown"; the timer saw
        # straight -> straight and reported two 3.6 s laps as one 7.2 s lap.
        from lap_tuning import LapTimer
        timer, t = LapTimer(sections_per_lap=4), 0.0
        lap = list(self.LAP)
        lap[60:80] = ["unknown"] * 20            # the second curve never gets a curvature fit
        for _ in range(5):
            for sec in lap:
                t += .05
                timer.update(t, "local", sec, True)
        self.assertEqual([l["seconds"] for l in timer.laps], [4.0] * 4)

    def test_impossible_lap_time_is_rejected(self):
        from lap_tuning import LapTimer
        timer, t = LapTimer(sections_per_lap=4, debounce_s=0.0, min_lap_s=2.0), 0.0
        lap = list(self.LAP)
        lap[25:35] = ["straight"] * 10           # a long false "straight" inside the first curve: 2 extra changes
        for _ in range(4):
            for sec in lap:
                t += .05
                timer.update(t, "local", sec, True)
        times = [l["seconds"] for l in timer.laps]
        self.assertTrue(times and all(s >= 2.0 for s in times), times)

    def test_stopped_lap_is_dropped_and_counted_as_a_reposition(self):
        from lap_tuning import LapTimer
        timer, t = LapTimer(sections_per_lap=4, race_laps=2), 0.0
        for sec in self.LAP + self.LAP[:30]:
            t += .05
            timer.update(t, "local", sec, True)
        for _ in range(100):                     # 5 s in Manual / stopped: the lap clock must not keep running
            t += .05
            timer.update(t, "user", "straight", True)
        self.assertIsNone(timer.status(t)["current"])
        for _ in range(3):
            for sec in self.LAP:
                t += .05
                timer.update(t, "local", sec, True)
        st = timer.status(t)
        self.assertEqual(max(l["seconds"] for l in st["laps"]), 4.0)
        self.assertEqual((st["race_stops"], st["race_penalty_s"]), (1, 10.0))

    def test_estimate_matches_the_simulated_lap(self):
        from lap_tuning import estimate_lap
        table = ((370, 0.0), (387, 0.0), (390, 0.35), (407, 1.8))
        # traced track 4.98 m, 71% curve; the car rides about 11 cm outside the traced line
        self.assertAlmostEqual(estimate_lap(1.00, 0.92, table, 4.98, 0.71, 0.11), 3.5, delta=0.2)    # simulator: 3.50 s
        self.assertAlmostEqual(estimate_lap(0.70, 0.65, table, 4.98, 0.71, 0.11), 7.9, delta=0.4)    # simulator: 7.90 s
        self.assertEqual(estimate_lap(0.92, 0.92, table, 4.98, 0.71), estimate_lap(0.93, 0.93, table, 4.98, 0.71))  # same PWM tick
        self.assertIsNone(estimate_lap(0.30, 0.30, table, 4.98, 0.71))                             # doesn't move


class RaceValuesFirst(unittest.TestCase):
    """Start: race values on the car, estimate reported, then climb (or step down)."""

    def boot(self, saved=None):
        import json
        import tempfile
        import line_config
        import webui
        from donkey_part import ThrottleCap, load_line_config, car_model
        path = os.path.join(tempfile.mkdtemp(), "tuning.json")
        cfg, ex = load_line_config(line_config)
        self.t = 0.0
        pilot = LineFollowerPilot(cfg, clock=lambda: self.t, model=car_model(ex),
                                  speed_plan=SpeedPlan(ex["SPEED_STRAIGHT"], ex["SPEED_CORNER"], transition=ex["SPEED_TRANSITION"]))
        hub = webui.Hub(cfg, pilot, None, cap=ThrottleCap(1.0), tuning_path=path, clock=lambda: self.t,
                        race_speeds=webui.race_speeds(ex), safe_speeds=webui.safe_speeds(ex))
        if saved:                                  # a file saved under these same defaults (Hub.save stamps the id)
            with open(path, "w") as f:
                json.dump({**saved, "_defaults_id": hub.defaults_id}, f)
        hub.load_saved()
        hub.attach_autotuner(AutoTuner(pilot, clock=lambda: self.t, log=hub.log))
        hub.telemetry = {"mode": "local"}
        return hub, line_config

    def lap(self, at):
        cmd = SimpleNamespace(error=0.0, angle=0.0, p=0.0, d=0.0)
        for section, n in (("straight", 45), ("corner", 45), ("straight", 1)):   # both sections pass once
            for _ in range(n):
                self.t += .05
                at.update("local", cmd, True, section)

    def test_start_puts_race_values_on_the_car_and_reports_the_estimate(self):
        hub, lc = self.boot(saved={"speed_straight": 0.6, "speed_corner": 0.5, "target_x_ratio": 0.30, "pure_pursuit": 0})
        self.assertEqual(hub.pilot.speed_plan.straight, 0.6)              # the saved file was in charge
        st = hub.autotune("sections_start")
        plan = hub.pilot.speed_plan
        self.assertEqual((plan.straight, plan.corner), (lc.RACE_SPEED_STRAIGHT, lc.RACE_SPEED_CORNER))
        self.assertEqual((hub.cfg.target_x_ratio, hub.cfg.pure_pursuit), (lc.TARGET_X_RATIO, lc.PURE_PURSUIT))
        race = st["race"]
        self.assertEqual((race["state"], race["straight"], race["corner"]), ("driving", lc.RACE_SPEED_STRAIGHT, lc.RACE_SPEED_CORNER))
        self.assertAlmostEqual(race["total_s"], race["laps"] * race["lap_s"], delta=0.06)
        self.assertIn("race values first", st["message"])
        self.assertIn("Estimated lap", st["message"])

    def test_race_values_must_pass_before_it_climbs_and_then_it_reports(self):
        hub, lc = self.boot()
        hub.apply({"max_auto": 1.0, "tune_confirm_laps": 2})
        hub.autotune("sections_start")
        at = hub.autotuner
        at.ceiling = 1.0
        self.lap(at)
        self.assertEqual(at.status()["race"]["state"], "driving")         # one clean lap of two: still the race check
        self.assertEqual(hub.pilot.speed_plan.corner, lc.RACE_SPEED_CORNER)
        self.lap(at)
        self.assertEqual(at.status()["race"]["state"], "passed")
        self.assertGreater(hub.pilot.speed_plan.corner, lc.RACE_SPEED_CORNER)   # only now does it climb
        self.assertTrue(any("Race values clean" in e["text"] for e in hub.events))

    def test_race_values_that_fail_step_down(self):
        hub, lc = self.boot()
        hub.autotune("sections_start")
        at = hub.autotuner
        cmd = SimpleNamespace(error=0.0, angle=0.0, p=0.0, d=0.0)
        for _ in range(5):
            self.t += .05
            at.update("local", cmd, True, "corner")
        for _ in range(9):                                                # line gone longer than the grace period
            self.t += .05
            at.update("local", cmd, False, "unknown")
        self.assertEqual(at.state, "paused")
        self.assertEqual(at.status()["race"]["state"], "stepped down")
        self.assertLess(hub.pilot.speed_plan.corner, lc.RACE_SPEED_CORNER)

    def test_switch_off_keeps_the_sliders(self):
        hub, lc = self.boot(saved={"speed_straight": 0.6, "speed_corner": 0.5, "tune_race_first": 0})
        st = hub.autotune("sections_start")
        self.assertEqual((hub.pilot.speed_plan.straight, hub.pilot.speed_plan.corner), (0.6, 0.5))
        self.assertIsNone(st["race"])

    def test_confirm_lap_failure_rolls_back_to_the_last_fully_confirmed_speed(self):
        hub, lc = self.boot(saved={"tune_race_first": 0, "speed_straight": 0.8, "speed_corner": 0.6, "speed_transition": 0.7,
                                   "tune_confirm_laps": 2, "tune_margin_steps": 0, "tune_step": 0.04})
        hub.autotune("sections_start")
        at = hub.autotuner
        self.lap(at); self.lap(at)                                        # 0.80 / 0.60 confirmed -> now 0.84 / 0.62
        self.assertEqual((hub.pilot.speed_plan.straight, hub.pilot.speed_plan.corner), (0.84, 0.62))
        self.lap(at)                                                      # one clean lap at 0.84 / 0.62 (not confirmed)
        at.fail_sections("corner")                                        # then the curve fails
        self.assertEqual(at.status()["limits"]["corner"]["last_clean"], 0.60)
        self.assertEqual(hub.pilot.speed_plan.corner, 0.60)


class PresetsAndPage(unittest.TestCase):
    def hub(self):
        import line_config
        import webui
        from donkey_part import ThrottleCap, car_model, load_line_config
        cfg, ex = load_line_config(line_config)
        pilot = LineFollowerPilot(cfg, model=car_model(ex),
                                  speed_plan=SpeedPlan(ex["SPEED_STRAIGHT"], ex["SPEED_CORNER"], transition=ex["SPEED_TRANSITION"]))
        hub = webui.Hub(cfg, pilot, None, cap=ThrottleCap(1.0), race_speeds=webui.race_speeds(ex), safe_speeds=webui.safe_speeds(ex))
        hub.attach_autotuner(AutoTuner(pilot))
        return hub, line_config, webui

    def test_forward_pwm_slider_drives_the_throttle_part_and_the_model(self):
        hub, lc, webui = self.hub()

        class FakeThrottle:
            max_pulse, zero_pulse, min_pulse = 407, 370, 300
        part = FakeThrottle()
        hub.attach_throttle(part)
        self.assertEqual(part.max_pulse, lc.THROTTLE_FORWARD_PWM)       # line_config value applied at start
        hub.apply({"forward_pwm": 425})
        self.assertEqual((part.max_pulse, hub.pilot.model.pwm_full), (425, 425.0))
        hub.apply({"forward_pwm": 999})                                   # hard limit
        self.assertEqual(part.max_pulse, 460)
        self.assertEqual((part.zero_pulse, part.min_pulse), (370, 300))   # stop and reverse never touched
        hub.apply_preset("slow")                                          # slow speeds were driven at 407
        self.assertEqual(part.max_pulse, 407)

    def test_presets_only_use_real_knobs_and_race_is_line_config(self):
        hub, lc, webui = self.hub()
        presets = {p["id"]: p for p in hub.presets()}
        self.assertEqual(set(presets), {"race", "safe", "slow", "pd"})
        for p in presets.values():
            self.assertTrue(set(p["values"]) <= webui.KNOB_IDS, set(p["values"]) - webui.KNOB_IDS)
            self.assertTrue(p["estimate"]["estimate"])
        race = presets["race"]["values"]
        self.assertEqual((race["speed_straight"], race["speed_corner"], race["target_x_ratio"], race["max_steering"]),
                         (lc.RACE_SPEED_STRAIGHT, lc.RACE_SPEED_CORNER, lc.TARGET_X_RATIO, 1.0))
        hub.apply({"speed_straight": 0.5, "target_x_ratio": 0.3, "pure_pursuit": 0})
        hub.apply_preset("race")
        self.assertEqual((hub.pilot.speed_plan.straight, hub.cfg.target_x_ratio, hub.cfg.pure_pursuit),
                         (lc.RACE_SPEED_STRAIGHT, lc.TARGET_X_RATIO, True))
        safe = presets["safe"]["values"]
        self.assertEqual((safe["speed_straight"], safe["speed_corner"], safe["target_x_ratio"]),
                         (lc.SAFE_SPEED_STRAIGHT, lc.SAFE_SPEED_CORNER, lc.TARGET_X_RATIO))
        pwm = lambda p, k: 370 + p[k] * (p["forward_pwm"] - 370)                # throttle is relative to the preset's PWM
        self.assertEqual((race["forward_pwm"], safe["forward_pwm"]), (lc.RACE_FORWARD_PWM, lc.SAFE_FORWARD_PWM))
        self.assertLessEqual(pwm(safe, "speed_corner"), pwm(race, "speed_corner") + 0.5)
        self.assertLess(pwm(safe, "speed_straight"), pwm(race, "speed_straight"))
        hub.apply_preset("pd")
        self.assertFalse(hub.cfg.pure_pursuit)
        self.assertEqual(hub.cfg.target_x_ratio, 0.40)
        self.assertEqual(hub.cfg.max_steering, 1.0)                        # the hard standard survives every preset

    def test_every_knob_is_a_slider_on_the_page_and_nothing_is_hard_coded(self):
        import re
        hub, lc, webui = self.hub()
        with open(os.path.join(ROOT, "pi", "laptuner.html")) as f:
            page = f.read()
        groups = page[page.index("const GROUPS"):page.index("const HELP")]
        on_page = set(re.findall(r"\['([a-z_0-9]+)',", groups))
        self.assertEqual(webui.KNOB_IDS - on_page, set())                  # every tunable has a row
        self.assertEqual(on_page - webui.KNOB_IDS, set())                  # and every row is a real knob
        self.assertNotIn("Object.assign(pending, {", page)                 # no tuning numbers baked into the page
        with open(os.path.join(ROOT, "deploy.sh")) as f:
            self.assertIn("laptuner.html", f.read())

    def test_reset_to_defaults_also_resets_saved_tuner_rules(self):
        import json
        import tempfile
        hub, lc, webui = self.hub()
        path = os.path.join(tempfile.mkdtemp(), "tuning.json")
        cfg = hub.cfg
        hub2 = webui.Hub(cfg, hub.pilot, None, tuning_path=path)
        with open(path, "w") as f:
            json.dump({"tune_margin_steps": 3, "tune_confirm_laps": 4, "_defaults_id": hub2.defaults_id}, f)
        hub2.load_saved()
        hub2.attach_autotuner(AutoTuner(hub.pilot))
        self.assertEqual(hub2.autotuner.margin_steps, 3)
        hub2.revert()
        self.assertEqual((hub2.autotuner.margin_steps, hub2.autotuner.confirm_laps), (1, 2))

    def test_tuner_never_retunes_kp_under_pure_pursuit_even_with_a_pd_share(self):
        from linetrack.config import Config
        for pd in (0.0, 0.15, 0.5):
            at = AutoTuner(LineFollowerPilot(Config(pure_pursuit=True, pursuit_pd=pd)))
            self.assertFalse(at._gains_matter(), pd)       # sim: retuning KP under pursuit drove it 3.5 -> 0.5
        self.assertTrue(AutoTuner(LineFollowerPilot(Config(pure_pursuit=False)))._gains_matter())

    def test_car_starts_at_driven_speed_and_tuner_climbs_from_it(self):
        import line_config as lc
        from donkey_part import load_line_config
        _, ex = load_line_config(lc)
        self.assertLessEqual(ex["SPEED_STRAIGHT"], 0.75)                  # never boot above what the car has driven
        self.assertFalse(ex["TUNE_RACE_FIRST"])                           # Start climbs from there
        hub, _, _ = self.hub()
        race = next(p for p in hub.presets() if p["id"] == "race")["values"]
        self.assertEqual((race["speed_straight"], race["speed_corner"]), (lc.RACE_SPEED_STRAIGHT, lc.RACE_SPEED_CORNER))

    def test_lap_estimate_for_the_race_values(self):
        hub, lc, webui = self.hub()
        est = next(p for p in hub.presets() if p["id"] == "race")["estimate"]
        self.assertTrue(est["estimate"])
        # Estimate at PWM 407, 0.95 / 0.75 from the assumed (tired-battery) speed table, which also ignores the shorter
        # inside line. Real laps Nov 11 (7.9 V, inside line 0.18 m): 3.26-3.48 s.
        self.assertAlmostEqual(est["lap_s"], 5.6, delta=0.25)
        self.assertAlmostEqual(est["total_s"], 10 * est["lap_s"], delta=0.06)
