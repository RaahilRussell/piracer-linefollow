"""The PiRacer simulator: mappings, physics limits, camera, linear analysis, provenance."""
import json
import math
import os
import sys
import unittest

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "pi"), ROOT]

import line_config  # noqa: E402
from donkey_part import load_line_config  # noqa: E402
from linetrack.config import Config  # noqa: E402
from linetrack.detector import LeftLineDetector  # noqa: E402
from sim.__main__ import run_one  # noqa: E402
from sim.actuators import SteeringMap, ThrottleMap  # noqa: E402
from sim.camera import Camera, target_offset  # noqa: E402
from sim.feasibility import corner_report  # noqa: E402
from sim.params import STATUSES, Profile  # noqa: E402
from sim.track import Track  # noqa: E402
from sim.tune import LinearLoop  # noqa: E402

P = Profile.load()


class TestProvenance(unittest.TestCase):
    def test_every_parameter_has_status_and_source(self):
        for k in P.names():
            e = P.entry(k)
            self.assertIn(e.get("status"), STATUSES, k)
            self.assertTrue(e.get("source"), k)
            if e["status"] == "assumed" and isinstance(e["value"], (int, float)):
                self.assertIn("range", e, f"assumed {k} needs a range to sweep")

    def test_verified_specs(self):
        self.assertAlmostEqual(P.wheelbase_m, 0.1741)
        self.assertEqual(P.entry("wheelbase_m")["status"], "verified")

    def test_unknowns_are_not_claimed_as_known(self):
        for k in ("steer_angle_table", "speed_table", "mu_lat", "servo_rate_dps", "cam_height_m", "cam_pitch_deg", "accel_max"):
            self.assertEqual(P.entry(k)["status"], "assumed", k)


class TestActuators(unittest.TestCase):
    def test_steering_command_maps_like_donkeycar(self):
        sm = SteeringMap(P)
        self.assertEqual(sm.pwm(-1), 450)      # STEERING_LEFT_PWM
        self.assertEqual(sm.pwm(1), 300)       # STEERING_RIGHT_PWM
        self.assertEqual(sm.pwm(0), 375)
        # a command is NOT an angle: the angle comes from the calibration table
        q = P.with_values(steer_angle_table=[[300, 30.0], [380, 0.0], [450, -18.0]])
        sq = SteeringMap(q)
        self.assertAlmostEqual(sq.angle_deg(sq.pwm(1)), 30.0)
        self.assertAlmostEqual(sq.angle_deg(sq.pwm(-1)), -18.0)   # asymmetric left/right is kept
        self.assertNotAlmostEqual(sq.angle_deg(sq.pwm(0)), 0.0)   # trim: 375 isn't straight on this car

    def test_throttle_maps_like_donkeycar(self):
        tm = ThrottleMap(P)
        self.assertEqual(tm.pwm(0), 370)
        self.assertEqual(tm.pwm(1), 407)
        self.assertEqual(tm.pwm(-1), 300)
        self.assertEqual(tm.steady_speed(tm.pwm(0.30), True), 0.0)          # PWM 381: deadband (didn't move on the car)
        self.assertEqual(tm.steady_speed(tm.start_pwm - 1, False), 0.0)     # can't start below start_pwm
        self.assertGreater(tm.steady_speed(407, True), 0)


class TestPhysicsLimits(unittest.TestCase):
    def test_geometrically_impossible_corner_is_detected(self):
        q = P.with_values(steer_angle_table=[[300, 5.0], [375, 0.0], [450, -5.0]])   # almost no lock
        tr = Track.stadium(q)
        rows, _ = corner_report(q, tr, 0.3, 0.3)
        self.assertTrue(all(r["verdict"].startswith("IMPOSSIBLE") for r in rows))

    def test_too_fast_corner_is_detected_and_slides(self):
        q = P.with_values(mu_lat=0.25, speed_table=[[370, 0.0], [387, 0.0], [390, 0.4], [407, 2.4]])
        tr = Track.stadium(q)
        rows, _ = corner_report(q, tr, 0.3, 2.4)
        self.assertTrue(all(r["verdict"].startswith("TOO FAST") for r in rows))
        w, _ = run_one(q, seconds=15, speed=1.0)
        s = w.metrics.summary()
        self.assertTrue(s["slip_pct"] > 0 or s["off_track"] > 0)

    def test_crawl_laps_cleanly(self):
        w, _ = run_one(P, seconds=40, speed=0.65)
        s = w.metrics.summary()
        self.assertEqual(s["off_track"], 0)
        self.assertGreaterEqual(s["laps"], 2)


class TestCamera(unittest.TestCase):
    def test_geometric_fast_path_matches_real_detector_on_rendered_frame(self):
        cam, tr = Camera(P), Track.stadium(P)
        off = target_offset(cam, None, 0.20, 0.62, P.tape_width_m)
        pose = (0.4, -(P.tape_width_m / 2 + off), 0.0)
        img = cam.render(pose, tr, np.random.default_rng(0))
        det = LeftLineDetector(Config()).detect(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        x_geo = cam.measure_row(pose, tr, 0.62, 0.47)[0]
        self.assertTrue(det.found)
        self.assertLess(abs(det.x_ratio - x_geo), 0.02)
        self.assertLess(abs(x_geo - 0.20), 0.02)

    def test_lookahead_is_ahead_and_lower_rows_are_closer(self):
        cam = Camera(P)
        self.assertGreater(cam.row_distance(0.55), cam.row_distance(0.62))
        self.assertGreater(cam.row_distance(0.62), cam.row_distance(0.8))


class TestLinearAnalysis(unittest.TestCase):
    def test_safe_gain_shrinks_with_speed(self):
        cfg, _ = load_line_config(line_config)
        slow, fast = LinearLoop(P, 0.5, cfg), LinearLoop(P, 1.8, cfg)
        a_slow, a_fast = slow.analyze(3.5, 0.4), fast.analyze(3.5, 0.4)
        self.assertTrue(a_slow["stable"])
        self.assertGreater(a_slow["pm_deg"], 45)
        self.assertTrue(not a_fast["stable"] or a_fast["pm_deg"] < a_slow["pm_deg"] - 20)
        self.assertFalse(fast.analyze(9.0, 0.4)["stable"])

    def test_more_delay_less_margin(self):
        cfg, _ = load_line_config(line_config)
        a = LinearLoop(P, 1.1, cfg).analyze(2.5, 0.5)
        b = LinearLoop(P.with_values(actuator_delay_s=0.06), 1.1, cfg).analyze(2.5, 0.5)
        self.assertLess(b["delay_margin_ms"], a["delay_margin_ms"])


class TestSchedule(unittest.TestCase):
    def test_schedule_file_is_sane(self):
        with open(os.path.join(ROOT, "sim", "gain_schedule.json")) as f:
            sch = json.load(f)
        rows = sch["profiles"]
        self.assertGreaterEqual(len(rows), 4)
        self.assertTrue(sch["assumed_parameters"])                      # says what's still unmeasured
        kps = [r["kp"] for r in rows]
        self.assertGreater(kps[0], kps[-1])                             # gains fall as speed rises
        for r in rows:
            self.assertGreaterEqual(r["pm_deg"], 45 if r["meets_margins"] else 0)


if __name__ == "__main__":
    unittest.main()
