"""Corner geometry (linetrack/geometry.py): ground projection, curvature fit, feedforward,
corner speed, the corner-aware PD target, and how the pilot and simulator use them."""
import math
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "pi"), ROOT]

import line_config  # noqa: E402
from donkey_part import LineFollowerPilot, load_line_config  # noqa: E402
from linetrack.config import Config  # noqa: E402
from linetrack.controller import DRIVE, SteeringController  # noqa: E402
from linetrack.geometry import (Bend, CarModel, GroundModel, allowed_speed, cmd_for_speed, corner_speed, predict_goal,  # noqa: E402
                                fit_bend, min_radius, pursuit_goal, pursuit_steer, speed_of_cmd, steer_for_curvature)


def arc(R, lat=0.15, s_max=1.0, n=12, noise=0.0, seed=0):
    """Ground points of an edge starting 0.3 m ahead at lateral `lat`, curving with radius R (+ left)."""
    rng = np.random.default_rng(seed)
    s = np.linspace(0, s_max, n)
    if math.isinf(R):
        X, Y = 0.3 + s, np.full_like(s, lat)
    else:
        k = 1 / R
        X, Y = 0.3 + np.sin(k * s) / k, lat + (1 - np.cos(k * s)) / k
    return np.stack([X + rng.normal(0, noise, n), Y + rng.normal(0, noise, n)], 1)


class TestGroundModel(unittest.TestCase):
    def test_matches_the_simulator_camera(self):
        from sim.camera import Camera
        from sim.params import Profile
        g, cam = GroundModel(CarModel()), Camera(Profile.load())
        both = np.isfinite(cam.gx) & np.isfinite(g.gx)
        self.assertGreater(both.sum(), 20000)
        self.assertLess(float(np.max(np.abs(g.gx[both] - cam.gx[both]))), 1e-9)

    def test_projection_defaults_equal_the_plain_equidistant_model(self):
        from linetrack.geometry import project_pixels
        W, H, fov, pitch, h, fwd = 240, 192, 160.0, 8.0, 0.17, 0.15
        u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
        f = math.hypot(W / 2, H / 2) / math.radians(fov / 2)
        th, az = np.hypot(u - W / 2, v - H / 2) / f, np.arctan2(v - H / 2, u - W / 2)
        rx, ry, rz = np.sin(th) * np.cos(az), np.sin(th) * np.sin(az), np.cos(th)
        p = math.radians(pitch)
        X, Y, Z = -ry * math.sin(p) + rz * math.cos(p), -rx, -ry * math.cos(p) - rz * math.sin(p)
        t = np.where(Z < -1e-6, h / np.maximum(-Z, 1e-9), np.nan)
        gx, gy = project_pixels(u, v, W, H, fov, pitch, h, fwd)
        ok = np.isfinite(t)
        self.assertLess(float(np.max(np.abs(gx[ok] - (fwd + t * X)[ok]))), 1e-9)
        self.assertLess(float(np.max(np.abs(gy[ok] - (t * Y)[ok]))), 1e-9)

    def test_roll_tilts_the_ground_and_centre_offset_shifts_it(self):
        from linetrack.geometry import project_pixels
        u = np.array([60.0, 180.0])
        v = np.array([150.0, 150.0])
        _, gy0 = project_pixels(u, v, 240, 192, 160, 8, 0.17, 0.15)
        gx1, _ = project_pixels(u, v, 240, 192, 160, 8, 0.17, 0.15, roll_deg=5)
        self.assertAlmostEqual(gy0[0], -gy0[1], places=9)                  # symmetric without roll
        self.assertNotAlmostEqual(gx1[0], gx1[1], places=3)                # roll: left and right reach differently
        _, gyc = project_pixels(np.array([120.0]), np.array([150.0]), 240, 192, 160, 8, 0.17, 0.15, cx_px=10)
        self.assertGreater(gyc[0], 0)                                       # image centre now right of the axis

    def test_lower_rows_are_closer(self):
        g = GroundModel(CarModel())
        self.assertLess(g.row_distance(0.9), g.row_distance(0.6))


class TestFit(unittest.TestCase):
    def test_straight_reads_zero_even_with_noise(self):
        ks = [fit_bend(arc(math.inf, noise=0.005, seed=i)).curvature for i in range(100)]
        self.assertLess(abs(np.mean(ks)), 0.02)
        self.assertLess(np.std(ks), 0.05)

    def test_arcs_both_directions(self):
        for R in (3.0, 1.2, 0.525, -0.525):
            b = fit_bend(arc(R, s_max=min(1.0, 0.9 * abs(R) * math.pi / 2)))
            self.assertAlmostEqual(b.edge_curvature, 1 / R, places=2)
            # the car rides 0.15 m to the right of the edge: left turns are wider, right turns tighter
            self.assertAlmostEqual(b.curvature, (1 / R) / (1 + 0.15 / R), places=2)

    def test_too_few_points(self):
        self.assertIsNone(fit_bend(np.zeros((3, 2))))
        self.assertIsNone(fit_bend(None))


class TestFormulas(unittest.TestCase):
    m = CarModel()

    def test_steering_follows_atan_and_sign(self):
        k = 1 / 0.861                                    # the stage corner, path radius
        cmd = steer_for_curvature(self.m, k)
        self.assertAlmostEqual(cmd, -math.degrees(math.atan(0.1741 * k)) / self.m.steer_max_deg, places=6)   # left = negative
        self.assertLess(cmd, 0)
        self.assertGreater(steer_for_curvature(self.m, -k), 0)
        self.assertEqual(steer_for_curvature(self.m, 1 / (0.5 * min_radius(self.m))), -1.0)   # clipped

    def test_corner_speed_and_braking(self):
        self.assertAlmostEqual(corner_speed(self.m, 1 / 0.861, margin=1.0), math.sqrt(0.5 * 9.81 * 0.861), places=6)
        self.assertTrue(math.isinf(corner_speed(self.m, 0.0)))
        bend = Bend(1 / 0.861, 2.0, 0.15, 0.6, 1.2, 10, 0.0)
        v_c = corner_speed(self.m, bend.curvature)
        self.assertGreater(allowed_speed(self.m, bend, 1.0), v_c)          # still room to brake
        far = Bend(bend.curvature, 2.0, 0.15, 3.0, 4.0, 10, 0.0)
        self.assertGreater(allowed_speed(self.m, far, 1.0), allowed_speed(self.m, bend, 1.0))

    def test_speed_table_round_trip(self):
        for v in (0.35, 0.8, 1.2, 1.7):
            self.assertAlmostEqual(speed_of_cmd(self.m, cmd_for_speed(self.m, v)), v, places=6)
        self.assertEqual(cmd_for_speed(self.m, 5.0), 1.0)

    def test_corner_aware_target(self):
        g = GroundModel(self.m)
        self.assertEqual(g.expected_x_ratio(0.62, 0.20, 0.0), 0.20)
        self.assertLess(g.expected_x_ratio(0.62, 0.20, 1.16), 0.20)    # left curve: the line appears further left
        self.assertGreater(g.expected_x_ratio(0.62, 0.20, -1.16), 0.20)


class TestControllerAndPilot(unittest.TestCase):
    def test_section_gains_blend_without_a_kick(self):
        c = Config()
        c.kp, c.kd = 2.0, 0.2
        c.straight_kp, c.straight_kd = 4.0, 0.4
        c.corner_kp, c.corner_kd = 1.0, 0.1
        c.curvature_smoothing = 0.0
        c.steering_smoothing, c.steering_rate_limit = 0.0, 100
        t = [0.0]
        pilot = LineFollowerPilot(c, clock=lambda: t[0])
        straight = Bend(0.0, 0.0, 0.15, 0.4, 1.0, 10, 0.0)
        corner = Bend(1.2, 1.0, 0.15, 0.4, 1.0, 10, 0.0)
        pilot.run("local_angle", 0.25, True, 0.0, bend=straight)
        self.assertEqual(pilot.last_cmd.section, "unknown")
        t[0] = 0.2
        pilot.run("local_angle", 0.25, True, 0.0, bend=straight)
        self.assertEqual(pilot.last_cmd.section, "straight")
        self.assertAlmostEqual(pilot.last_cmd.kp_used, 4.0)
        pilot.run("local_angle", 0.25, True, 0.0, bend=corner)
        t[0] = 0.4
        pilot.run("local_angle", 0.25, True, 0.0, bend=corner)
        self.assertEqual(pilot.last_cmd.section, "corner")
        self.assertAlmostEqual(pilot.last_cmd.kp_used, 1.0)

    def test_feedforward_adds_and_target_moves_the_error(self):
        c = Config()
        c.kp, c.kd, c.ff_gain, c.steering_smoothing, c.steering_rate_limit = 2.0, 0.0, 0.5, 0.0, 100
        ctl = SteeringController(c)
        cmd = ctl.update(0.20, True, 0, 0.05, 0, DRIVE, ff=-0.4)
        self.assertAlmostEqual(cmd.ff, -0.2)
        self.assertAlmostEqual(cmd.angle, -0.2)
        ctl.reset()
        cmd = ctl.update(0.20, True, 0, 0.05, 0, DRIVE, target=0.10)
        self.assertAlmostEqual(cmd.error, 0.10)
        c.invert_steering = True
        ctl.reset()
        self.assertAlmostEqual(ctl.update(0.20, True, 0, 0.05, 0, DRIVE, ff=-0.4).ff, 0.2)

    def test_off_by_default_on_the_car(self):
        cfg, _ = load_line_config(line_config)
        self.assertEqual(cfg.ff_gain, 0.0)
        self.assertFalse(cfg.corner_plan)
        pilot = LineFollowerPilot(cfg, clock=lambda: 0.0)
        pilot.run("local_angle", 0.2, True, 0.0, bend=Bend(1.5, 2.0, 0.15, 0.4, 1.0, 10, 0.0))
        self.assertEqual(pilot.last_cmd.ff, 0.0)

    def test_pilot_feedforward_and_corner_cap(self):
        cfg, _ = load_line_config(line_config)
        cfg.ff_gain, cfg.corner_plan, cfg.max_throttle = 1.0, True, 1.0
        t = [0.0]
        from donkey_part import SpeedPlan
        pilot = LineFollowerPilot(cfg, clock=lambda: t[0], speed_plan=SpeedPlan(1.0, 1.0))
        pilot.model.grip_mu = 0.3
        tight = Bend(2.5, 3.0, 0.15, 0.35, 0.9, 10, 0.0)                # sharp left ahead
        for i in range(6):
            t[0] = 0.05 * i
            _, thr = pilot.run("local", 0.2, True, 0.0, motion=5.0, bend=tight)
        self.assertLess(pilot.last_cmd.ff, 0)                            # steers left
        self.assertIsNotNone(pilot.corner_cap)
        self.assertLess(thr, 1.0)                                        # slows for the corner
        self.assertGreaterEqual(thr, cfg.corner_min_throttle)            # never below the stall point

    def test_speed_plan_uses_curvature_transition_and_slews(self):
        from donkey_part import SpeedPlan
        p = SpeedPlan(0.70, 0.50, transition=0.60, ramp_down=1.0, ramp_up=0.4)
        self.assertAlmostEqual(p.target(0.0, 0.8, 0.0), 0.70)
        self.assertAlmostEqual(p.target(0.0, 0.8, 0.5), 0.60)
        self.assertAlmostEqual(p.target(0.0, 0.8, 1.0), 0.50)
        self.assertAlmostEqual(p.throttle(0.0, 0.8, 0.0, 0.1), 0.70)
        self.assertAlmostEqual(p.throttle(0.0, 0.8, 1.0, 0.1), 0.60)  # 0.1 s of controlled braking


class TestFullLockAngle(unittest.TestCase):
    """One full-lock wheel angle for the car, the tools and the simulator (it was 25 on the car and
    18 in the simulator, so feedforward under-steered by 28%)."""

    def test_car_config_and_simulator_agree(self):
        from sim.params import Profile
        table = Profile.load().steer_angle_table
        sim_lock = max(abs(row[1]) for row in table)
        cfg, extras = load_line_config(line_config)
        self.assertEqual(cfg.steer_max_deg, sim_lock)
        self.assertEqual(extras["STEER_MAX_DEG"], sim_lock)
        self.assertEqual(Config().steer_max_deg, sim_lock)
        self.assertEqual(CarModel().steer_max_deg, sim_lock)

    def test_feedforward_uses_the_slider_value(self):
        cfg = Config(ff_gain=1.0, pure_pursuit=False, max_steering=1.0, steering_smoothing=0.0, steering_rate_limit=100.0)
        k = 1 / 0.69
        out = {}
        for lock in (18.0, 25.0):
            cfg.steer_max_deg = lock
            t = [0.0]
            p = LineFollowerPilot(cfg, clock=lambda: t[0])
            for _ in range(5):
                t[0] += 0.05
                p.run("local_angle", cfg.target_x_ratio, True, 0.0, bend=Bend(k, k, 0.1, 0.3, 1.0, 10, 0.0))
            out[lock] = p.last_cmd.ff
        self.assertAlmostEqual(out[18.0], -math.degrees(math.atan(0.1741 * k)) / 18.0, places=4)
        self.assertAlmostEqual(out[25.0] / out[18.0], 18.0 / 25.0, places=4)


class TestPurePursuit(unittest.TestCase):
    L = 0.1741

    def test_prediction_off_or_stopped_leaves_the_goal(self):
        self.assertEqual(predict_goal((0.6, 0.1), 1.5, 10.0, self.L, 0.0), (0.6, 0.1))
        self.assertEqual(predict_goal((0.6, 0.1), 0.0, 10.0, self.L, 0.15), (0.6, 0.1))

    def test_prediction_on_a_straight_moves_the_goal_closer(self):
        x, y = predict_goal((0.6, 0.1), 1.0, 0.0, self.L, 0.1)       # 0.1 m driven straight ahead
        self.assertAlmostEqual(x, 0.5, places=6)
        self.assertAlmostEqual(y, 0.1, places=6)

    def test_prediction_on_a_circle_keeps_a_point_of_that_circle_on_it(self):
        # turning left on radius R: the circle stays the same circle in the new car frame, so a goal on
        # it stays on it, and it comes closer by the arc driven
        R, wheel = 0.56, math.degrees(math.atan(self.L / 0.56))
        th = 1.2
        goal = (R * math.sin(th), R * (1 - math.cos(th)))
        x, y = predict_goal(goal, 1.5, wheel, self.L, 0.1)
        self.assertAlmostEqual(math.hypot(x, y - R), R, places=6)     # still on the circle (centre (0, R))
        self.assertAlmostEqual(math.atan2(x, R - y), th - 0.15 / R, places=6)

    def test_prediction_never_puts_the_goal_behind_the_car(self):
        self.assertEqual(predict_goal((0.1, 0.0), 2.0, 0.0, self.L, 0.2), (0.1, 0.0))

    @staticmethod
    def edge(radius=None, lateral=0.10, n=30, reach=0.9):
        """Ground points of a left edge `lateral` m left of a car riding its line: straight, or an arc
        (the car's path has `radius`, the edge is the concentric circle inside it)."""
        if radius is None:
            return np.stack([np.linspace(0.25, reach, n), np.full(n, lateral)], 1)
        a = np.linspace(0.25 / radius, reach / radius, n)
        r = radius - lateral
        return np.stack([r * np.sin(a), radius - r * np.cos(a)], 1)

    def test_on_the_line_it_steers_exactly_the_curve(self):
        self.assertAlmostEqual(pursuit_steer(pursuit_goal(self.edge(), 0.10, 0.6), self.L, 18.0), 0.0, places=3)
        for radius in (0.6, 0.69, 1.0):
            for ld in (0.4, 0.55, 0.7):       # any lookahead gives the same answer on a circle
                cmd = pursuit_steer(pursuit_goal(self.edge(radius, reach=1.0 * radius), 0.10, ld), self.L, 18.0)
                self.assertAlmostEqual(cmd, -math.degrees(math.atan(self.L / radius)) / 18.0, delta=0.03)

    def test_sign_and_offset(self):
        # edge further left than it should be = the car has drifted right of its line -> steer left
        self.assertLess(pursuit_steer(pursuit_goal(self.edge(lateral=0.20), 0.10, 0.6), self.L, 18.0), -0.05)
        self.assertGreater(pursuit_steer(pursuit_goal(self.edge(lateral=0.02), 0.10, 0.6), self.L, 18.0), 0.05)
        # a right-hand line (negative lateral) rides the same way
        right = self.edge(lateral=-0.25)
        self.assertAlmostEqual(pursuit_steer(pursuit_goal(right, -0.25, 0.6), self.L, 18.0), 0.0, places=3)

    def test_short_edge_uses_its_far_end_and_too_short_is_refused(self):
        goal = pursuit_goal(self.edge(reach=0.45), 0.10, 0.8)
        self.assertLess(math.hypot(*goal), 0.5)                       # never extrapolates past what it sees
        self.assertIsNone(pursuit_goal(self.edge(n=3), 0.10, 0.6))
        self.assertIsNone(pursuit_goal(np.zeros((0, 2)), 0.10, 0.6))

    def test_pilot_uses_pursuit_and_falls_back_to_pd_without_an_edge(self):
        cfg = Config(pure_pursuit=True, pursuit_pd=0.0, pursuit_ld_m=0.5, pursuit_ld_gain_s=0.0, kp=3.5, kd=0.0,
                     max_steering=1.0, steering_smoothing=0.0, steering_rate_limit=100.0, target_x_ratio=0.40)
        t = [0.0]
        p = LineFollowerPilot(cfg, clock=lambda: t[0])
        lateral = GroundModel(p.model).lateral_at(cfg.lookahead_ratio, cfg.target_x_ratio)
        t[0] += 0.05
        p.run("local_angle", 0.30, True, 0.0, path=(self.edge(lateral=lateral), "left", 0.47))
        self.assertIsNotNone(p.pursuit_goal)
        self.assertAlmostEqual(p.last_cmd.angle, 0.0, places=2)      # on its line: the image-row error is ignored (PD share 0)
        t[0] += 0.05
        p.run("local_angle", 0.30, True, 0.0, path=None)            # no edge geometry this frame -> plain PD
        self.assertIsNone(p.pursuit_goal)
        self.assertAlmostEqual(p.last_cmd.angle, 3.5 * (0.30 - 0.40), places=3)

    def test_race_defaults_lap_the_calibrated_track_without_running_wide(self):
        sys.path.insert(0, os.path.join(ROOT, "tools"))
        import simdrive
        import line_config as lc
        r = simdrive.drive({"forward_pwm": lc.RACE_FORWARD_PWM}, straight=lc.RACE_SPEED_STRAIGHT, corner=lc.RACE_SPEED_CORNER,
                           seconds=20, seed=1)  # race values
        self.assertTrue(simdrive.build()[0].pure_pursuit)
        self.assertEqual((r["wide"], r["inside"]), (0, 0))
        self.assertGreaterEqual(r["laps"], 4)
        self.assertLess(r["lap"], 4.2)    # simulated 4.0 s at 1.00 / 0.86; the real car did 3.54-3.77 s (Nov 10)


class TestRecorder(unittest.TestCase):
    def test_runs_log_curvature_for_validation(self):
        import csv
        import tempfile
        from donkey_part import LineDetectorPart, LineRecorder
        cfg, _ = load_line_config(line_config)
        det = LineDetectorPart(cfg, show_overlay=False)
        pilot = LineFollowerPilot(cfg)
        det.pilot = pilot
        img = np.full((192, 240, 3), 180, np.uint8)
        rec = LineRecorder(tempfile.mkdtemp(), det)
        rec.run(img, "user", None, False, 0.0, 0.0, 0.0)
        pilot.run("local_angle", 0.2, True, 0.0)
        det.bend = Bend(1.2, 1.5, 0.15, 0.3, 1.0, 10, 0.0)
        rec.run(img, "local_angle", 0.2, True, 0.0, 0.1, 0.0)
        rec.shutdown()
        with open(os.path.join(rec.dir, "log.csv")) as f:
            raw = list(csv.reader(f))
        self.assertEqual({len(r) for r in raw}, {len(raw[0])})
        with open(os.path.join(rec.dir, "log.csv")) as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(rows[1]["curv"], "1.2000")
        self.assertEqual(rows[1]["ff"], "0.0000")


class TestInSimulator(unittest.TestCase):
    def test_measured_curvature_matches_the_track(self):
        from sim.__main__ import _Clock
        from sim.controllers import PilotController
        from sim.params import Profile
        from sim.world import World
        prof, clk = Profile.load(), _Clock()
        ctrl = PilotController("t", {"kp": 3.0, "kd": 1.0, "ff_gain": 0.6}, speed=0.7, clock=clk)
        w = World(prof, seed=3, target_ratio=ctrl.cfg.target_x_ratio, row_ratio=ctrl.cfg.lookahead_ratio)
        ks = []
        while w.t < 20:
            clk.t = w.t
            x, found, both, age, motion, img = w.observe()
            a, th, _ = ctrl.step(x, found, both, age, motion, img, edges=w.edges, source=w.edges_source)
            w.tick(a, th)
            if ctrl.pilot.curvature is not None:
                ks.append(ctrl.pilot.curvature)
        ride = w.p.tape_width_m / 2 + w._ride
        k_true = 2.0 / (1 + 2.0 * ride)                 # tape centreline radius 0.5 m, moved to the ride path
        corner = [k for k in ks if k > 0.8]
        self.assertGreater(len(corner), 50)
        self.assertAlmostEqual(float(np.median(corner)), k_true, delta=0.15 * k_true)
        self.assertEqual(w.metrics.off_track, 0)

    def test_feedforward_stays_off_while_pure_pursuit_steers(self):
        # Curvature feedforward only acts under PD steering (the "PD fallback" preset). Under the Nov 10 evening
        # camera (0.15 m / 8 deg) it helps PD only marginally in the simulator (0.75 throttle, 10 draws: 30 PD vs
        # 26 PD + feedforward 0.6 off-tracks; under the old 0.12 m / 15 deg model it was 14 vs 25), and the PD
        # fallback is meant to be the setup the real car drove on Nov 9, so it stays off.
        import line_config as lc
        self.assertTrue(lc.PURE_PURSUIT)
        self.assertEqual(lc.FF_GAIN, 0.0)

if __name__ == "__main__":
    unittest.main()
