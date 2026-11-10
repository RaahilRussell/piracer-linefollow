"""Tests for the dashboard's data-source honesty, the diagnoser (pi/diagnose.py) and the
simulator advisor (sim/advisor.py)."""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "pi"), ROOT]

import diagnose as dg  # noqa: E402
from sim import advisor  # noqa: E402
from tests.test_webui import Clock, make_hub  # noqa: E402

PROFILES = {"profiles": [
    {"profile": "Crawl", "straight_cmd": 0.55, "kp": 8.5, "kd": 1.2},
    {"profile": "Slow", "straight_cmd": 0.62, "kp": 6.0, "kd": 1.1},
    {"profile": "Medium", "straight_cmd": 0.70, "kp": 3.0, "kd": 1.0},
]}


def tele(**kw):
    t = {"t": 0.0, "mode": "local", "status": "TRACKING", "source": "left", "conf": 0.9, "hz": 20, "throttle": 0.6,
         "motion": 5.0, "single_stop": False, "web_driving": False, "pwm": 392}
    t.update(kw)
    return t


class TestLiveIsOnlyTheCar(unittest.TestCase):
    def test_car_without_telemetry_is_not_live(self):
        hub = make_hub(clock=Clock())[0]
        st = hub.status()
        self.assertEqual(st["source"], "car")
        self.assertFalse(st["live"])
        self.assertFalse(st["fresh"])

    def test_car_with_fresh_telemetry_is_live_and_stale_is_not(self):
        clk = Clock()
        hub = make_hub(clock=clk)[0]
        hub.publish({"t": clk.t, "mode": "user"}, None)
        self.assertTrue(hub.status()["live"])
        clk.t += 5
        self.assertFalse(hub.status()["live"])

    def test_simulator_and_replay_are_never_live(self):
        for source in ("simulator", "replay"):
            clk = Clock()
            hub = make_hub(clock=clk)[0]
            hub.source = source
            hub.publish({"t": clk.t, "mode": "user"}, None)
            st = hub.status()
            self.assertTrue(st["fresh"])
            self.assertFalse(st["live"], source)

    def test_no_data_means_diagnosis_says_so(self):
        hub = make_hub(clock=Clock())[0]
        d = hub.diagnose()
        self.assertEqual(d["issues"][0]["id"], "no_data")

    def test_simulator_refused_on_car_while_pilot_drives(self):
        clk = Clock()
        hub = make_hub(clock=clk)[0]
        hub.publish({"t": clk.t, "mode": "local"}, None)
        with self.assertRaises(RuntimeError):
            hub.sim({"action": "speed", "cmd": 0.6})
        hub.source = "simulator"
        self.assertIn("mps", hub.sim({"action": "speed", "cmd": 0.6}))


class TestDiagnose(unittest.TestCase):
    knobs = {"kp": 3.5, "kd": 0.4, "s_lo": 110, "max_auto": 0.6, "lookahead_ratio": 0.62, "single_line_stop_s": 2.5}

    def test_frozen_camera_is_the_top_problem(self):
        out = dg.diagnose(tele(status="STALE"), [], None, self.knobs)
        self.assertEqual(out[0]["id"], "stale")
        self.assertEqual(out[0]["level"], "bad")

    def test_weave_gives_a_concrete_kp_fix(self):
        health = {"findings": [{"level": "bad", "code": "weave", "text": "Weaving fast (1.2 swings/s): KP too high"}],
                  "suggest": {"kp": 0.8}, "seconds": 10, "swing_rate": 1.2, "err_p90": 0.1, "saturated": 0.0}
        out = dg.diagnose(tele(), [tele()] * 50, health, self.knobs)
        weave = next(o for o in out if o["id"] == "health_weave")
        self.assertEqual(weave["apply"], {"kp": 2.8})
        self.assertIn("KP 3.5 → 2.8", weave["apply_label"])

    def test_lost_line_with_low_confidence_points_at_detection(self):
        hist = [tele(status="LOST", conf=0.3)] * 30 + [tele(conf=0.4)] * 20
        out = dg.diagnose(tele(), hist, None, self.knobs)
        lost = next(o for o in out if o["id"].startswith("lost"))
        self.assertEqual(lost["id"], "lost_detect")
        self.assertEqual(lost["apply"], {"s_lo": 100})

    def test_lost_line_with_good_confidence_points_at_speed(self):
        hist = [tele(status="LOST", conf=0.9)] * 30 + [tele()] * 20
        out = dg.diagnose(tele(), hist, None, self.knobs)
        self.assertTrue(any(o["id"] == "lost_speed" for o in out))

    def test_stall_and_gains_far_from_schedule(self):
        out = dg.diagnose(tele(throttle=0.62, motion=0.9), [tele(throttle=0.62)] * 50, None, self.knobs, PROFILES)
        ids = [o["id"] for o in out]
        self.assertIn("stall", ids)
        sched = next(o for o in out if o["id"] == "vs_schedule")
        self.assertEqual(sched["apply"], {"kp": 6.0, "kd": 1.1})

    def test_schedule_interpolates_between_profiles(self):
        s = dg.schedule_for(PROFILES, 0.66)
        self.assertAlmostEqual(s["kp"], 4.5, places=1)   # halfway between 6.0 and 3.0
        self.assertEqual(dg.schedule_for(PROFILES, 0.4)["kp"], 8.5)
        self.assertIsNone(dg.schedule_for({"profiles": []}, 0.6))

    def test_healthy_driving_says_so(self):
        knobs = dict(self.knobs, kp=6.0, kd=1.1)
        out = dg.diagnose(tele(throttle=0.62), [tele(throttle=0.62)] * 50, None, knobs, PROFILES)
        self.assertEqual([o["id"] for o in out], ["ok"])


class TestAdvisor(unittest.TestCase):
    def test_too_slow_to_move(self):
        self.assertFalse(advisor.speed_info(0.30)["moves"])
        self.assertEqual(advisor.evaluate(0.30, 3.5, 0.4)["verdict"][0]["level"], "bad")

    def test_good_gains_pass_and_fast_gains_get_flagged(self):
        good = advisor.evaluate(0.70, 3.0, 1.0)
        # Throttle recovery now respects the acceleration ramp after a dropout.
        # Stable gains may correctly earn a settling-time warning; do not hide it.
        self.assertFalse(any(v["level"] == "bad" for v in good["verdict"]))
        for v in good["verdict"]:
            if v["level"] == "warn":
                self.assertIn("Slow to settle", v["text"])
        self.assertTrue(good["linear"]["stable"])
        self.assertTrue(good["step"]["trace"]["t"])
        self.assertGreater(len(good["path"]), 10)
        fast = advisor.evaluate(0.90, 3.5, 0.4)          # the old KP 3.5 at high speed: thin phase margin
        self.assertTrue(any(v["level"] == "warn" and "Phase margin" in v["text"] for v in fast["verdict"]))

    def test_stability_map_has_all_three_classes(self):
        m = advisor.tuning_map(0.90)
        classes = {c[2] for c in m["cells"]}
        self.assertEqual(classes, {0, 1, 2})

    def test_recommend_matches_the_saved_schedule(self):
        r = advisor.recommend(0.90, mc=4)
        rob = r["robustness"]
        self.assertEqual(r["robust"], rob["unstable"] == 0 and rob["off_track_in_2_runs"] == 0
                         and rob["clearance_failures_in_2_runs"] == 0)
        self.assertLessEqual(abs(r["kp"] - 2.0), 1.0)    # schedule: Faster 0.90 -> KP 2.0
        again = advisor.evaluate(0.90, r["kp"], r["kd"])
        self.assertEqual(again["lap"]["off_track"], 0)


if __name__ == "__main__":
    unittest.main()
