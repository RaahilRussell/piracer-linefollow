"""Tests for pi/webui.py: knob limits, web-drive dead-man, stops, runs, and the HTTP API."""
import json
import os
import shutil
import sys
import tempfile
import time
import unittest
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "pi"), ROOT]

import line_config  # noqa: E402
import webui  # noqa: E402
from donkey_part import AutoCrawl, KillSwitch, LineDetectorPart, LineFollowerPilot, ThrottleCap, load_line_config  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make_hub(clock=None, data_dir="data", tuning=None):
    cfg, ex = load_line_config(line_config)
    crawl = AutoCrawl()
    pilot = LineFollowerPilot(cfg, crawl=crawl, single_line_stop_s=2.5)
    det = LineDetectorPart(cfg)
    det.pilot = pilot
    cap = ThrottleCap(0.6)
    hub = webui.Hub(cfg, pilot, det, crawl=crawl, cap=cap, data_dir=data_dir, tuning_path=tuning,
                    clock=clock or time.monotonic)
    return hub, cfg, pilot, crawl, cap


class TestKnobs(unittest.TestCase):
    def test_values_apply_live_and_are_clamped(self):
        hub, cfg, pilot, crawl, cap = make_hub()
        out = hub.apply({"kp": 4.2, "kd": 99, "top_throttle": 5, "s_lo": 120.4, "bogus": 1})
        self.assertAlmostEqual(cfg.kp, 4.2)          # same object the controller reads every frame
        self.assertAlmostEqual(cfg.kd, 2.0)          # clamped to the knob's max
        self.assertAlmostEqual(cap.top, 1.0)         # clamped to the knob's max (full throttle since "Allow full-scale throttle")
        self.assertEqual(cfg.s_lo, 120)
        self.assertNotIn("bogus", out)

    def test_per_mode_speed_limits(self):
        hub, cfg, pilot, crawl, cap = make_hub()
        hub.apply({"top_throttle": 0.7, "max_user": 0.5, "max_pilot_steers": 0.65, "max_auto": 0.55})
        self.assertAlmostEqual(cap.run(1.0, 0.0, "user"), 0.5)
        self.assertAlmostEqual(cap.run(1.0, 0.0, "local_angle"), 0.65)
        self.assertAlmostEqual(cap.run(1.0, 0.0, "local"), 0.55)
        self.assertEqual(cap.run(-1.0, 0.0, "user"), -1.0)          # braking untouched
        # Full-auto max also reins in auto-crawl and the pilot's own limit
        self.assertAlmostEqual(crawl.hi, 0.55)
        self.assertAlmostEqual(cfg.max_throttle, 0.55)
        hub.apply({"top_throttle": 0.4})                            # overall top wins over every mode
        self.assertAlmostEqual(cap.run(1.0, 0.0, "local_angle"), 0.4)

    def test_auto_crawl_toggle(self):
        hub, cfg, pilot, crawl, cap = make_hub()
        hub.apply({"auto_crawl": 0})
        self.assertIsNone(pilot.crawl)
        hub.apply({"auto_crawl": 1})
        self.assertIs(pilot.crawl, crawl)

    def test_save_load_revert(self):
        d = tempfile.mkdtemp()
        try:
            path = os.path.join(d, "tuning.json")
            hub, cfg, *_ = make_hub(tuning=path)
            default_kp = cfg.kp
            hub.apply({"kp": 5.0})
            hub.save()
            hub2, cfg2, *_ = make_hub(tuning=path)
            hub2.load_saved()
            self.assertAlmostEqual(cfg2.kp, 5.0)
            hub2.revert()
            self.assertAlmostEqual(cfg2.kp, default_kp)
            self.assertFalse(os.path.exists(path))
        finally:
            shutil.rmtree(d)


class TestDriveAndStops(unittest.TestCase):
    def test_web_joystick_dead_man(self):
        clk = Clock()
        hub, *_ = make_hub(clock=clk)
        part = webui.WebDrivePart(hub)
        self.assertEqual(part.run(0.1, 0.2), (0.1, 0.2))      # not held: gamepad passes through
        hub.set_drive(-0.5, 3.0)
        self.assertEqual(part.run(0.0, 0.0), (-0.5, 1.0))     # held, neutral physical pad: web values
        hub.set_drive(-0.5, 1.0)
        self.assertEqual(part.run(0.1, -0.2), (0.1, -0.2))   # physical pad takes priority
        self.assertIsNone(hub.web_drive())
        clk.t += 0.5                                          # no update for 0.5 s (tab closed, Wi-Fi lost)
        self.assertEqual(part.run(0.1, 0.2), (0.1, 0.2))

    def test_mode_request_is_one_shot(self):
        hub, *_ = make_hub()
        part = webui.WebModePart(hub)
        hub.request_mode("local")
        self.assertEqual(part.run("user"), "local")
        self.assertEqual(part.run("local_angle"), "local_angle")  # afterwards the controllers own it
        with self.assertRaises(ValueError):
            hub.request_mode("turbo")

    def test_stop_keeps_running_and_zeroes_forward(self):
        clk = Clock()
        hub, *_ = make_hub(clock=clk)

        class FakePad:
            stopped = False

            def emergency_stop(self):
                self.stopped = True
        hub.joystick = FakePad()
        hub.kill = KillSwitch(clock=clk)
        hub.set_drive(0, 1)
        hub.estop()
        self.assertTrue(hub.joystick.stopped)
        self.assertIsNone(hub.web_drive())
        self.assertEqual(webui.WebModePart(hub).run("local"), "user")
        guard = webui.WebGuardPart(hub)
        self.assertEqual(guard.run(0.5), 0.0)
        self.assertEqual(guard.run(-1.0), 0.0)                   # dashboard stop zeroes both directions
        self.assertIsNone(hub.kill.t_kill)                       # program NOT quitting
        clk.t += 1.1
        self.assertEqual(guard.run(0.5), 0.5)

    def test_quit_triggers_kill_switch(self):
        clk = Clock()
        hub, *_ = make_hub(clock=clk)
        hub.kill = KillSwitch(clock=clk)
        hub.off()
        self.assertIsNotNone(hub.kill.t_kill)

    def test_poweroff_refused_unless_on_car(self):
        hub, *_ = make_hub()
        with self.assertRaises(RuntimeError):
            hub.poweroff()


class TestLog(unittest.TestCase):
    def test_actions_are_logged_and_saved_to_file(self):
        d = tempfile.mkdtemp()
        try:
            hub, *_ = make_hub(tuning=None)
            hub.log_dir = d
            hub.apply({"kp": 4.0})
            hub.estop()
            texts = [e["text"] for e in hub.events_since(0)]
            self.assertTrue(any("KP (proportional)" in t and "-> 4" in t for t in texts))
            self.assertTrue(any("STOP pressed" in t for t in texts))
            self.assertIn("STOP pressed", hub.log_file_text())
            self.assertEqual(hub.events_since(hub.event_id), [])
        finally:
            shutil.rmtree(d)

    def test_transitions_logged_once_with_min_duration(self):
        clk = Clock()
        hub, cfg, pilot, *_ = make_hub(clock=clk)
        tp = webui.TelemetryPart(hub, clock=clk)
        tp._watch({"mode": "local_angle", "status": "TRACKING", "source": "left", "single_stop": False,
                   "throttle": 0.0, "motion": 3.0, "steering": 0.1, "web_driving": False}, clk.t)
        n0 = hub.event_id

        def frame(status):
            clk.t += 0.05
            tp._watch({"mode": "local_angle", "status": status, "source": "none" if status == "LOST" else "left",
                       "single_stop": False, "throttle": 0.0, "motion": 3.0, "steering": 0.1, "web_driving": False}, clk.t)
        frame("LOST"); frame("TRACKING")                     # one-frame flicker: not logged
        self.assertEqual(hub.event_id, n0)
        for _ in range(8):
            frame("LOST")                                    # 0.4 s lost: logged once
        frame("TRACKING")
        texts = [e["text"] for e in hub.events_since(n0)]
        self.assertEqual(texts[0], "Line lost")
        self.assertTrue(texts[1].startswith("Line found again after"))
        self.assertEqual(len(texts), 2)


class TestRuns(unittest.TestCase):
    def test_summary_finds_problems(self):
        rows = []
        for i in range(60):
            lost = 20 <= i < 35
            rows.append({"n": str(i), "t": str(100 + i * 0.05), "mode": "local", "found": "0" if lost else "1",
                         "source": "none" if lost else "left", "angle": "0.1", "throttle": "0.4", "motion": "3.0",
                         "single_stop": "1" if i == 50 else "0"})
        s = webui.summarize_run(rows, "20261108-185226")
        kinds = [e["kind"] for e in s["events"]]
        self.assertIn("lost", kinds)
        self.assertIn("single", kinds)
        self.assertAlmostEqual(s["tracked_pct"], 75.0)

    def test_error_stats(self):
        rows = [{"n": str(i), "t": str(i * 0.05), "mode": "local_angle", "found": "1", "source": "left",
                 "angle": "0", "throttle": "0", "error": str(e)} for i, e in enumerate([0.1, -0.1, 0.2, 0.0])]
        rows.append({"n": "4", "t": "0.2", "mode": "user", "found": "1", "source": "left", "angle": "0",
                     "throttle": "0", "error": "0.9"})                  # user-mode frames don't count
        e = webui.summarize_run(rows, "20261108-185226")["error"]
        self.assertAlmostEqual(e["mean_abs"], 0.1)
        self.assertAlmostEqual(e["max_abs"], 0.2)
        old = [{"n": "0", "t": "0", "mode": "local", "found": "1", "x_ratio": "0.30", "angle": "0", "throttle": "0"}]
        self.assertAlmostEqual(webui.summarize_run(old, "20261108-185226")["error"]["mean_abs"], 0.1)  # older runs

    def test_bad_names_rejected(self):
        hub, *_ = make_hub()
        with self.assertRaises(ValueError):
            hub.load_run("../../etc")
        with self.assertRaises(ValueError):
            hub.frame_path("../x", 1)


class TestHttp(unittest.TestCase):
    def test_api_roundtrip(self):
        d = tempfile.mkdtemp()
        try:
            run = os.path.join(d, "20261108-185226")
            os.makedirs(run)
            with open(os.path.join(run, "log.csv"), "w") as f:
                f.write("n,t,mode,found,source,conf,x_ratio,frame_age,angle,throttle,motion\n0,1.0,local,1,left,0.8,0.2,0,0.1,0.4,3\n")
            shutil.copy(os.path.join(ROOT, "tests", "frames", "live_001.jpg"), os.path.join(run, "000000.jpg"))
            hub, cfg, *_ = make_hub(data_dir=d)
            srv, _ = webui.start_server(hub, port=0)
            base = f"http://127.0.0.1:{srv.server_address[1]}"

            def post(path, body):
                req = urllib.request.Request(base + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
                return json.load(urllib.request.urlopen(req, timeout=3))
            state = json.load(urllib.request.urlopen(base + "/api/state", timeout=3))
            self.assertTrue(any(k["id"] == "kp" for k in state["knobs"]))
            self.assertEqual(post("/api/knobs", {"kp": 3.0})["applied"]["kp"], 3.0)
            self.assertAlmostEqual(cfg.kp, 3.0)
            self.assertTrue(post("/api/mode", {"mode": "local"})["ok"])
            runs = json.load(urllib.request.urlopen(base + "/api/runs", timeout=3))
            self.assertEqual(runs[0]["name"], "20261108-185226")
            img = urllib.request.urlopen(base + "/runs/20261108-185226/0.jpg", timeout=3).read()
            self.assertTrue(img.startswith(b"\xff\xd8"))
            csvb = urllib.request.urlopen(base + "/runs/20261108-185226/log.csv", timeout=3).read()
            self.assertTrue(csvb.startswith(b"n,t,mode"))
            logs = json.load(urllib.request.urlopen(base + "/api/log?since=0", timeout=3))
            self.assertTrue(any("Pilot steers" in e["text"] or "Full auto" in e["text"] for e in logs))
            page = urllib.request.urlopen(base + "/", timeout=3).read()
            self.assertIn(b"Lap Tuner", page)
            classic = urllib.request.urlopen(base + "/classic", timeout=3).read()
            self.assertIn(b"Line Follower", classic)
            srv.shutdown()
        finally:
            shutil.rmtree(d)


if __name__ == "__main__":
    unittest.main()
