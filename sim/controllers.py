"""Controllers to compare. All use the REAL pilot code from the car (pi/donkey_part.py
+ linetrack/controller.py) with different settings, plus the car's real speed cap."""
from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(ROOT, "pi"), ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

import line_config  # noqa: E402
from donkey_part import (AutoCrawl, LineDetectorPart, LineFollowerPilot, SpeedPlan,  # noqa: E402
                         car_model, load_line_config, make_throttle_cap)
from linetrack.geometry import GroundModel, fit_bend  # noqa: E402


# The KP/KD tools (gain schedule in sim/tune.py, the dashboard advisor, the classic auto-tune
# strategies and their backtests) study PD steering: with pure pursuit and no PD share, KP and KD do
# nothing. They start from these overrides so they keep analysing the PD law, whatever the car's default.
PD_STEERING = {"pure_pursuit": False, "target_x_ratio": 0.40, "steering_smoothing": 0.35}


class PilotController:
    """The car's pilot in Full auto, with optional setting overrides.

    speed: None -> as line_config (auto-crawl), float -> fixed throttle command,
           (straight, corner) -> speed plan.
    """

    def __init__(self, name="current", overrides=None, speed=None, cap=True, clock=None):
        self.name = name
        self.cfg, ex = load_line_config(line_config)
        for k, v in (overrides or {}).items():
            if k.startswith(("speed_", "pilot_")) or k == "auto_crawl":
                continue
            setattr(self.cfg, k, v)
        self.clock = clock
        plan = None
        if speed is None and ex["SPEED_STRAIGHT"] > 0:
            plan = SpeedPlan(ex["SPEED_STRAIGHT"], ex["SPEED_CORNER"], transition=ex["SPEED_TRANSITION"])
        elif isinstance(speed, (int, float)):
            plan = SpeedPlan(float(speed), float(speed))
        elif isinstance(speed, (tuple, list)):
            plan = SpeedPlan(float(speed[0]), float(speed[1]))
        if plan is not None:
            options = overrides or {}
            plan = SpeedPlan(plan.straight, plan.corner, transition=options.get("speed_transition", plan.transition),
                             ramp_down=options.get("speed_ramp_down", ex["SPEED_RAMP_DOWN"]), ramp_up=options.get("speed_ramp_up", ex["SPEED_RAMP_UP"]))
        if plan is not None:
            self.cfg.max_throttle = max(self.cfg.max_throttle, plan.straight)
        self.crawl = AutoCrawl(start=ex["CRAWL_START"], hi=max(ex["CRAWL_MAX"], plan.straight if plan else 0))
        self.pilot = LineFollowerPilot(self.cfg, clock=clock, crawl=self.crawl, speed_plan=plan,
                                       coast_s=(overrides or {}).get("pilot_coast_s", ex["LOST_COAST_S"]),
                                       slow_s=(overrides or {}).get("pilot_slow_s", ex["LOST_SLOW_S"]),
                                       single_line_stop_s=(overrides or {}).get("pilot_single_line_stop_s", 0.0))
        if not (overrides or {}).get("auto_crawl", True):
            self.pilot.crawl = None
        self.cap = make_throttle_cap(self.cfg, ex) if cap else None
        if self.cap is not None and plan is not None:
            # the sweep asks for this speed explicitly: lift Max: Full auto to allow it (top speed still applies)
            self.cap.mode_caps["local"] = max(self.cap.mode_caps.get("local", 1.0), plan.straight)
            self.cap.top = max(self.cap.top, plan.straight)
        self.det = None
        # the pilot's corner geometry uses the NOMINAL car model (line_config / CarModel defaults),
        # even when the simulated car differs: exactly the mismatch the real car will have
        self.model = car_model(ex)
        for k in ("grip_mu", "steer_max_deg", "brake_decel", "delay_s"):   # what the PILOT believes (tests)
            if k in (overrides or {}):
                setattr(self.model, k, float(overrides[k]))
        self.pilot.model = self.model
        self.ground = None

    @property
    def needs_edges(self) -> bool:
        """Corner geometry is only used with feedforward or corner speed planning on."""
        return True  # Section gains and speed planning also require geometry.

    def bend_from_edges(self, edges):
        if edges is None or not len(edges):
            return None
        if self.ground is None:
            self.ground = GroundModel(self.model)
        return fit_bend(self.ground.project(edges, self.model.image_w, self.model.image_h))

    def path_from_edges(self, edges, source="left"):
        """Edge ground points for pure pursuit (near -> far), like LineDetectorPart.path."""
        if edges is None or len(edges) < 2:
            return None
        if self.ground is None:
            self.ground = GroundModel(self.model)
        pts = np.asarray(edges, float)
        pts = pts[np.argsort(-pts[:, 1])]
        return self.ground.project(pts, self.model.image_w, self.model.image_h), source, self.cfg.lane_width_ratio

    def step(self, x, found, both, age, motion, image=None, edges=None, source="left"):
        if image is not None:                         # rendered mode: run the real detector too
            import cv2
            if self.det is None:
                self.det = LineDetectorPart(self.cfg, clock=self.clock, show_overlay=False, model=self.model)
            x, found, age, _, motion_img, both = self.det.run(image)
            bend, path = self.det.bend, self.det.path
        else:
            bend = self.bend_from_edges(edges) if found else None
            path = self.path_from_edges(edges, source) if found else None
        angle, thr = self.pilot.run("local", x, found, age, motion, both, bend=bend, path=path)
        if self.cap is not None:
            thr = self.cap.run(thr, angle, "local")
        return angle, thr, self.pilot.last_cmd


VARIANTS = {
    "current": {},
    "P only (KD 0)": {"overrides": {"kd": 0.0}},
    "KP 2.2 (old)": {"overrides": {"kp": 2.2, "kd": 0.25}},
    "KP 5": {"overrides": {"kp": 5.0}},
    "no smoothing": {"overrides": {"steering_smoothing": 0.0}},
    "look further (row 0.55)": {"overrides": {"lookahead_ratio": 0.55}},
}
