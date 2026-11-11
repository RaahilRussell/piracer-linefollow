"""Line-follower dashboard: live video, live PID telemetry, tuning knobs, web
driving, and a browser for past recorded runs.

Runs inside drive_line.py on the Pi (port 8890, next to Donkeycar's own 8887).
Standard library only (http.server + Server-Sent Events), so it adds nothing to
install. Everything it writes stays in ~/linefollow (tuning.json; runs are read
from ~/linefollow/data).

Threads: the Donkeycar loop calls the *Part classes 20x/s; the HTTP server runs
in its own daemon threads. They share one Hub, guarded by a lock.
"""
from __future__ import annotations

import csv
import json
import os
import re
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

import line_config
import tuning

MODES = ("user", "local_angle", "local")
MODE_NAMES = {"user": "User", "local_angle": "Pilot steers", "local": "Full auto", None: "?"}
RUN_NAME = re.compile(r"^\d{8}-\d{6}$")


# --------------------------------------------------------------------- knobs
# id, group, label, min, max, step, help. Ranges are hard limits: the server
# clamps every value, so nothing typed in a browser can exceed them.
KNOBS = [
    ("top_throttle", "Speed limits", "Overall top speed", 0.0, 1.0, 0.01, "Hard ceiling for every mode, gamepad included."),
    ("forward_pwm", "Speed limits", "Motor PWM at full throttle", 390, 460, 1, "PWM the ESC gets at throttle 1.0 (stopped = 370). ~/mycar uses 407. Higher makes EVERY throttle value faster, gamepad included: lower the curve speed to match. Live; hard limit 460."),
    ("max_user", "Speed limits", "Max: User mode", 0.0, 1.0, 0.01, "You drive (gamepad or web joystick)."),
    ("max_pilot_steers", "Speed limits", "Max: Pilot steers", 0.0, 1.0, 0.01, "Pilot steers, you give throttle (A once)."),
    ("max_auto", "Speed limits", "Max: Full auto", 0.0, 1.0, 0.01, "Pilot steers and drives (A twice). Also the auto-crawl ceiling."),
    ("kp", "Steering", "KP (proportional)", 0.0, 10.0, 0.1, "Steer this much per unit of error. 0.1 error x KP 3.5 = 0.35 steer."),
    ("kd", "Steering", "KD (derivative)", 0.0, 2.0, 0.05, "Damps weaving by reacting to how fast the error changes."),
    ("straight_kp", "Section gains", "Straight KP", 0.0, 10.0, 0.1, "Gain used on straights. 0 = shared KP."),
    ("straight_kd", "Section gains", "Straight KD", 0.0, 2.0, 0.05, "Damping used on straights. 0 = shared KD."),
    ("corner_kp", "Section gains", "Corner KP", 0.0, 10.0, 0.1, "Gain used in curves. 0 = shared KP."),
    ("corner_kd", "Section gains", "Corner KD", 0.0, 2.0, 0.05, "Damping used in curves. 0 = shared KD."),
    ("corner_blend_start", "Section gains", "Corner blend starts", 0.0, 3.0, 0.05, "Curvature where corner gains begin blending in."),
    ("corner_blend_full", "Section gains", "Corner blend completes", 0.05, 3.0, 0.05, "Curvature where corner gains are fully active."),
    ("corner_exit_steer", "Section gains", "Curve exit hold", 0.0, 1.0, 0.05, "Stay in curve gains and curve speed until steering unwinds below this fraction of max. 0 = off."),
    ("target_x_ratio", "Steering", "Target position", 0.05, 0.95, 0.01, "Where the left line's inside edge should sit (fraction of image width)."),
    ("lookahead_ratio", "Steering", "Lookahead row", 0.45, 0.95, 0.01, "Image row where the error is measured. Lower = further ahead."),
    ("max_steering", "Steering", "Max steering", 0.2, 1.0, 0.05, "Steering is clipped to +/- this."),
    ("steering_smoothing", "Steering", "Smoothing", 0.0, 0.9, 0.05, "0 = raw, 0.9 = very smooth (slower to react)."),
    ("steering_rate_limit", "Steering", "Rate limit /s", 0.5, 10.0, 0.5, "Max change in steering per second."),
    ("ff_gain", "Steering", "Corner feedforward", 0.0, 1.0, 0.05, "PD steering only: steer by the corner's measured curvature (atan(wheelbase/R)) so PD only corrects what's left. 0 = off. In the simulator it runs wide at curve exits above 0.90 throttle."),
    ("pure_pursuit", "Steering law", "Pure pursuit", 0, 1, 1, "On: aim at a point of the ride line a lookahead distance ahead. Off: PD on the lookahead row (+ feedforward). Simulator: pure pursuit is clean at higher speed. Not yet driven on the real car."),
    ("pursuit_ld_m", "Steering law", "Pursuit lookahead (m)", 0.2, 1.2, 0.05, "Distance ahead of the rear axle that pure pursuit aims at, at standstill. Shorter = follows tighter but twitchier."),
    ("pursuit_ld_gain_s", "Steering law", "Lookahead per m/s", 0.0, 0.6, 0.05, "Extra lookahead per m/s of modelled speed, so it looks further ahead when fast."),
    ("pursuit_pd", "Steering law", "PD share with pursuit", 0.0, 1.0, 0.05, "Share of KP/KD added on top of pure pursuit as camera-error feedback. 0.15 = default mix; 0 = pursuit alone; 0.3+ ran wide at race speed in the simulator."),
    ("steer_max_deg", "Steering law", "Full-lock wheel angle (deg)", 10.0, 30.0, 0.5, "Wheel angle at full steering. 18 is fitted to recorded runs, not measured. Used by pure pursuit and feedforward; the simulator uses the same value."),
    ("pursuit_predict_s", "Steering law", "Delay compensation (s)", 0.0, 0.3, 0.01, "Pure pursuit aims from where the car will be this long after the camera frame (it keeps moving while the frame is processed and the servo turns). 0 = off. Logs suggest about 0.15 s from command to motion."),
    ("pursuit_turn_in", "Steering law", "Curve turn-in", 0.0, 1.0, 0.05, "On curve entry, pull steering this share of the way toward what the visible curve needs, when pure pursuit is turning in less (it aims at where the inner line leaves the image, so it turns in late). Only ever adds turn into the curve. 0 = off."),
    ("racing_steer_max_deg", "Racing line", "Racing Line steering model (deg)", 10.0, 30.0, 0.1, "Only used when Racing Line mode is enabled. Middle profile: 13.4 deg. Legacy Race stays on the nominal 18 deg steering model."),
    ("racing_line", "Racing line", "Racing Line mode", 0, 1, 1, "Default physical racing line: pure pursuit stays a little wide on the straights, then cuts toward the inside tape through each curve. Works from either orange line."),
    ("racing_apex_m", "Racing line", "Curve distance from inside tape (m)", -0.25, 0.15, 0.005, "Where the car's centre aims in a curve, measured from the inside tape's lane-side edge. Lower = cuts in more = shorter lap. The simulated car ends up about 3 cm further in than this, so 0 already straddles the tape and below 0 it loses the line. 0.07 = inner wheels on the tape edge. Raise it if the car loses the inside line."),
    ("racing_straight_m", "Racing line", "Straight distance from inside tape (m)", -0.05, 0.25, 0.005, "The same distance on the straights. A little wider than the curve distance opens the turn, so the steering is not pinned at full lock. Lower = shorter path but more time at full lock."),
    ("racing_turn_in_m", "Racing line", "Turn-in distance before the curve (m)", 0.2, 1.5, 0.05, "The car stays on the wide straight line until the curve starts this close ahead, then goes for the inside tape and holds the cut right through the curve. 1.5 = cut in the moment a curve is visible (it then creeps inside along the straight). Lower = later, harder turn-in. Too low and it only reaches the inside at the exit."),
    ("conf_slow_below", "Full auto speed", "Slow when detection below", 0.0, 0.9, 0.05, "Below this line-detection confidence, throttle blends down to curve speed (reaching it at Min confidence). Real tracking is usually 0.7-0.85. 0 = off."),
    ("cam_height_m", "Steering law", "Camera height (m)", 0.08, 0.22, 0.005, "Lens centre above the floor. 0.15 is fitted to real race-mat frames (lane width, curve radius, line spacing); a ruler beats it. Pure pursuit aims through this: set too high and the car runs wide. 0.12 (with tilt 15) made it cut every curve."),
    ("cam_pitch_deg", "Steering law", "Camera tilt down (deg)", 0.0, 30.0, 0.5, "How far the camera's view points down from horizontal. 8 is fitted to real race-mat frames (the two tape edges come out parallel); 15 made them converge 4x more. Measure with a phone level. Set too low and pure pursuit runs wide."),
    ("speed_straight", "Full auto speed", "Straight speed (plan)", 0.0, 1.0, 0.01, "0 = off. Fast target on a settled straight. Bends and large corrections reduce it."),
    ("speed_transition", "Full auto speed", "Transition speed (plan)", 0.0, 1.0, 0.01, "Speed while a bend is building. 0 = midpoint of straight and corner speed."),
    ("speed_corner", "Full auto speed", "Corner speed (plan)", 0.0, 1.0, 0.01, "Lower target in a bend or large recovery. Never above straight speed. Keep high enough to roll."),
    ("speed_ramp_down", "Full auto speed", "Slowdown rate /s", 0.05, 2.0, 0.05, "How quickly throttle falls as curvature builds. Faster is safer; too fast can feel abrupt."),
    ("speed_ramp_up", "Full auto speed", "Speed-up rate /s", 0.05, 1.0, 0.05, "How quickly throttle returns after the curve is genuinely over."),
    ("auto_crawl", "Full auto speed", "Auto-crawl", 0, 1, 1, "On: find the slowest speed that keeps rolling. Off: fixed pilot throttle."),
    ("crawl_start", "Full auto speed", "Crawl start", 0.2, 0.7, 0.01, "Auto-crawl starts here, then creeps up until the camera sees motion."),
    ("crawl_ramp", "Full auto speed", "Crawl ramp /s", 0.02, 0.4, 0.01, "How fast auto-crawl raises throttle while not moving."),
    ("throttle", "Full auto speed", "Fixed pilot throttle", 0.0, 1.0, 0.01, "Pilot throttle in local mode when auto-crawl is off."),
    ("corner_plan", "Full auto speed", "Corner speed planning", 0, 1, 1, "On: cap speed so the car can brake to the corner's grip limit (sqrt(grip*g*R)) in time. Needs measured grip; adds little in the simulator so far."),
    ("corner_margin", "Full auto speed", "Corner speed margin", 0.5, 1.0, 0.05, "Fraction of the grip-limited corner speed to allow."),
    ("lap_sections", "Lap timer", "Straight/curve changes per lap", 2, 16, 1, "4 for an oval (straight, curve, straight, curve). Count yours once and set it here."),
    ("race_laps", "Lap timer", "Laps in a race", 1, 30, 1, "The race total adds up this many of the latest laps (competition: 10)."),
    ("lap_debounce_s", "Lap timer", "Section must hold (s)", 0.0, 1.0, 0.05, "A straight/curve change only counts once the new section has held this long, so a brief flicker is not a change. 0.25 missed the short straights at race speed (Nov 10 night): 0.10."),
    ("lap_min_s", "Lap timer", "Shortest possible lap (s)", 0.5, 10.0, 0.1, "Changes that add up to a lap faster than this are treated as flicker and ignored."),
    ("track_length_m", "Lap timer", "Track length (m)", 1.0, 30.0, 0.01, "Length of the inside line, for the lap-time estimate. 4.98 is traced from the Waveshare mat."),
    ("track_curve_share", "Lap timer", "Share of the track that is curve", 0.0, 1.0, 0.01, "For the lap-time estimate. 0.71 is traced from the Waveshare mat."),
    ("tune_race_first", "Lap tuner rules", "Start with the race values", 0, 1, 1, "On: Start puts the race values (the defaults in line_config.py) on the car and checks them before climbing. Off: it starts from whatever the sliders say."),
    ("tune_step", "Lap tuner rules", "Faster each lap by", 0.01, 0.12, 0.01, "Straight throttle added after a clean lap."),
    ("tune_curve_ratio", "Lap tuner rules", "Curve step share", 0.0, 1.0, 0.05, "Curve step = straight step x this. 0 = curves never speed up."),
    ("tune_lost_grace_s", "Lap tuner rules", "Line failure after (s)", 0.0, 1.5, 0.05, "The line must be gone this long before it counts as a line failure."),
    ("tune_min_tracked", "Lap tuner rules", "Min line tracked", 0.5, 1.0, 0.01, "A section fails if the line is tracked less than this share of the time."),
    ("tune_curve_sat", "Lap tuner rules", "Curve: full lock allowed", 0.1, 1.0, 0.05, "Share of a curve the steering may sit at full lock and still pass. Full lock in curves is normal."),
    ("tune_straight_sat", "Lap tuner rules", "Straight: full lock allowed", 0.05, 1.0, 0.05, "Share of a straight at full lock that means it is going too fast."),
    ("tune_weave", "Lap tuner rules", "Weave limit (swings/s)", 0.3, 2.0, 0.05, "Faster weaving than this lowers KP."),
    ("tune_evidence", "Lap tuner rules", "Samples to pass a section", 10, 120, 1, "How much driving each section needs before it can pass (20 samples = 1 s)."),
    ("tune_kp_max", "Lap tuner rules", "Max KP the tuner may set", 1.0, 8.0, 0.1, "The tuner never raises KP above this. Above 3.5 caused curve crashes in testing."),
    ("tune_confirm_laps", "Lap tuner rules", "Clean laps before faster", 1, 5, 1, "Clean laps in a row needed at a speed before the next step. More = fewer surprises on race day."),
    ("tune_margin_steps", "Lap tuner rules", "Safety margin (steps)", 0, 3, 1, "When a section hits its limit, race it this many steps below its last clean speed."),
    ("tune_max_retunes", "Lap tuner rules", "Gain fixes before a limit", 0, 6, 1, "How many KP/KD adjustments a section gets before its speed counts as the limit."),
    ("s_lo", "Detection", "Min saturation", 60, 200, 1, "Orange filter. Tape ~151, wood floor ~53 (up to 150)."),
    ("v_lo", "Detection", "Min brightness", 60, 230, 1, "Orange filter brightness floor."),
    ("min_confidence", "Detection", "Min confidence", 0.1, 0.9, 0.01, "Below this the line counts as lost."),
    ("single_line_stop_s", "Safety", "One-line stop after (s)", 0.0, 10.0, 0.5, "Rolling with one line this long -> throttle 0. 0 = off."),
    ("lost_timeout_s", "Safety", "Hold steering when lost (s)", 0.0, 2.0, 0.1, "Then straightens (unless Hold heading is on)."),
    ("lost_hold_heading", "Safety", "Hold heading when lost", 0, 1, 1, "1 = keep the last steering while blind (backtested: better)."),
    ("coast_s", "Safety", "Coast when lost (s)", 0.0, 2.0, 0.1, "Full auto keeps its speed this long after losing the line."),
    ("slow_s", "Safety", "Then slow to a stop over (s)", 0.0, 2.0, 0.1, "0 + 0 = cut throttle the moment the line is lost."),
]
KNOB_IDS = {k[0] for k in KNOBS}
KNOB_LABELS = {k[0]: k[2] for k in KNOBS}
INT_KNOBS = {"forward_pwm", "s_lo", "v_lo", "auto_crawl", "lost_hold_heading", "corner_plan", "pure_pursuit", "racing_line", "tune_race_first", "tune_evidence", "tune_max_retunes", "tune_confirm_laps", "tune_margin_steps", "lap_sections", "race_laps"}

# What a preset sets: everything that decides how the car steers and how fast it goes. Presets are
# built on the server from line_config.py (Hub.presets), so the page never hard-codes a tuning value.
RACE_KEYS = ("speed_straight", "speed_corner", "speed_transition", "speed_ramp_down", "speed_ramp_up",
             "kp", "kd", "straight_kp", "straight_kd", "corner_kp", "corner_kd",
             "target_x_ratio", "lookahead_ratio", "max_steering", "steering_smoothing", "steering_rate_limit",
             "ff_gain", "pure_pursuit", "pursuit_ld_m", "pursuit_ld_gain_s", "pursuit_pd", "pursuit_predict_s", "pursuit_turn_in", "steer_max_deg", "conf_slow_below",
             "racing_line", "racing_steer_max_deg", "racing_apex_m", "racing_straight_m", "racing_turn_in_m",
             "corner_blend_start", "corner_blend_full", "corner_exit_steer", "corner_plan",
             "lost_hold_heading", "coast_s", "slow_s", "max_auto", "top_throttle", "forward_pwm")
# Changes each preset makes to the race values.
def race_speeds(ex) -> dict:
    """The Race preset's speeds from line_config (RACE_SPEED_*); missing ones fall back to the start-up speeds."""
    return {k: ex.get("RACE_" + k.upper()) for k in ("speed_straight", "speed_transition", "speed_corner", "forward_pwm")}


def safe_speeds(ex) -> dict:
    """The Safe race preset's speeds from line_config (SAFE_SPEED_*)."""
    return {k: ex.get("SAFE_" + k.upper()) for k in ("speed_straight", "speed_transition", "speed_corner", "forward_pwm")}


SLOW_LAP = {"speed_straight": 0.75, "speed_corner": 0.70, "speed_transition": 0.75, "forward_pwm": 407}   # speeds driven at PWM 407
# PD + no feedforward: the steering that has driven the real car (Nov 9, at 0.70-0.75). With the Nov 10
# camera estimate (0.12 m / 15 deg) 0.75 / 0.70 is the fastest speed that never ran wide in any simulated
# variation (0.80 / 0.75 fails with delay; 0.90 / 0.85 fails almost everywhere). Target 0.40: PD cuts curves.
PD_FALLBACK = {"pure_pursuit": 0, "ff_gain": 0.0, "target_x_ratio": 0.40, "steering_smoothing": 0.35,
               "speed_straight": 0.75, "speed_corner": 0.70, "speed_transition": 0.75, "forward_pwm": 407}


# The saved best physical setup, also used by the default dashboard preset.
RACING_LINE_SETUP = dict(line_config.BEST_MODEL_SETUP)


def defaults_fingerprint(values: dict) -> str:
    """Short stable id of a set of default knob values (order and float noise don't matter)."""
    import hashlib
    norm = {k: round(float(v), 6) for k, v in values.items() if isinstance(v, (int, float, bool))}
    return hashlib.sha1(json.dumps(norm, sort_keys=True).encode()).hexdigest()[:12]


class Hub:
    """Everything the car loop and the web pages share."""

    def __init__(self, cfg, pilot, det, crawl=None, cap=None, data_dir="data", tuning_path=None,
                 clock=time.monotonic, log_dir=None, track_length_m=4.98, track_curve_share=0.71, race_speeds=None, safe_speeds=None):
        # Race preset speeds when they differ from the start-up speeds (line_config RACE_SPEED_*)
        self.race_speeds = {k: float(v) for k, v in (race_speeds or {}).items() if v is not None}
        self.safe_speeds = {k: float(v) for k, v in (safe_speeds or {}).items() if v is not None}
        self.cfg, self.pilot, self.det, self.crawl, self.cap = cfg, pilot, det, crawl, cap
        self.battery_monitor = None  # optional asynchronous INA219 reader; never drives throttle decisions
        self.data_dir = data_dir
        self.tuning_path = tuning_path
        self.clock = clock
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        # live data
        self.seq = 0
        self.telemetry: dict = {}
        self.history: list = []          # last ~10 s of telemetry for late-joining pages
        self.jpeg: bytes | None = None
        self.jpeg_seq = 0
        self.hardware_test = {"state": "idle", "result": None, "error": None}
        # commands from the web
        self.pending_mode: str | None = None
        self.drive = {"steering": 0.0, "throttle": 0.0, "t": -1e9}
        self.drive_timeout_s = 0.4       # web joystick dead-man: no update for this long -> released
        self.estop_until = -1e9
        self.joystick = None             # Donkeycar gamepad controller (for its E-stop)
        self.kill = None                 # KillSwitch (gamepad X / web OFF)
        self.allow_poweroff = False      # only drive_line.py on the Pi turns this on
        self.recording_review = None
        self.autotuner = None            # AutoTuner (drive_line.py / sim set it)
        from lap_tuning import LapTimer
        self.laptimer = LapTimer()
        # traced Waveshare mat (sim/tracks/waveshare_inner_line.csv); only used for the lap-time estimate
        self.track_length_m, self.track_curve_share = float(track_length_m), float(track_curve_share)
        self._ground = None
        self.profiles = self._load_profiles()   # speed profiles from the simulator (gain_schedule.json)
        self.health = None               # last live tuning-health result (dict)
        self._health_t = -1e9
        # where the data comes from: "car" (drive_line.py on the Pi), "simulator" or "replay"
        # (pi/sim_dashboard.py). The page only says "live" for the car with fresh telemetry.
        self.source = "car"
        self.source_detail = ""
        # event log: last 500 in memory (for the page) + one file per day
        self.log_lock = threading.Lock()
        self.events: list = []
        self.event_id = 0
        self.log_dir = log_dir
        if cap is not None:  # keep the pilot's own limits in line with the Full-auto max
            self._set_auto_max(cap.limit("local"))
        self.defaults = self.knob_values()
        # Fingerprint of the line_config.py defaults this program started with. tuning.json records it, so a
        # file saved under OLDER defaults (before a git pull + deploy) is set aside instead of silently
        # overriding the new ones.
        self.defaults_id = defaults_fingerprint(self.defaults)

    # ------------------------------------------------------------ knobs
    def knob_values(self) -> dict:
        c, cr = self.cfg, self.crawl
        v = {
            "kp": c.kp, "kd": c.kd, "target_x_ratio": c.target_x_ratio, "lookahead_ratio": c.lookahead_ratio,
            "straight_kp": c.straight_kp or c.kp, "straight_kd": c.straight_kd or c.kd,
            "corner_kp": c.corner_kp or c.kp, "corner_kd": c.corner_kd or c.kd,
            "corner_blend_start": c.corner_blend_start, "corner_blend_full": c.corner_blend_full, "corner_exit_steer": getattr(c, "corner_exit_steer", 0.0),
            "max_steering": c.max_steering, "steering_smoothing": c.steering_smoothing,
            "steering_rate_limit": c.steering_rate_limit, "throttle": c.throttle,
            "s_lo": c.s_lo, "v_lo": c.v_lo, "min_confidence": c.min_confidence,
            "lost_timeout_s": c.lost_timeout_s,
            "ff_gain": c.ff_gain, "corner_plan": int(bool(c.corner_plan)), "corner_margin": c.corner_margin,
            "pure_pursuit": int(bool(c.pure_pursuit)), "pursuit_ld_m": c.pursuit_ld_m,
            "pursuit_ld_gain_s": c.pursuit_ld_gain_s, "pursuit_pd": c.pursuit_pd, "pursuit_predict_s": c.pursuit_predict_s, "pursuit_turn_in": c.pursuit_turn_in, "conf_slow_below": c.conf_slow_below, "steer_max_deg": c.steer_max_deg,
            "racing_line": int(bool(c.racing_line)), "racing_steer_max_deg": c.racing_steer_max_deg, "racing_apex_m": c.racing_apex_m, "racing_turn_in_m": c.racing_turn_in_m, "racing_straight_m": c.racing_straight_m,
            "cam_height_m": self._model().cam_height_m, "cam_pitch_deg": self._model().cam_pitch_deg,
            "forward_pwm": int(round(self._model().pwm_full)),
            "single_line_stop_s": getattr(self.pilot, "single_line_stop_s", 0.0),
            "lost_hold_heading": int(bool(c.lost_hold_heading)),
            "coast_s": getattr(self.pilot, "coast_s", 0.0), "slow_s": getattr(self.pilot, "slow_s", 0.0),
            "auto_crawl": int(self.pilot is not None and self.pilot.crawl is not None),
            "top_throttle": self.cap.top if self.cap is not None else 1.0,
            "speed_straight": self.pilot.speed_plan.straight if getattr(self.pilot, "speed_plan", None) else 0.0,
            "speed_transition": self.pilot.speed_plan.transition if getattr(self.pilot, "speed_plan", None) else 0.0,
            "speed_corner": self.pilot.speed_plan.corner if getattr(self.pilot, "speed_plan", None) else 0.0,
            "speed_ramp_down": self.pilot.speed_plan.ramp_down if getattr(self.pilot, "speed_plan", None) else 1.0,
            "speed_ramp_up": self.pilot.speed_plan.ramp_up if getattr(self.pilot, "speed_plan", None) else 0.4,
        }
        v.update(lap_sections=self.laptimer.sections_per_lap, race_laps=self.laptimer.race_laps,
                 lap_debounce_s=self.laptimer.debounce_s, lap_min_s=self.laptimer.min_lap_s,
                 track_length_m=self.track_length_m, track_curve_share=self.track_curve_share)
        at = self.autotuner
        if at is not None:
            v.update(tune_race_first=int(bool(at.race_first)))
            v.update(tune_step=getattr(at, "step", 0.04), tune_curve_ratio=at.curve_step_ratio,
                     tune_lost_grace_s=at.lost_grace_s, tune_min_tracked=at.min_tracked,
                     tune_curve_sat=at.curve_sat, tune_straight_sat=at.straight_sat, tune_weave=at.weave_fast,
                     tune_evidence=at.evidence, tune_max_retunes=at.max_retunes,
                     tune_confirm_laps=at.confirm_laps, tune_margin_steps=at.margin_steps, tune_kp_max=at.kp_max)
        if self.cap is not None:
            v.update(max_user=self.cap.mode_caps.get("user", self.cap.top),
                     max_pilot_steers=self.cap.mode_caps.get("local_angle", self.cap.top),
                     max_auto=self.cap.mode_caps.get("local", self.cap.top))
        if cr is not None:
            v.update(crawl_start=cr.start, crawl_ramp=cr.ramp_up)
        return v

    def _model(self):
        """The camera/car model the pilot and detector project through (shared object on the car)."""
        from linetrack.geometry import CarModel
        return getattr(self.pilot, "model", None) or getattr(self.det, "model", None) or CarModel()

    def attach_throttle(self, part):
        """The donkeycar PWMThrottle (drive_line.py, after add_drivetrain): its forward PWM follows the slider."""
        self.throttle_part = part
        self.set_forward_pwm(self._model().pwm_full)

    def set_forward_pwm(self, pwm):
        """PWM at throttle 1.0, live. The car model follows it so speed estimates (lookahead, lap estimate)
        stay in step. Clamped to the slider's hard range; the stopped and reverse PWMs are never touched."""
        spec = next(k for k in KNOBS if k[0] == "forward_pwm")
        pwm = int(round(min(max(float(pwm), spec[3]), spec[4])))
        models = {id(m): m for m in (getattr(self.pilot, "model", None), getattr(self.det, "model", None)) if m is not None}
        for m in models.values():
            m.pwm_full = float(pwm)
        part = getattr(self, "throttle_part", None)
        if part is not None:
            part.max_pulse = pwm
        return pwm

    def set_camera(self, **values):
        """Change the camera mount (height/tilt) live. The ground projection is rebuilt here and swapped
        in whole, so the car loop never sees a half-built or missing model."""
        from linetrack.geometry import GroundModel
        models = {id(m): m for m in (getattr(self.pilot, "model", None), getattr(self.det, "model", None)) if m is not None}
        for m in models.values():
            for k, v in values.items():
                setattr(m, k, float(v))
        for m in models.values():
            g = GroundModel(m)
            for part in (self.pilot, self.det):
                if part is not None and getattr(part, "model", None) is m:
                    part.ground = g
        self._ground = None

    def set_knob(self, kid: str, value) -> float:
        if kid not in KNOB_IDS:
            raise KeyError(kid)
        spec = next(k for k in KNOBS if k[0] == kid)
        x = float(value)
        if not np.isfinite(x):
            raise ValueError("not a number")
        x = min(max(x, spec[3]), spec[4])
        if kid in INT_KNOBS:
            x = int(round(x))
        c = self.cfg
        if kid in ("kp", "kd", "straight_kp", "straight_kd", "corner_kp", "corner_kd", "corner_blend_start", "corner_blend_full", "corner_exit_steer",
                   "target_x_ratio", "lookahead_ratio", "max_steering", "steering_smoothing",
                   "steering_rate_limit", "s_lo", "v_lo", "min_confidence", "lost_timeout_s", "ff_gain", "corner_margin",
                   "pursuit_ld_m", "pursuit_ld_gain_s", "pursuit_pd", "pursuit_predict_s", "pursuit_turn_in", "steer_max_deg", "racing_steer_max_deg", "conf_slow_below", "racing_apex_m", "racing_straight_m", "racing_turn_in_m"):
            setattr(c, kid, x)
        elif kid in ("cam_height_m", "cam_pitch_deg"):
            self.set_camera(**{kid: x})
        elif kid == "forward_pwm":
            self.set_forward_pwm(x)
        elif kid == "throttle":
            c.throttle = min(x, c.max_throttle)
            x = c.throttle
        elif kid == "single_line_stop_s":
            self.pilot.single_line_stop_s = x
        elif kid in ("lost_hold_heading", "corner_plan", "pure_pursuit", "racing_line"):
            setattr(c, kid, bool(x))
        elif kid in ("coast_s", "slow_s"):
            setattr(self.pilot, kid, x)
        elif kid in ("speed_straight", "speed_transition", "speed_corner", "speed_ramp_down", "speed_ramp_up"):
            from donkey_part import SpeedPlan
            plan = self.pilot.speed_plan
            # Let either section speed slider be adjusted first. A corner value
            # entered before a straight plan exists should still create a plan,
            # using the current Full Auto ceiling for the straight target.
            default_straight = (min(self.cap.mode_caps.get("local", self.cap.top), self.cap.top)
                                if self.cap is not None else self.cfg.max_throttle)
            st = x if kid == "speed_straight" else (plan.straight if plan else
                 (default_straight if kid == "speed_corner" and x > 0 else 0.0))
            tr = x if kid == "speed_transition" else (plan.transition if plan else 0.0)
            co = x if kid == "speed_corner" else (plan.corner if plan else 0.0)
            down = x if kid == "speed_ramp_down" else (plan.ramp_down if plan else 1.0)
            up = x if kid == "speed_ramp_up" else (plan.ramp_up if plan else 0.4)
            self.pilot.speed_plan = SpeedPlan(st, co or st, transition=tr or None,
                                              ramp_down=down, ramp_up=up) if st > 0 else None
            if self.pilot.speed_plan is not None:
                attr = {"speed_straight": "straight", "speed_corner": "corner", "speed_transition": "transition",
                        "speed_ramp_down": "ramp_down", "speed_ramp_up": "ramp_up"}[kid]
                x = getattr(self.pilot.speed_plan, attr)
        elif kid in ("lap_sections", "race_laps"):
            setattr(self.laptimer, "sections_per_lap" if kid == "lap_sections" else "race_laps", int(x))
        elif kid in ("lap_debounce_s", "lap_min_s"):
            setattr(self.laptimer, "debounce_s" if kid == "lap_debounce_s" else "min_lap_s", x)
        elif kid in ("track_length_m", "track_curve_share"):
            setattr(self, kid, x)
        elif kid.startswith("tune_"):
            at = self.autotuner
            if at is None:   # saved rules load before the tuner exists: apply them when it attaches
                self._tune_pending = {**getattr(self, "_tune_pending", {}), kid: x}
                return x
            attr = {"tune_step": "step", "tune_curve_ratio": "curve_step_ratio", "tune_lost_grace_s": "lost_grace_s",
                    "tune_min_tracked": "min_tracked", "tune_curve_sat": "curve_sat", "tune_straight_sat": "straight_sat",
                    "tune_weave": "weave_fast", "tune_evidence": "evidence", "tune_max_retunes": "max_retunes",
                    "tune_confirm_laps": "confirm_laps", "tune_margin_steps": "margin_steps", "tune_kp_max": "kp_max",
                    "tune_race_first": "race_first"}[kid]
            setattr(at, attr, bool(x) if kid == "tune_race_first" else x)
            if kid in ("tune_step", "tune_curve_ratio") and at.state in ("running", "paused") and at.strategy == "sections":
                at._set_steps()
        elif kid == "top_throttle":
            if self.cap is not None:
                self.cap.top = x
        elif kid in ("max_user", "max_pilot_steers"):
            if self.cap is not None:
                self.cap.mode_caps["user" if kid == "max_user" else "local_angle"] = x
        elif kid == "max_auto":
            self._set_auto_max(x)
        elif kid == "auto_crawl":
            if self.pilot is not None:
                self.pilot.crawl = self.crawl if (x and self.crawl is not None) else None
        elif self.crawl is not None:
            if kid == "crawl_start":
                self.crawl.start = x
            elif kid == "crawl_ramp":
                self.crawl.ramp_up = x
        return x

    def attach_autotuner(self, at):
        """Install the lap tuner, apply any saved tuner rules, and record its defaults."""
        self.autotuner = at
        at.lap_source = lambda: self.laptimer.status(self.clock())
        # defaults first (what "Reset to defaults" returns to), THEN the saved rules on top
        self.defaults.update({k: v for k, v in self.knob_values().items() if k.startswith("tune_")})
        for k, v in getattr(self, "_tune_pending", {}).items():
            self.set_knob(k, v)
        self._tune_pending = {}

    # ------------------------------------------------------------ presets + lap-time estimate
    def presets(self) -> list:
        """One-tap setups for the page, built from line_config.py (the defaults this program started
        with), so the page holds no tuning numbers of its own."""
        race = {k: self.defaults[k] for k in RACE_KEYS if k in self.defaults}
        race.update(self.race_speeds)
        # Previous Race / Safe / Slow / PD modes must not inherit the Racing Line default.
        race["racing_line"] = 0
        law = "pure pursuit" if race.get("pure_pursuit") else "PD steering"
        out = []
        for pid, label, change, text in (
                ("race", "Race", {}, f"The race values from line_config.py: straight {race.get('speed_straight', 0):.2f}, curve "
                 f"{race.get('speed_corner', 0):.2f} at motor PWM {race.get('forward_pwm', 407):.0f}, {law}, target {race.get('target_x_ratio', 0):.2f}. "
                 "Real car, Nov 10 night, fresh battery: 14 laps 3.52-3.84 s, no wide laps. If it runs wide, press Safe race; on a tired battery raise the curve throttle."),
                ("racing", "Racing Line", RACING_LINE_SETUP,
                 f"Default racing line: hugs the inside tape (magenta line on the camera). Curves {RACING_LINE_SETUP['racing_apex_m'] * 100:.0f} cm from the "
                 f"inside tape's edge (negative = over it), straights {RACING_LINE_SETUP['racing_straight_m'] * 100:.0f} cm, motor PWM "
                 f"{RACING_LINE_SETUP['forward_pwm']}, straight {RACING_LINE_SETUP['speed_straight']:.2f} / curve {RACING_LINE_SETUP['speed_corner']:.2f}. "
                 "Sets everything itself: no need to press Race first. Uses the 13.4 deg middle-profile steering tune ONLY in Racing Line. "
                 "If it loses the inside line, raise Curve distance from inside tape; Race is the rollback."),
                ("safe", "Safe race", self.safe_speeds, f"Race steering at straight {self.safe_speeds.get('speed_straight', 0):.2f} / curve "
                 f"{self.safe_speeds.get('speed_corner', 0):.2f} at motor PWM {self.safe_speeds.get('forward_pwm', 407):.0f}: one step slower in the curves "
                 "than Race: real car, Nov 10 night, 7 laps 3.82-4.06 s. Use it if Race runs wide (fresh battery, different floor)."),
                ("slow", "First slow lap", SLOW_LAP, "The race steering at 0.75 / 0.70 throttle (the speed the car has already "
                 "driven, and what the lap tuner climbs from). Use it for the first lap of the day, then press Race."),
                ("pd", "PD fallback", PD_FALLBACK, "Switches pure pursuit off: PD steering as driven on the real car on Nov 9, "
                 "target 0.40, straight 0.75 / curve 0.70 (the fastest it was clean in the simulator). Use it if pure pursuit misbehaves.")):
            if pid == "safe" and not change:
                continue                                  # no SAFE_SPEED_* in line_config: no Safe race preset
            vals = {**race, **change}
            est = self.lap_estimate(vals)
            out.append({"id": pid, "label": label, "help": text, "values": vals, "estimate": est})
        return out

    def apply_preset(self, pid):
        pr = next((p for p in self.presets() if p["id"] == pid), None)
        if pr is None:
            raise ValueError(f"no preset '{pid}'")
        out = self.apply(pr["values"], quiet=True)
        if pid == "racing":
            self.log("warn", f"Racing Line applied: curves {out.get('racing_apex_m', 0) * 100:.0f} cm / straights "
                             f"{out.get('racing_straight_m', 0) * 100:.0f} cm from the inside tape, PWM {out.get('forward_pwm', 0):.0f}, "
                             f"straight {out.get('speed_straight', 0):.2f}, curve {out.get('speed_corner', 0):.2f}", "tune")
            return out
        self.log("info", f"Preset {pr['label']} applied: straight {out.get('speed_straight', 0):.2f}, curve "
                         f"{out.get('speed_corner', 0):.2f} (Save on car to keep it)", "tune")
        return out

    def lap_estimate(self, values=None) -> dict | None:
        """ESTIMATED lap and race time for these settings (default: what is on the car now), from the
        traced track length and curve share and the assumed throttle-to-speed table."""
        import lap_tuning
        v = values or self.knob_values()
        model = getattr(self.pilot, "model", None)
        if model is None or not v.get("speed_straight"):
            return None
        try:
            if self._ground is None:
                from linetrack.geometry import GroundModel
                self._ground = GroundModel(model)
            side = self._ground.lateral_at(v.get("lookahead_ratio", self.cfg.lookahead_ratio),
                                           v.get("target_x_ratio", self.cfg.target_x_ratio)) or 0.0
        except Exception:
            side = 0.0
        # the traced line is the tape's centre; the car rides half a tape (2.5 cm) + its target offset outside it
        lap = lap_tuning.estimate_lap(v["speed_straight"], v.get("speed_corner") or v["speed_straight"], model.speed_table,
                                      self.track_length_m, self.track_curve_share, ride_offset_m=max(0.0, side) + 0.025,
                                      pwm_stop=model.pwm_stop, pwm_full=float(v.get("forward_pwm") or model.pwm_full))
        if lap is None:
            return None
        n = self.laptimer.race_laps
        return {"lap_s": lap, "laps": n, "total_s": round(n * lap, 1), "straight": v["speed_straight"],
                "corner": v.get("speed_corner") or v["speed_straight"], "estimate": True,
                "basis": f"{self.track_length_m:.2f} m track, {100 * self.track_curve_share:.0f}% curve, assumed speed table; no penalties"}

    def laps_status(self) -> dict:
        """Lap timer + the estimate for the current settings (the page shows the estimate until real laps exist)."""
        st = self.laptimer.status(self.clock())
        with self.lock:
            st["estimate"] = self.lap_estimate()
        return st

    def _set_auto_max(self, x: float):
        """Full-auto max: caps the motor in local mode, the pilot's own limit and auto-crawl."""
        c = self.cfg
        if self.cap is not None:
            self.cap.mode_caps["local"] = x
        c.max_throttle = x
        c.throttle = min(c.throttle, x)
        if self.crawl is not None:
            self.crawl.hi = x
            self.crawl.thr = min(self.crawl.thr, x)
            self.crawl.start = min(self.crawl.start, x)

    def apply(self, values: dict, quiet: bool = False) -> dict:
        out, changes = {}, []
        with self.lock:
            before = self.knob_values()
            if any(k.startswith("speed_") for k in values) and self.autotuner is not None and self.autotuner.state in ("running", "paused"):
                self.autotuner.abort("speed settings changed; restart trial to verify the new speeds")
            for k, v in values.items():
                if k in KNOB_IDS:
                    out[k] = self.set_knob(k, v)
                    old = before.get(k)
                    if old is not None and abs(float(old) - float(out[k])) > 1e-9:
                        changes.append(f"{KNOB_LABELS[k]} {old:g} -> {out[k]:g}")
            if self.autotuner is not None and any(k in out for k in ("kp", "kd", "straight_kp", "straight_kd", "corner_kp", "corner_kd")):
                self.autotuner.adopt_user_gains()
            if any(k.startswith("speed_") for k in out):
                current = self.knob_values()
                out.update({k: current[k] for k in current if k.startswith("speed_")})
        if changes and not quiet:
            self.log("info", "Changed " + ", ".join(changes), "tune")
        return out

    def save(self) -> str:
        if not self.tuning_path:
            raise RuntimeError("no tuning file configured")
        with self.lock:
            vals = self.knob_values()
        vals["_defaults_id"] = self.defaults_id
        tmp = self.tuning_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(vals, f, indent=2)
        os.replace(tmp, self.tuning_path)
        self.log("info", f"Saved tuning to {self.tuning_path}", "tune")
        return self.tuning_path

    def load_saved(self) -> dict:
        """Apply tuning.json on top of line_config.py, but only if it was saved under the SAME defaults.
        A file from older defaults (or unreadable) is renamed aside and the new defaults stay in force."""
        if not (self.tuning_path and os.path.exists(self.tuning_path)):
            return {}
        try:
            with open(self.tuning_path) as f:
                saved = json.load(f)
        except (OSError, ValueError):
            saved = None
        if not isinstance(saved, dict) or saved.pop("_defaults_id", None) != self.defaults_id:
            aside = f"{self.tuning_path}.stale-{time.strftime('%Y%m%d-%H%M%S')}"
            os.replace(self.tuning_path, aside)
            self.log("warn", f"Saved tuning was made with older line_config.py defaults: moved aside to "
                             f"{os.path.basename(aside)}. Using the new defaults.", "tune")
            return {}
        out = self.apply(saved, quiet=True)
        self.log("info", f"Loaded saved tuning ({os.path.basename(self.tuning_path)})", "tune")
        return out

    def revert(self) -> dict:
        out = self.apply(self.defaults)
        self.log("info", "Reverted all knobs to line_config.py", "tune")
        if self.tuning_path and os.path.exists(self.tuning_path):
            os.remove(self.tuning_path)
        return out

    # ------------------------------------------------------------ speed profiles (simulator gain schedule)
    HERE_DIR = os.path.dirname(os.path.abspath(__file__))
    LIVE_NAME = "gain_schedule_live.json"   # the Mac simulator mode uses its own file, never the car's

    @classmethod
    def _load_profiles(cls):
        """Simulator gain schedule (gain_schedule.json), with car-verified gains merged in
        from gain_schedule_live.json (written by the schedule-guided auto-tune)."""
        try:
            with open(os.path.join(cls.HERE_DIR, "gain_schedule.json")) as f:
                data = json.load(f)
        except (OSError, ValueError):
            return {"profiles": []}
        live = []
        try:
            with open(os.path.join(cls.HERE_DIR, cls.LIVE_NAME)) as f:
                live = json.load(f).get("verified", [])
        except (OSError, ValueError):
            pass
        for pr in data.get("profiles", []):
            rob = pr.get("robustness", {})
            pr["robust"] = (rob.get("unstable", 1) == 0 and rob.get("off_track_in_4_runs", 1) == 0
                            and rob.get("clearance_failures_in_4_runs", 1) == 0)
            pr["model_kp"], pr["model_kd"] = pr["kp"], pr["kd"]
            match = [v for v in live if abs(v["throttle"] - pr["straight_cmd"]) <= 0.02]
            if match:
                v = match[-1]
                pr["kp"], pr["kd"], pr["live_verified"] = v["kp"], v["kd"], v.get("when", True)
        data["live_count"] = len(live)
        return data

    def schedule_points(self):
        return [(p["straight_cmd"], p["kp"], p["kd"]) for p in self.profiles.get("profiles", [])]

    def apply_profile(self, name):
        pr = next((p for p in self.profiles.get("profiles", []) if p["profile"] == name), None)
        if pr is None:
            raise ValueError(f"no speed profile '{name}'")
        vals = {"max_auto": pr["straight_cmd"], "speed_straight": pr["straight_cmd"], "speed_corner": pr["corner_cmd"],
                "kp": pr["kp"], "kd": pr["kd"]}
        out = self.apply(vals, quiet=True)
        top = self.cap.top if self.cap is not None else 1.0
        note = f" (Overall top speed {top:.2f} caps it lower)" if top < pr["straight_cmd"] else ""
        self.log("warn" if not pr.get("robust") else "info",
                 f"Speed profile {name}: straight {pr['straight_cmd']:.2f}, corner {pr['corner_cmd']:.2f}, "
                 f"KP {pr['kp']}, KD {pr['kd']}{note}" + ("" if pr.get("robust") else
                 " - NOT robust in simulation until the car is measured; use with STOP ready"), "tune")
        return out

    # ------------------------------------------------------------ tuning health + auto-tune
    def update_health(self, now):
        """Judge the last 10 s of pilot driving (at most once a second)."""
        if now - self._health_t < 1.0:
            return
        self._health_t = now
        with self.lock:
            hist = [h for h in self.history if h["t"] >= now - 10 and h.get("mode") in ("local_angle", "local")]
        if len(hist) < 40:
            self.health = None
            return
        h = tuning.analyze([x["t"] for x in hist], [x["error"] if x["error"] is not None else float("nan") for x in hist],
                           [x["steering"] or 0.0 for x in hist], [x["p"] or 0.0 for x in hist], [x["d"] or 0.0 for x in hist],
                           [x["status"] == "TRACKING" for x in hist], max_steering=self.cfg.max_steering)
        self.health = {"summary": tuning.summary(h), "findings": [{"level": a, "code": b, "text": c} for a, b, c in h.findings],
                       "swing_rate": round(h.swing_rate, 2), "err_p90": round(h.err_p90, 3), "jitter": round(h.jitter, 3),
                       "saturated": round(h.saturated, 3), "tracked": round(h.tracked, 3), "seconds": round(h.seconds, 1),
                       "suggest": dict(h.suggest)}

    # ------------------------------------------------------------ data source, diagnosis, simulator
    def review_status(self):
        available = os.path.isfile(os.path.join(os.path.dirname(__file__), "recording_review.py")) and os.path.isdir(os.path.join(os.path.dirname(__file__), "..", "tools"))
        status = self.recording_review.status() if self.recording_review else {"state": "idle", "stage": "Ready", "progress": 0}
        return dict(status, available=available, default_host="")

    def review_action(self, body):
        if not self.review_status()["available"]:
            raise RuntimeError("Open the Mac driving console to download and analyze recordings; analysis stays off the Pi")
        if self.recording_review is None:
            from recording_review import RecordingReview
            self.recording_review = RecordingReview()
        if body.get("action") == "cancel":
            return self.recording_review.cancel()
        if body.get("action") != "start":
            raise ValueError("Unknown analysis action")
        with self.lock:
            knobs = self.knob_values()
        host = body.get("host")
        if not host:
            raise ValueError("Enter the Pi's current IP address first")
        return self.recording_review.start(host, knobs, deep=body.get("deep", True))

    # ------------------------------------------------------------ read-only hardware test
    def hardware_test_status(self) -> dict:
        with self.lock:
            return json.loads(json.dumps(self.hardware_test))

    def start_hardware_test(self) -> dict:
        """Start read-only Pi checks; never changes tuning or sends a motor command."""
        with self.lock:
            mode = self.telemetry.get("mode")
            web_throttle = float(self.drive.get("throttle", 0.0) or 0.0)
            if mode in ("local", "local_angle") or abs(web_throttle) > 1e-6:
                raise RuntimeError("Select Manual and release the web joystick before running the hardware test")
            if self.hardware_test.get("state") == "running":
                return dict(self.hardware_test)
            baseline = self.knob_values()
            self.hardware_test = {"state": "running", "started": time.time(), "result": None, "error": None}
        threading.Thread(target=self._run_hardware_test, args=(baseline,), name="hardware-test", daemon=True).start()
        return self.hardware_test_status()

    @staticmethod
    def _camera_modes() -> dict:
        """List camera modes without starting a preview or changing camera settings."""
        attempts = []
        for exe in ("rpicam-hello", "libcamera-hello"):
            try:
                p = subprocess.run([exe, "--list-cameras"], capture_output=True, text=True, timeout=8)
                out = (p.stdout + p.stderr).strip()
                attempts.append({"command": exe, "returncode": p.returncode, "output": out[-6000:]})
                if p.returncode == 0 and out:
                    return {"available": True, "command": exe, "output": out[-6000:]}
            except FileNotFoundError:
                attempts.append({"command": exe, "available": False, "output": "not installed"})
            except Exception as e:
                attempts.append({"command": exe, "available": False, "output": f"{type(e).__name__}: {e}"})
        return {"available": False, "attempts": attempts, "output": "Camera mode listing unavailable"}

    def _run_hardware_test(self, baseline: dict):
        started = time.monotonic()
        samples = []
        try:
            while time.monotonic() - started < 5.0:
                with self.lock:
                    samples.append(dict(self.telemetry))
                time.sleep(0.10)
            with self.lock:
                after = self.knob_values()
            keys = sorted(set(baseline) | set(after))
            unchanged = all(baseline.get(k) == after.get(k) for k in keys)

            def nums(key):
                return [float(t[key]) for t in samples if isinstance(t.get(key), (int, float))]

            hz, proc, overlay, jpeg, battery = (nums(k) for k in ("hz", "proc_ms", "overlay_ms", "jpeg_ms", "battery_v"))
            found = [t for t in samples if "found" in t]
            result = {
                "safe": unchanged,
                "settings_unchanged": unchanged,
                "samples": len(samples),
                "loop_hz_median": round(float(np.median(hz)), 2) if hz else None,
                "detector_ms_median": round(float(np.median(proc)), 2) if proc else None,
                "overlay_ms_median": round(float(np.median(overlay)), 2) if overlay else None,
                "jpeg_ms_median": round(float(np.median(jpeg)), 2) if jpeg else None,
                "tracking_pct": round(100 * sum(bool(t.get("found")) for t in found) / len(found), 1) if found else None,
                "fresh_frames": sum(t.get("status") != "STALE" for t in samples) if samples else 0,
                "battery_available": any(t.get("battery_available") for t in samples),
                "battery_v_min": round(min(battery), 3) if battery else None,
                "battery_v_max": round(max(battery), 3) if battery else None,
                "camera_modes": self._camera_modes(),
            }
            if not samples:
                result["safe"] = False
                result["warning"] = "No live telemetry arrived; the Pi or drive program may be offline"
            elif not unchanged:
                result["safe"] = False
                result["warning"] = "Settings changed during the test; do not use this result"
            with self.lock:
                self.hardware_test = {"state": "done", "finished": time.time(), "result": result, "error": None}
        except Exception as e:
            with self.lock:
                self.hardware_test = {"state": "failed", "finished": time.time(), "result": None,
                                      "error": f"{type(e).__name__}: {e}"}

    def status(self) -> dict:
        """Where the data comes from and whether it's fresh. 'live' only for the real car."""
        with self.lock:
            t = self.telemetry.get("t")
        age = None if t is None else max(0.0, self.clock() - t)
        fresh = age is not None and age < 1.5
        ok, why = self.sim_available()
        return {"source": self.source, "detail": self.source_detail, "fresh": fresh,
                "age_s": None if age is None else round(age, 2), "live": self.source == "car" and fresh,
                "mode": self.telemetry.get("mode"), "sim_available": ok, "sim_reason": why}

    def diagnose(self) -> dict:
        import diagnose as dg
        st = self.status()
        now = self.clock()
        with self.lock:
            tele = dict(self.telemetry) if self.telemetry else None
            hist = [h for h in self.history if h["t"] >= now - 10]
        issues = dg.diagnose(tele if st["fresh"] else None, hist, self.health, self.knob_values(), self.profiles,
                             video_ok=st["fresh"])
        return {"status": st, "issues": issues}

    def sim_available(self):
        if getattr(self, "_sim_ok", None) is None:
            try:
                import sim.advisor  # noqa: F401
                self._sim_ok, self._sim_why = True, ""
            except Exception as e:
                self._sim_ok, self._sim_why = False, f"Simulator not installed here ({type(e).__name__}). It runs on the Mac: python3 pi/sim_dashboard.py --sim 8891"
        return self._sim_ok, self._sim_why

    def sim(self, body: dict) -> dict:
        """Simulator answers for the dashboard. On the real car they're refused while the pilot
        drives: a simulation would take CPU from the control loop."""
        ok, why = self.sim_available()
        if not ok:
            raise RuntimeError(why)
        if self.source == "car" and self.telemetry.get("mode") in ("local_angle", "local"):
            raise RuntimeError("Switch to Manual first: simulations would slow the car's control loop")
        from sim import advisor
        action = body.get("action")
        cmd = float(body.get("cmd") or 0.6)
        over = {k: getattr(self.cfg, k) for k in advisor.TUNABLE if hasattr(self.cfg, k)}
        over.update({k: body[k] for k in advisor.TUNABLE if body.get(k) is not None})
        if action == "speed":
            return advisor.speed_info(cmd, over)
        if action == "evaluate":
            kp = float(body["kp"]) if body.get("kp") is not None else self.cfg.kp
            kd = float(body["kd"]) if body.get("kd") is not None else self.cfg.kd
            cc = body.get("corner_cmd")
            return advisor.evaluate(cmd, kp, kd, over, corner_cmd=float(cc) if cc else None)
        if action == "map":
            return advisor.tuning_map(cmd, over)
        if action == "recommend":
            return advisor.recommend(cmd, over)
        raise ValueError(f"unknown simulator action {action!r}")

    def autotune(self, action, ceiling=None, strategy=None, speed=None, step=None):
        at = self.autotuner
        if action == "recompute":
            return self.recompute_schedule()
        if at is None:
            raise RuntimeError("Auto-tune isn't available in this program")
        if action == "start":
            if self.telemetry.get("mode") != "local":
                raise RuntimeError("Switch to Full auto first (the tuner drives in Full auto)")
            lim = self.cap.limit("local") if self.cap is not None else 1.0
            at.ceiling = min(float(ceiling) if ceiling else lim, lim)
            at.schedule = self.schedule_points()
            at.begin(strategy or "ladder", speed=None if speed in (None, "") else min(float(speed), at.ceiling))
        elif action == "section_start":
            if self.telemetry.get("mode") != "local":
                raise RuntimeError("Switch to Full Auto first (the tuner drives in Full auto)")
            lim = self.cap.limit("local") if self.cap is not None else 1.0
            at.ceiling = min(float(ceiling) if ceiling else lim, lim)
            at.begin_section(strategy, speed=None if speed in (None, "") else min(float(speed), at.ceiling))
        elif action == "sections_start":
            if self.telemetry.get("mode") != "local":
                raise RuntimeError("Switch to Full Auto first (the tuner drives in Full Auto)")
            lim = self.cap.limit("local") if self.cap is not None else 1.0
            at.ceiling = min(float(ceiling) if ceiling else lim, lim)
            at.schedule = self.schedule_points()
            race = None
            if at.race_first:
                # Race values first: put them on the car, then let the tuner check them before climbing.
                pr = next(p for p in self.presets() if p["id"] == "race")
                self.apply(pr["values"], quiet=True)
                lim = self.cap.limit("local") if self.cap is not None else 1.0
                at.ceiling = lim if not ceiling else min(float(ceiling), lim)
                est = pr["estimate"] or {}
                plan = self.pilot.speed_plan
                race = {"straight": plan.straight, "corner": plan.corner, "lap_s": est.get("lap_s"),
                        "total_s": est.get("total_s"), "laps": self.laptimer.race_laps}
                self.laptimer.reset()
            at.begin_sections(speed=None if speed in (None, "") else min(float(speed), at.ceiling), step=step, race=race)
        elif action == "sections_fail":
            at.fail_sections(strategy if strategy in ("straight", "corner") else None)
        elif action == "section_reset":
            at.reset_section(strategy)
        elif action == "section_finish":
            at.finish_section()
        elif action == "stop":
            at.abort("stopped from the dashboard")
            self.estop()
        elif action == "keep":
            b = at.apply_best()
            if b is None:
                raise RuntimeError("Nothing to keep yet")
            self.log("info", f"Kept auto-tune result: straight {b.straight:.2f}, corner {b.corner:.2f}, KP {b.kp:.2f}, KD {b.kd:.2f} (Save to make it permanent)", "tune")
        elif action == "save_schedule":
            if not at.verified:
                raise RuntimeError("No car-verified speeds yet: run the schedule-guided strategy first")
            path = os.path.join(self.HERE_DIR, self.LIVE_NAME)
            old = []
            try:
                with open(path) as f:
                    old = json.load(f).get("verified", [])
            except (OSError, ValueError):
                pass
            when = time.strftime("%Y-%m-%d %H:%M")
            new = [dict(v, when=when) for v in at.verified]
            keep = [o for o in old if all(abs(o["throttle"] - n["throttle"]) > 0.02 for n in new)]
            with open(path, "w") as f:
                json.dump({"verified": keep + new}, f, indent=2)
            self.profiles = self._load_profiles()
            self.log("info", f"Saved {len(new)} car-verified speed(s) to the speed profiles ({os.path.basename(path)})", "tune")
        else:
            raise ValueError(action)
        return at.status()

    def recompute_schedule(self):
        """Re-run the simulator gain schedule (only where the simulator is installed: the Mac)."""
        try:
            import sim.tune  # noqa: F401
        except Exception:
            raise RuntimeError("The simulator isn't installed here: run `python3 -m sim.tune` on the Mac, then ./deploy.sh")
        if getattr(self, "_recomputing", False):
            raise RuntimeError("Already recomputing")
        self._recomputing = True

        def work():
            try:
                import shutil

                from sim.tune import schedule
                self.log("info", "Recomputing the simulator gain schedule (about a minute)", "tune")
                schedule(verbose=False)
                root = os.path.dirname(self.HERE_DIR)
                shutil.copy(os.path.join(root, "sim", "gain_schedule.json"), os.path.join(self.HERE_DIR, "gain_schedule.json"))
                self.profiles = self._load_profiles()
                self.log("info", "Simulator gain schedule recomputed", "tune")
            except Exception as e:
                self.log("bad", f"Recompute failed: {e}", "tune")
            finally:
                self._recomputing = False
        threading.Thread(target=work, daemon=True).start()
        return {"state": "recomputing"}

    # ------------------------------------------------------------ event log
    def log(self, level: str, text: str, kind: str = "info"):
        """level: info | warn | bad. Kept in memory for the page and appended to logs/events-<date>.log."""
        now = time.time()
        with self.log_lock:
            self.event_id += 1
            ev = {"id": self.event_id, "ts": now, "time": time.strftime("%H:%M:%S", time.localtime(now)),
                  "level": level, "kind": kind, "text": text}
            self.events.append(ev)
            if len(self.events) > 500:
                del self.events[:100]
        if self.log_dir:
            try:
                os.makedirs(self.log_dir, exist_ok=True)
                path = os.path.join(self.log_dir, time.strftime("events-%Y%m%d.log", time.localtime(now)))
                with open(path, "a") as f:
                    f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now))} {level.upper():4} {kind:10} {text}\n")
            except OSError:
                pass
        return ev

    def events_since(self, since: int):
        with self.log_lock:
            return [e for e in self.events if e["id"] > since]

    def log_file_text(self) -> str:
        if not self.log_dir or not os.path.isdir(self.log_dir):
            return ""
        files = sorted(f for f in os.listdir(self.log_dir) if f.startswith("events-"))
        out = []
        for f in files[-3:]:  # last three days
            with open(os.path.join(self.log_dir, f)) as fh:
                out.append(f"# {f}\n" + fh.read())
        return "\n".join(out)

    # ------------------------------------------------------------ commands
    def request_mode(self, mode: str):
        if mode not in MODES:
            raise ValueError(mode)
        if mode in ("local", "local_angle"):
            self.pilot.resume_motion()
        with self.lock:
            self.pending_mode = mode
        self.log("info", f"Dashboard: switch to {MODE_NAMES[mode]}", "action")

    def take_mode(self):
        with self.lock:
            m, self.pending_mode = self.pending_mode, None
        return m

    def set_drive(self, steering, throttle):
        s = min(max(float(steering), -1.0), 1.0)
        t = min(max(float(throttle), -1.0), 1.0)
        with self.lock:
            self.drive = {"steering": s, "throttle": t, "t": self.clock()}
            if abs(s) > 0.08 or abs(t) > 0.08:
                self.pending_mode = "user"
        if abs(s) > 0.08 or abs(t) > 0.08:
            if self.autotuner is not None:
                self.autotuner.abort("manual touchpad override")
            self.pilot.stop_motion()

    def release_drive(self):
        with self.lock:
            self.drive = {"steering": 0.0, "throttle": 0.0, "t": -1e9}

    def web_drive(self):
        """(steering, throttle) if the web joystick is held right now, else None."""
        with self.lock:
            d = dict(self.drive)
        if self.clock() - d["t"] > self.drive_timeout_s:
            return None
        return d["steering"], d["throttle"]

    def estop(self):
        self.pilot.stop_motion()
        with self.lock:
            self.pending_mode = "user"
            self.drive = {"steering": 0.0, "throttle": 0.0, "t": -1e9}
            self.estop_until = self.clock() + 1.0
        js = self.joystick
        if js is not None and hasattr(js, "emergency_stop"):
            js.emergency_stop()
        self.log("bad", "STOP pressed on the dashboard (program keeps running)", "stop")
        if self.autotuner is not None:
            self.autotuner.abort("stopped (STOP pressed)")

    def off(self):
        """Quit program: stop the car, then shut the line follower down cleanly."""
        self.estop()
        self.log("bad", "Quit program pressed on the dashboard", "stop")
        if self.kill is not None:
            self.kill.trigger()

    def poweroff(self):
        """Shut down the Pi: stop, schedule a clean OS shutdown, quit the program.
        The shutdown runs in its own session so it survives this program exiting.
        Needs passwordless sudo for the pi user (Raspberry Pi OS default)."""
        self.estop()  # Even a refused/failed shutdown request must stop driving.
        if not self.allow_poweroff:
            raise RuntimeError("Shutting down is only available on the car")
        ok = subprocess.run(["sudo", "-n", "true"], capture_output=True, timeout=5).returncode == 0
        if not ok:
            raise RuntimeError("This Pi needs a password for sudo, so it can't be shut down from here. "
                               "Use Quit program, then run: sudo shutdown -h now")
        self.estop()
        self.log("bad", "Shut down Pi pressed on the dashboard", "stop")
        subprocess.Popen(["sh", "-c", "sleep 3; sudo -n /sbin/shutdown -h now"],
                         start_new_session=True, stdin=subprocess.DEVNULL,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if self.kill is not None:
            self.kill.trigger()

    # ------------------------------------------------------------ live data
    def publish(self, tele: dict, jpeg: bytes | None):
        with self.cond:
            self.seq += 1
            tele["seq"] = self.seq
            self.telemetry = tele
            self.history.append(tele)
            if len(self.history) > 400:
                del self.history[:100]
            if jpeg is not None:
                self.jpeg = jpeg
                self.jpeg_seq += 1
            self.cond.notify_all()

    def wait(self, last_seq: int, timeout=1.0):
        with self.cond:
            if self.seq == last_seq:
                self.cond.wait(timeout)
            return self.seq, [h for h in self.history if h["seq"] > last_seq][-40:]

    def wait_jpeg(self, last: int, timeout=1.0):
        with self.cond:
            if self.jpeg_seq == last:
                self.cond.wait(timeout)
            return self.jpeg_seq, self.jpeg

    # ------------------------------------------------------------ past runs
    def list_runs(self):
        out = []
        if not os.path.isdir(self.data_dir):
            return out
        for name in sorted(os.listdir(self.data_dir), reverse=True):
            if RUN_NAME.match(name) and os.path.exists(os.path.join(self.data_dir, name, "log.csv")):
                try:
                    out.append(summarize_run(self.load_run(name), name))
                except Exception as e:  # a half-written run must not break the list
                    out.append({"name": name, "error": str(e)})
        return out

    def load_run(self, name: str):
        if not RUN_NAME.match(name):
            raise ValueError("bad run name")
        with open(os.path.join(self.data_dir, name, "log.csv"), newline="") as f:
            return list(csv.DictReader(f))

    def frame_path(self, name: str, n: int) -> str:
        if not RUN_NAME.match(name) or n < 0:
            raise ValueError("bad frame")
        return os.path.join(self.data_dir, name, f"{n:06d}.jpg")


def _f(r, k, default=None):
    v = r.get(k)
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def summarize_run(rows, name):
    """Run summary + the moments worth looking at ("events")."""
    n = len(rows)
    if not n:
        return {"name": name, "frames": 0, "events": []}
    t0, t1 = _f(rows[0], "t", 0.0), _f(rows[-1], "t", 0.0)
    found = sum(1 for r in rows if r.get("found") == "1")
    modes = {}
    for r in rows:
        modes[r.get("mode", "?")] = modes.get(r.get("mode", "?"), 0) + 1
    events = []

    def streaks(pred, min_len):
        start = None
        for i, r in enumerate(rows + [None]):
            ok = r is not None and pred(r)
            if ok and start is None:
                start = i
            elif not ok and start is not None:
                if i - start >= min_len:
                    yield start, i - 1
                start = None

    for a, b in streaks(lambda r: r.get("found") == "0", 10):
        events.append({"kind": "lost", "from": a, "to": b, "text": f"Line lost for {b - a + 1} frames"})
    for a, b in streaks(lambda r: r.get("source") == "right", 10):
        events.append({"kind": "fallback", "from": a, "to": b, "text": f"Following the right line (backup) for {b - a + 1} frames"})
    for a, b in streaks(lambda r: abs(_f(r, "angle", 0.0)) >= 0.75, 6):
        events.append({"kind": "saturated", "from": a, "to": b, "text": "Steering near its limit"})
    for a, b in streaks(lambda r: r.get("single_stop") == "1", 1):
        events.append({"kind": "single", "from": a, "to": b, "text": "Stopped: only one line visible too long"})
    for a, b in streaks(lambda r: r.get("mode") == "local" and _f(r, "throttle", 0) >= 0.35
                        and (_f(r, "motion", 9.0) or 9.0) < 1.5, 40):
        events.append({"kind": "stalled", "from": a, "to": b, "text": "Throttle on but not moving (stalled)"})
    for a, b in streaks(lambda r: _f(r, "throttle", 0) < -0.05, 1):
        events.append({"kind": "brake", "from": a, "to": b, "text": "Reverse / E-stop braking"})
    events.sort(key=lambda e: e["from"])
    # merge repeats of the same kind that are close together (e.g. a burst of reverse taps)
    merged = []
    for e in events:
        prev = next((m for m in reversed(merged) if m["kind"] == e["kind"]), None)
        if prev is not None and e["from"] - prev["to"] <= 40:
            prev["to"] = max(prev["to"], e["to"])
            prev["count"] = prev.get("count", 1) + 1
        else:
            merged.append(dict(e))
    for m in merged:
        if m.get("count", 1) > 1:
            m["text"] += f" (x{m['count']})"
    events = sorted(merged, key=lambda e: (e["kind"] == "brake", e["from"]))  # problems first
    errs = []
    for r in rows:
        if r.get("found") != "1" or r.get("mode") not in ("local_angle", "local"):
            continue
        e = _f(r, "error")
        if e is None and _f(r, "x_ratio") is not None:
            e = _f(r, "x_ratio") - 0.20  # older runs: no error column; target was 0.20
        if e is not None:
            errs.append(e)
    err_stats = None
    if errs:
        a = np.abs(np.array(errs))
        err_stats = {"mean_abs": round(float(a.mean()), 4), "rms": round(float(np.sqrt((a ** 2).mean())), 4),
                     "max_abs": round(float(a.max()), 4), "p95_abs": round(float(np.percentile(a, 95)), 4)}
    return {
        "error": err_stats,
        "name": name, "frames": n, "duration_s": round(max(0.0, t1 - t0), 1),
        "tracked_pct": round(100.0 * found / n, 1), "modes": modes,
        "max_throttle": round(max((_f(r, "throttle", 0.0) for r in rows), default=0.0), 3),
        "events": events, "has_pid": "p" in rows[0],
    }


def jpeg_of(img_rgb, quality=70) -> bytes | None:
    if img_rgb is None:
        return None
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


# --------------------------------------------------------------------- parts
class WebModePart:
    """Add BEFORE the gamepad/web controllers: a mode chosen on the dashboard is
    written into user/mode for one loop; the controllers then adopt it, exactly
    as if A had been pressed."""

    def __init__(self, hub: Hub):
        self.hub = hub

    def run(self, mode):
        m = self.hub.take_mode()
        return m if m is not None else mode


class WebDrivePart:
    """Add after the controllers, before DriveMode. While the dashboard joystick
    is held (updates at least every 0.4 s) it replaces the gamepad's steering and
    throttle; let go, close the tab or lose Wi-Fi and the gamepad is back."""

    def __init__(self, hub: Hub):
        self.hub = hub

    def run(self, steering, throttle):
        if abs(steering or 0) > 0.08 or abs(throttle or 0) > 0.08:
            self.hub.release_drive()
            return steering, throttle
        d = self.hub.web_drive()
        if d is None:
            return steering, throttle
        return d


class ManualOverridePart:
    """Raw physical sticks take Auto back to Manual in the same vehicle loop."""
    def __init__(self, pilot, controller, hub=None):
        self.pilot, self.controller, self.hub = pilot, controller, hub
        self.previous_mode = "user"
        self.previous_stopped = False
        self.wait_neutral = False

    def run(self, mode, steering, throttle):
        s, t = float(steering or 0), float(throttle or 0)
        moved = abs(s) > 0.08 or abs(t) > 0.08
        if self.pilot.motion_stopped and not self.previous_stopped:
            self.wait_neutral = True
        takeover = (mode == "local" and moved) or (mode == "local_angle" and (abs(s) > 0.08 or t < -0.08))
        if takeover:
            if self.hub:
                self.hub.release_drive()
                if self.hub.autotuner:
                    self.hub.autotuner.abort("gamepad override")
            self.pilot.stop_motion()
            mode = "user"
            if hasattr(self.controller, "mode"):
                self.controller.mode = "user"
            self.wait_neutral = False
        elif mode != "user" and self.previous_mode == "user":
            self.pilot.resume_motion()
        if self.pilot.motion_stopped and mode != "user":
            if self.hub:
                self.hub.release_drive()
            mode = "user"
            if hasattr(self.controller, "mode"):
                self.controller.mode = "user"
            self.wait_neutral = True
        if self.pilot.motion_stopped and mode == "user" and not takeover:
            if self.wait_neutral and moved:
                s, t = 0.0, 0.0
            elif not moved:
                self.wait_neutral = False
        self.previous_mode = mode
        self.previous_stopped = self.pilot.motion_stopped
        return mode, s, t


class AutoMotionGuard:
    """Last motor gate: tuner completion takes effect in the same loop."""
    def __init__(self, pilot):
        self.pilot = pilot

    def run(self, mode, throttle):
        return 0.0 if mode == "local" and self.pilot.motion_stopped else throttle


class WebGuardPart:
    """For 1 s after dashboard STOP, force throttle to zero in both directions."""

    def __init__(self, hub: Hub):
        self.hub = hub

    def run(self, throttle):
        if self.hub.clock() < self.hub.estop_until:
            return 0.0
        return throttle


class TelemetryPart:
    """Add last (before the drivetrain): publishes what the car is doing this loop."""

    def __init__(self, hub: Hub, jpeg_every=1, clock=time.monotonic):
        self.hub = hub
        self.jpeg_every = jpeg_every
        self.clock = clock
        self.n = 0
        self.t_prev = None
        self.hz = 0.0
        self.state: dict = {}

    def _watch(self, tele, now):
        """Turn the live stream into log lines: only transitions, each with a minimum
        duration so one-frame flickers don't spam the log."""
        hub, st = self.hub, self.state
        m = tele["mode"]
        if m != st.get("mode"):
            if st.get("mode") is not None:
                hub.log("info", f"Mode: {MODE_NAMES.get(st.get('mode'), st.get('mode'))} -> {MODE_NAMES.get(m, m)}", "mode")
            st["mode"] = m
        pilot = m in ("local_angle", "local")

        def timed(key, cond, after_s, on_text, level, off_text=None, kind="info"):
            t0 = st.get(key)
            if cond:
                if t0 is None:
                    st[key] = now
                elif t0 is not False and now - t0 >= after_s:
                    hub.log(level, on_text(now - t0) if callable(on_text) else on_text, kind)
                    st[key] = False            # logged; wait for it to clear
                    st[key + "_since"] = t0
            elif t0 is not None:
                if t0 is False and off_text:
                    hub.log("info", off_text(now - st.get(key + "_since", now)), kind)
                st[key] = None

        timed("lost", pilot and tele["status"] == "LOST", 0.25, "Line lost", "warn",
              lambda d: f"Line found again after {d:.1f} s", "line")
        timed("stale", tele["status"] == "STALE", 0.3, "Camera stopped sending new frames (throttle forced to 0)", "bad",
              lambda d: f"Camera back after {d:.1f} s", "camera")
        timed("backup", pilot and tele["source"] == "right", 0.5,
              "Left line out of view: following the right line minus lane width (backup)", "warn",
              lambda d: f"Back on the left line after {d:.1f} s", "line")
        timed("racing_fallback", pilot and tele.get("racing_enabled") and not tele.get("racing_active"), 0.5,
              lambda d: f"Racing Line off, normal pursuit steering: {tele.get('racing_fallback') or 'no usable line'}", "warn",
              lambda d: f"Racing Line steering again after {d:.1f} s", "racing")
        timed("single", tele["single_stop"], 0.0,
              "Stopped: only one line visible for too long (could be the wrong line)", "bad",
              lambda d: "Both lines visible again: moving", "line")
        timed("stall", m == "local" and (tele["throttle"] or 0) >= 0.4 and (tele["motion"] or 9) < 1.5, 2.0,
              lambda d: f"Throttle {tele['throttle']:.2f} but the car isn't moving for {d:.0f} s: raise Max: Full auto, "
                        f"or check the motor battery/ESC switch", "warn", None, "speed")
        timed("sat", pilot and abs(tele["steering"] or 0) >= 0.95 * hub.cfg.max_steering, 1.0,
              "Steering at its limit for 1 s (turn too tight, or the car is far off the line)", "warn", None, "steer")
        timed("slow", self.hz and self.hz < 14, 2.0, lambda d: f"Loop slowed to {self.hz:.0f} Hz (target 20)", "warn",
              lambda d: "Loop speed back to normal", "cpu")
        timed("web", tele["web_driving"], 0.0, "Web joystick took control", "info",
              lambda d: f"Web joystick released after {d:.1f} s", "drive")

    def run(self, mode, user_throttle, steering, throttle, overlay, motion, both, x_ratio, found):
        now = self.clock()
        if self.t_prev is not None:
            dt = max(now - self.t_prev, 1e-3)
            self.hz = 0.9 * self.hz + 0.1 * (1.0 / dt) if self.hz else 1.0 / dt
        self.t_prev = now
        hub = self.hub
        pilot, det, cfg = hub.pilot, hub.det, hub.cfg
        cmd = pilot.active_cmd() if pilot is not None else None
        d = det._det if det is not None else None
        crawl = pilot.crawl if pilot is not None else None
        tele = {
            "t": round(now, 3), "mode": mode, "hz": round(self.hz, 1),
            "found": bool(found), "x": None if x_ratio is None else round(float(x_ratio), 4),
            "target": cfg.target_x_ratio,
            "source": getattr(d, "source", "none"), "conf": round(float(getattr(d, "confidence", 0.0)), 3),
            "racing_enabled": bool(getattr(cfg, "racing_line", False)),
            "racing_active": bool(getattr(pilot, "racing_active", False)) if cmd is not None else False,
            "racing_fallback": getattr(pilot, "racing_fallback_reason", None) if cmd is not None else None,
            "both": bool(both), "motion": None if motion is None else round(float(motion), 2),
            "proc_ms": round(getattr(det, "proc_ms", 0.0), 1),
            "status": cmd.status if cmd is not None else ("TRACKING" if found else "LOST"),
            "error": round(cmd.error, 4) if cmd is not None else None,
            "p": round(cmd.p, 4) if cmd is not None else None,
            "d": round(cmd.d, 4) if cmd is not None else None,
            "ff": round(cmd.ff, 4) if cmd is not None else None,
            "curv": None if getattr(pilot, "curvature", None) is None or cmd is None else round(pilot.curvature, 3),
            "corner_weight": round(getattr(cmd, "corner_weight", 0.0), 3) if cmd is not None else 0.0,
            "section": getattr(cmd, "section", "straight") if cmd is not None else "straight",
            "speed_target": round(getattr(pilot, "speed_target", 0.0), 3) if getattr(pilot, "speed_target", None) is not None else None,
            "speed_reason": getattr(pilot, "speed_reason", "Speed plan off") if cmd is not None else "Manual",
            "kp_used": round(getattr(cmd, "kp_used", 0.0), 3) if cmd is not None else None,
            "kd_used": round(getattr(cmd, "kd_used", 0.0), 3) if cmd is not None else None,
            "corner_cap": None if cmd is None or getattr(pilot, "corner_cap", None) is None else round(pilot.corner_cap, 3),
            "pilot_angle": round(cmd.angle, 4) if cmd is not None else None,
            "single_stop": bool(getattr(pilot, "single_line_stopped", False)) if cmd is not None else False,
            "crawl_thr": round(crawl.thr, 3) if (crawl is not None and cmd is not None) else None,
            "stick_throttle": None if user_throttle is None else round(float(user_throttle), 3),
            "steering": None if steering is None else round(float(steering), 4),
            "throttle": None if throttle is None else round(float(throttle), 4),
            "web_driving": hub.web_drive() is not None,
        }
        battery = hub.battery_monitor.snapshot() if hub.battery_monitor is not None else {}
        tele["battery_v"] = battery.get("voltage_v") if battery.get("available") else None
        tele["battery_age_s"] = battery.get("age_s")
        tele["battery_available"] = bool(battery.get("available"))
        thr = tele["throttle"] or 0.0
        tele["pwm"] = round(370 + 37 * max(thr, 0.0), 1)
        self.n += 1
        if hub.autotuner is not None and cmd is not None:
            try:
                hub.autotuner.update(mode, cmd, cmd.status == "TRACKING", tele.get("section"))
            except Exception as e:  # never let tuning stop the car loop
                hub.autotuner.abort(f"error: {e}")
        if hub.autotuner is not None:
            tele["autotune"] = hub.autotuner.state
        # Always told the mode: a lap must end when the pilot stops driving (Manual, STOP, tuner pause),
        # or its clock would keep running until Full Auto comes back.
        driving = cmd is not None and not pilot.motion_stopped
        hub.laptimer.update(now, mode if driving else "user", tele.get("section"), cmd is not None and cmd.status == "TRACKING")
        st = hub.laptimer.status(now)
        tele["lap_current"], tele["lap_last"], tele["lap_best"], tele["lap_count"] = st["current"], st["last"], st["best"], st["count"]
        if mode == "local" and pilot.motion_stopped:
            tele["throttle"], tele["pwm"] = 0.0, 370.0
        try:
            hub.update_health(now)
        except Exception as e:
            tele["health_error"] = str(e)
        try:
            self._watch(tele, now)
        except Exception as e:  # the log must never stop the car loop
            tele["watch_error"] = str(e)
        t_jpeg = time.perf_counter()
        jpeg = jpeg_of(overlay) if (self.n % self.jpeg_every == 0) else None
        tele["overlay_ms"] = round(float(getattr(det, "overlay_ms", 0.0)), 3)
        tele["jpeg_ms"] = round((time.perf_counter() - t_jpeg) * 1000, 3)
        hub.publish(tele, jpeg)


# --------------------------------------------------------------------- http
class Handler(BaseHTTPRequestHandler):
    hub: Hub = None
    page_path: str = None
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # keep run.log readable
        pass

    # ---- helpers
    def _send(self, code, body: bytes, ctype="application/json", extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj).encode())

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n > 100_000:
            raise ValueError("body too large")
        return json.loads(self.rfile.read(n) or b"{}")

    # ---- routes
    def do_GET(self):
        u = urlparse(self.path)
        p, q = u.path, parse_qs(u.query)
        hub = self.hub
        try:
            if p in ("/", "/index.html", "/tuner"):
                tuner = os.path.join(os.path.dirname(os.path.abspath(self.page_path)), "laptuner.html")
                with open(tuner if os.path.exists(tuner) else self.page_path, "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if p == "/classic":
                with open(self.page_path, "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if p == "/api/state":
                with hub.lock:
                    vals = hub.knob_values()
                knobs = [dict(id=k[0], group=k[1], label=k[2], min=k[3], max=k[4], step=k[5], help=k[6],
                              value=vals.get(k[0]), default=hub.defaults.get(k[0])) for k in KNOBS]
                with hub.lock:
                    presets = hub.presets()
                return self._json({"knobs": knobs, "telemetry": hub.telemetry, "profiles": hub.profiles, "presets": presets,
                                   "saved": bool(hub.tuning_path and os.path.exists(hub.tuning_path)),
                                   "crawl_available": hub.crawl is not None})
            if p == "/api/runs":
                return self._json(hub.list_runs())
            if p == "/api/status":
                return self._json(hub.status())
            if p == "/api/hardware-test":
                return self._json(hub.hardware_test_status())
            if p == "/api/review":
                return self._json(hub.review_status())
            if p == "/api/diagnose":
                return self._json(hub.diagnose())
            if p == "/api/tuning":
                return self._json({"health": hub.health, "profiles": hub.profiles,
                                   "recomputing": getattr(hub, "_recomputing", False),
                                   "autotune": hub.autotuner.status() if hub.autotuner is not None else None})
            if p == "/api/laps":
                return self._json(hub.laps_status())
            if p == "/api/log":
                return self._json(hub.events_since(int(q.get("since", ["0"])[0] or 0)))
            if p == "/api/log.txt":
                return self._send(200, hub.log_file_text().encode(), "text/plain; charset=utf-8",
                                  {"Content-Disposition": 'attachment; filename="linefollow-events.log"'})
            m = re.match(r"^/runs/([^/]+)/log\.csv$", p)
            if m:
                if not RUN_NAME.match(m.group(1)):
                    return self._json({"error": "bad run name"}, 400)
                with open(os.path.join(hub.data_dir, m.group(1), "log.csv"), "rb") as f:
                    return self._send(200, f.read(), "text/csv",
                                      {"Content-Disposition": f'attachment; filename="run-{m.group(1)}.csv"'})
            m = re.match(r"^/api/runs/([^/]+)$", p)
            if m:
                rows = hub.load_run(m.group(1))
                return self._json({"summary": summarize_run(rows, m.group(1)), "rows": rows})
            m = re.match(r"^/runs/([^/]+)/(\d+)\.jpg$", p)
            if m:
                path = hub.frame_path(m.group(1), int(m.group(2)))
                if not os.path.exists(path):
                    return self._json({"error": "no such frame"}, 404)
                with open(path, "rb") as f:
                    return self._send(200, f.read(), "image/jpeg", {"Cache-Control": "max-age=3600"})
            if p == "/video":
                return self._mjpeg()
            if p == "/events":
                return self._events(int(q.get("since", ["0"])[0] or 0))
            return self._json({"error": "not found"}, 404)
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as e:
            try:
                self._json({"error": str(e)}, 400)
            except Exception:
                pass

    def do_POST(self):
        p = urlparse(self.path).path
        hub = self.hub
        try:
            body = self._body()
            if p == "/api/review":
                return self._json(hub.review_action(body))
            if p == "/api/lap-review":
                import lap_tuning
                return self._json(lap_tuning.assess_lap(body.get("samples"), body.get("speed"),
                                                       step=body.get("step", 0.05)))
            if p == "/api/knobs":
                return self._json({"applied": hub.apply(body)})
            if p == "/api/save":
                return self._json({"saved": hub.save()})
            if p == "/api/revert":
                return self._json({"applied": hub.revert()})
            if p == "/api/mode":
                hub.request_mode(body.get("mode"))
                return self._json({"ok": True})
            if p == "/api/drive":
                if body.get("release"):
                    hub.release_drive()
                else:
                    hub.set_drive(body.get("steering", 0.0), body.get("throttle", 0.0))
                return self._json({"ok": True})
            if p == "/api/estop":
                hub.estop()
                return self._json({"ok": True})
            if p == "/api/off":
                hub.off()
                return self._json({"ok": True})
            if p == "/api/profile":
                return self._json({"applied": hub.apply_profile(body.get("name"))})
            if p == "/api/laps":
                hub.laptimer.reset()
                hub.log("info", "Lap timer reset", "action")
                return self._json(hub.laps_status())
            if p == "/api/preset":
                return self._json({"applied": hub.apply_preset(body.get("name"))})
            if p == "/api/autotune":
                return self._json(hub.autotune(body.get("action"), body.get("ceiling"), body.get("strategy"),
                                               body.get("speed"), body.get("step")))
            if p == "/api/hardware-test":
                return self._json(hub.start_hardware_test())
            if p == "/api/poweroff":
                hub.poweroff()
                return self._json({"ok": True})
            if p == "/api/sim":
                return self._json(hub.sim(body))
            return self._json({"error": "not found"}, 404)
        except Exception as e:
            return self._json({"error": str(e)}, 400)

    def _mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        last = 0
        while True:
            last, jpg = self.hub.wait_jpeg(last)
            if jpg is None:
                continue
            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                             + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
            self.wfile.flush()

    def _events(self, since):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        last = since
        while True:
            seq, items = self.hub.wait(last)
            if seq == last:
                self.wfile.write(b": ping\n\n")
            else:
                last = seq
                self.wfile.write(b"data: " + json.dumps(items).encode() + b"\n\n")
            self.wfile.flush()
            time.sleep(0.05)  # at most ~20 messages/s per page, batched


def local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))  # no packet is sent; picks the Wi-Fi interface
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


def start_server(hub: Hub, port=8890, page_path=None):
    page_path = page_path or os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard.html")
    handler = type("LineFollowHandler", (Handler,), {"hub": hub, "page_path": page_path})
    srv = ThreadingHTTPServer(("0.0.0.0", port), handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, name="dashboard", daemon=True).start()
    return srv, f"http://{local_ip()}:{port}"
