"""Donkeycar parts for the orange left-line follower.

Two parts, both added by drive_line.py:

  LineDetectorPart   runs every loop (all modes). Input: cam/image_array (RGB).
                     Outputs: line/x_ratio, line/found, line/frame_age, line/overlay.
                     The overlay is what the web UI shows, so detection can be
                     checked in 'user' mode before handing over steering.

  LineFollowerPilot  runs only when run_pilot is True (mode != 'user').
                     Inputs: user/mode + the line/* values.
                     Outputs: pilot/angle, pilot/throttle.
                     Throttle is non-zero only in 'local' mode while TRACKING (or
                     briefly while coasting after the line is lost). Its source is
                     the speed plan if set, else auto-crawl if set, else the fixed
                     LINE_THROTTLE; it is 0 after SINGLE_LINE_STOP_S on one line.
                     DriveMode ignores pilot throttle in 'local_angle'.

Sign convention: angle -1 = left, +1 = right (matches the car's PWM calibration).
"""
from __future__ import annotations

import math
import time
import zlib

import cv2
import numpy as np

from linetrack.config import Config
from linetrack.controller import DRIVE, STEER_ONLY, Command, SteeringController
from linetrack.detector import LeftLineDetector
from linetrack.geometry import (Bend, CarModel, GroundModel, allowed_speed, cmd_for_speed, fit_bend, predict_goal, pursuit_goal, ride_line,
                                pursuit_steer, speed_of_cmd, steer_for_curvature)
from linetrack.racing_line import CURVE_FULL, CURVE_START, curve_start_distance, inner_curvature, ride_offset

# extra keys in line_config.py that are not Config fields
_EXTRAS = {"RESET_AFTER_GAP_S": 0.3, "SHOW_OVERLAY": True, "TOP_THROTTLE": 0.35, "RECORD": False,
           "TURN_SLOWDOWN": 0.0, "MIN_TURN_SCALE": 1.0,
           "AUTO_CRAWL": False, "CRAWL_START": 0.40, "CRAWL_MAX": 0.60, "CRAWL_RAMP_UP": 0.10,
           "CRAWL_MOVE_MOTION": 2.5, "CRAWL_FAST_MOTION": 6.0, "SINGLE_LINE_STOP_S": 0.0,
           "WEB_UI": True, "WEB_PORT": 8890,
           "MAX_USER": 1.0, "MAX_PILOT_STEERS": 1.0, "MAX_AUTO": 1.0,
           "LOST_COAST_S": 0.0, "LOST_SLOW_S": 0.0,
           "SPEED_STRAIGHT": 0.0, "SPEED_TRANSITION": 0.0, "SPEED_CORNER": 0.0,
           "SPEED_RAMP_DOWN": 1.0, "SPEED_RAMP_UP": 0.4,
           "AUTOTUNE_START": 0.55, "AUTOTUNE_STEP": 0.03, "AUTOTUNE_WINDOW_S": 10.0, "TUNE_RACE_FIRST": True,
           "RACE_SPEED_STRAIGHT": None, "RACE_SPEED_TRANSITION": None, "RACE_SPEED_CORNER": None,
           "THROTTLE_FORWARD_PWM": None, "RACE_FORWARD_PWM": None, "SAFE_FORWARD_PWM": None,
           "SAFE_SPEED_STRAIGHT": None, "SAFE_SPEED_TRANSITION": None, "SAFE_SPEED_CORNER": None,
           "TRACK_LENGTH_M": 4.98, "TRACK_CURVE_SHARE": 0.71,
           # car model for corner geometry (linetrack/geometry.py); None = the model defaults
           "WHEELBASE_M": None, "STEER_MAX_DEG": None, "CAMERA_FOV_DEG": None, "CAM_HEIGHT_M": None,
           "CAM_PITCH_DEG": None, "CAM_FWD_M": None, "GRIP_MU": None, "BRAKE_DECEL": None,
           "CONTROL_DELAY_S": None, "SPEED_TABLE": None,
           "RACING_LINE": False, "RACING_APEX_M": 0.03, "RACING_STRAIGHT_M": 0.12, "RACING_TURN_IN_M": 1.5,
           "BATTERY_I2C_BUS": 1, "BATTERY_I2C_ADDR": 0x42, "BATTERY_SAMPLE_S": 1.0}


def car_model(extras: dict | None = None) -> CarModel:
    """CarModel from line_config values (anything unset keeps the model default)."""
    m, e = CarModel(), extras or {}
    for key, attr in (("WHEELBASE_M", "wheelbase_m"), ("STEER_MAX_DEG", "steer_max_deg"),
                      ("CAMERA_FOV_DEG", "camera_fov_deg"), ("CAM_HEIGHT_M", "cam_height_m"),
                      ("CAM_PITCH_DEG", "cam_pitch_deg"), ("CAM_FWD_M", "cam_fwd_m"), ("GRIP_MU", "grip_mu"),
                      ("BRAKE_DECEL", "brake_decel"), ("CONTROL_DELAY_S", "delay_s")):
        if e.get(key) is not None:
            setattr(m, attr, float(e[key]))
    if e.get("THROTTLE_FORWARD_PWM") is not None:
        m.pwm_full = float(e["THROTTLE_FORWARD_PWM"])
    if e.get("SPEED_TABLE"):
        m.speed_table = tuple(tuple(r) for r in e["SPEED_TABLE"])
    return m


def load_line_config(module) -> tuple[Config, dict]:
    """Build a Config from a line_config module (UPPER_CASE names = Config fields)."""
    cfg = Config()
    data = {}
    for name in cfg.as_dict():
        if hasattr(module, name.upper()):
            data[name] = getattr(module, name.upper())
    if hasattr(module, "LINE_THROTTLE"):
        data["throttle"] = module.LINE_THROTTLE
    cfg.update(data)  # coerces types and caps throttle at max_throttle
    extras = {k: getattr(module, k, v) for k, v in _EXTRAS.items()}
    return cfg, extras


def make_throttle_cap(cfg: Config, extras: dict) -> "ThrottleCap":
    """ThrottleCap with the overall top speed and the per-mode maximums from line_config."""
    return ThrottleCap(extras["TOP_THROTTLE"], extras["TURN_SLOWDOWN"], extras["MIN_TURN_SCALE"],
                       cfg.max_steering, mode_caps={"user": extras["MAX_USER"],
                                                    "local_angle": extras["MAX_PILOT_STEERS"],
                                                    "local": extras["MAX_AUTO"]})


def _frame_key(img: np.ndarray) -> int:
    # cheap content fingerprint: a new camera frame always changes sensor noise
    return zlib.crc32(np.ascontiguousarray(img[::8, ::8]).tobytes())


class LineDetectorPart:
    def __init__(self, cfg: Config, show_overlay: bool = True, clock=time.monotonic, model: CarModel | None = None):
        self.cfg = cfg
        self.model = model or CarModel()
        self.ground = None           # GroundModel, built on first frame
        self.bend = None             # corner ahead (linetrack.geometry.Bend) from the tracked edge
        self.path = None             # (edge ground points near -> far, "left"/"right", lane width ratio) for pure pursuit
        self.show_overlay = show_overlay
        self.clock = clock
        self.detector = LeftLineDetector(cfg)
        self.pilot: LineFollowerPilot | None = None  # set by drive_line.py, for the HUD
        self._key = None
        self._t_new = None
        self._det = None
        self._overlay = None
        self._overlay_frame = None
        self._overlay_age = 0.0
        self.proc_ms = 0.0
        self.overlay_ms = 0.0
        self._small = None
        self.motion = None   # camera motion score: ~0.9 still, ~2.5+ rolling, ~17 normal driving

    def attach(self, pilot: "LineFollowerPilot"):
        """Connect a pilot: it reads this detector's corner geometry and edge path each frame."""
        self.pilot = pilot
        pilot.overlay_refresh = self.refresh_overlay
        pilot.bend_source = lambda: self.bend       # corner geometry for sections / feedforward / corner speed
        pilot.path_source = lambda: self.path       # edge ground points for pure pursuit
        pilot.conf_source = lambda: self._det.confidence if self._det is not None else None   # detection confidence

    def run(self, img_rgb):
        now = self.clock()
        if img_rgb is None:
            return None, False, float("inf"), None, None, False
        key = _frame_key(img_rgb)
        if key != self._key or self._det is None:
            self._key, self._t_new = key, now
            t0 = time.perf_counter()
            bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)  # camera gives RGB; detector wants BGR
            self._det = self.detector.detect(bgr)
            self.bend = self._bend(self._det, img_rgb.shape[1], img_rgb.shape[0])
            self.proc_ms = (time.perf_counter() - t0) * 1000
            small = cv2.resize(cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY), (60, 48),
                               interpolation=cv2.INTER_AREA).astype(np.int16)
            if self._small is not None:
                self.motion = float(np.mean(np.abs(small - self._small)))
            self._small = small
        det = self._det
        age = now - self._t_new
        self._overlay_frame, self._overlay_age = img_rgb, age
        t_overlay = time.perf_counter()
        overlay = self.draw(img_rgb, det, age) if self.show_overlay else img_rgb
        self._overlay = overlay
        self.overlay_ms = (time.perf_counter() - t_overlay) * 1000
        x = det.x_ratio if det.found else None
        return x, bool(det.found), age, overlay, self.motion, self.identity_sure()

    def refresh_overlay(self):
        """Redraw the current frame after the pilot computes its current command.

        The detector runs before the pilot in the vehicle loop, so drawing only inside
        ``run`` made the Racing Line overlay describe the previous pilot update. Keep
        the returned image object stable and update it in place so later dashboard parts
        see the current plan without changing the Donkeycar part wiring.
        """
        if not self.show_overlay or self._overlay is None or self._overlay_frame is None or self._det is None:
            return
        t_overlay = time.perf_counter()
        current = self.draw(self._overlay_frame, self._det, self._overlay_age)
        self._overlay[...] = current
        self.overlay_ms = (time.perf_counter() - t_overlay) * 1000

    def _bend(self, det, w, h):
        """Curvature of the corner ahead from the tracked inside edge (left line, or the right line
        while it's the backup). Never lets geometry trouble stop the car loop."""
        self.path = None
        try:
            if not det.found:
                return None
            if self.ground is None:
                self.ground = GroundModel(self.model)
            pts = det.edge_points if det.source == "left" else det.right_points
            if pts is not None and len(pts):
                pts = pts[np.argsort(-pts[:, 1])]           # bottom of the image (near the car) first
            ground = self.ground.project(pts, w, h)
            self.path = (ground, det.source, float(getattr(det, "lane_width_ratio", 0.0) or self.cfg.lane_width_ratio))
            return fit_bend(ground)
        except Exception:
            return None

    def both_lines(self) -> bool:
        """True when the left line AND the right line are both identified."""
        d = self._det
        return bool(d is not None and d.found and d.source == "left" and d.right_x_ratio is not None)

    def identity_sure(self) -> bool:
        """True when it is known WHICH line is being followed: both are in view, or the one in
        view has been tracked without a break since both were. This is what the pilot's
        one-line safety stop looks at, so a turn that hides the inside line doesn't trip it."""
        d = self._det
        return bool(d is not None and d.found and d.sure)

    def draw(self, img_rgb, det, age):
        """Annotated copy of the camera image (RGB, same size, for the web UI)."""
        c = self.cfg
        out = img_rgb.copy()
        H, W = out.shape[:2]
        ry = int(c.roi_top_ratio * H)
        out[:ry] = (out[:ry] * 0.5).astype(np.uint8)
        tx = int(c.target_x_ratio * W)
        cv2.line(out, (tx, ry), (tx, H), (0, 255, 255), 1)                    # target (cyan)
        if len(det.curve) > 1:
            cv2.polylines(out, [det.curve.astype(np.int32)], False, (0, 255, 0), 2)    # left edge (green)
        if len(det.right_curve) > 1:
            cv2.polylines(out, [det.right_curve.astype(np.int32)], False, (255, 170, 0), 1)  # right ref (amber)
        ride = getattr(self.pilot, "racing_ride", None) if self.cfg.racing_line else None
        if ride is not None and len(ride) > 1 and self.ground is not None:
            # the racing line the pilot steered along on the previous frame (magenta)
            px = self.ground.to_pixels(ride, W, H)
            px = px[np.isfinite(px).all(1) & (px[:, 1] > 0) & (px[:, 1] < H)]
            if len(px) > 1:
                cv2.polylines(out, [px.astype(np.int32)], False, (255, 0, 255), 2)
        if det.lookahead_x is not None:
            p = (int(det.lookahead_x), int(det.lookahead_y))
            cv2.circle(out, p, 4, (255, 0, 255), 1)
            cv2.line(out, (tx, p[1]), p, (255, 0, 255), 1)
        cmd = self.pilot.active_cmd() if self.pilot is not None else None
        status = cmd.status if cmd is not None else ("TRACKING" if det.found else "LOST")
        if age > c.max_frame_age_s:
            status = "STALE"
        col = (0, 220, 0) if status == "TRACKING" else (255, 60, 60)
        cv2.rectangle(out, (0, 0), (W, 14), (0, 0, 0), -1)
        src = {"left": "L", "right": "R"}.get(det.source, "-")
        mv = "" if self.motion is None else f" mv{self.motion:.1f}"
        mv += "" if self.both_lines() or not det.found else (" 1-LINE" if det.sure else " 1-LINE?")
        cv2.putText(out, f"{status} {src} c{det.confidence:.2f} {self.proc_ms:.0f}ms{mv}",
                    (2, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.32, col, 1, cv2.LINE_AA)
        if cmd is not None:  # steering bar along the bottom (only while pilot runs)
            if self.cfg.racing_line:
                label = "RACE-LINE" if getattr(self.pilot, "racing_active", False) else "RACE-LINE fallback"
                cv2.putText(out, label, (W - 104, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.30,
                            (255, 220, 0) if getattr(self.pilot, "racing_active", False) else (120, 120, 255), 1, cv2.LINE_AA)
            cx, by = W // 2, H - 5
            cv2.rectangle(out, (cx, by - 3), (cx + int(cx * 0.9 * cmd.angle), by + 3), (255, 200, 0), -1)
            cv2.line(out, (cx, by - 5), (cx, by + 5), (255, 255, 255), 1)
        return out


class AutoCrawl:
    """Finds and holds the slowest throttle that keeps the car rolling.

    Uses the camera motion score (no wheel encoder on this car):
      - not moving (median of last 3 scores < move_thresh): raise throttle by ramp_up/s
      - moving too fast (median > fast_thresh): lower it by ramp_down/s
      - otherwise hold.
    Always clamped to [lo, hi]; hi is the hard ceiling for the demo.
    """

    def __init__(self, start=0.40, lo=0.35, hi=0.60, ramp_up=0.10, ramp_down=0.05,
                 move_thresh=2.5, fast_thresh=6.0):
        self.start, self.lo, self.hi = start, lo, hi
        self.ramp_up, self.ramp_down = ramp_up, ramp_down
        self.move_thresh, self.fast_thresh = move_thresh, fast_thresh
        self.reset()

    def reset(self):
        self.thr = self.start
        self.hist = []

    def update(self, motion, dt):
        if motion is not None:
            self.hist = (self.hist + [motion])[-3:]
        if len(self.hist) == 3:
            m = sorted(self.hist)[1]
            if m < self.move_thresh:
                self.thr += self.ramp_up * dt
            elif m > self.fast_thresh:
                self.thr -= self.ramp_down * dt
        self.thr = min(max(self.thr, self.lo), self.hi)
        return self.thr

    @property
    def moving(self):
        return len(self.hist) == 3 and sorted(self.hist)[1] >= self.move_thresh


class SpeedPlan:
    """Section-aware speed plan with a safe, gradual handoff.

    Curvature is the primary signal: it sees a bend before steering has wound on.
    A steering-based fallback is kept for callers that do not have geometry yet.
    The optional transition speed prevents a straight -> corner speed cliff, while
    the slew limits make braking and acceleration visibly progressive.
    """

    def __init__(self, straight: float, corner: float, transition: float | None = None,
                 full_at: float = 0.6, ramp_down: float = 1.0, ramp_up: float = 0.4):
        self.straight = max(0.0, float(straight))
        self.corner = min(self.straight, max(0.0, float(corner)))
        self.transition = float(transition) if transition is not None and transition > 0 else (self.straight + self.corner) / 2
        self.transition = min(self.straight, max(self.corner, self.transition))
        self.full_at = full_at
        self.ramp_down, self.ramp_up = max(0.01, ramp_down), max(0.01, ramp_up)
        self._current = None

    def reset(self):
        self._current = None

    def target(self, steering: float, max_steering: float, corner_weight: float | None = None,
               recovery_error: float = 0.0) -> float:
        if corner_weight is None:
            frac = min(1.0, abs(steering) / max(1e-6, self.full_at * max_steering))
            return self.straight - (self.straight - self.corner) * frac
        w = min(1.0, max(0.0, float(corner_weight)))
        if w <= 0.5:
            planned = self.straight + (self.transition - self.straight) * (w * 2.0)
        else:
            planned = self.transition + (self.corner - self.transition) * ((w - 0.5) * 2.0)
        # A nominal straight can still need a large recovery. Small corrections
        # leave speed alone; strong steering/error independently request slower speed.
        demand = abs(steering) / max(max_steering, 1e-6)
        recovery = max((demand - 0.25) / 0.5, (abs(recovery_error) - 0.08) / 0.14)
        recovery = min(1.0, max(0.0, recovery))
        return min(planned, self.straight + (self.corner - self.straight) * recovery)

    def throttle(self, steering: float, max_steering: float, corner_weight: float | None = None,
                 dt: float | None = None, recovery_error: float = 0.0) -> float:
        target = self.target(steering, max_steering, corner_weight, recovery_error)
        if dt is None or dt <= 0:
            return target
        if self._current is None:
            self._current = target
        rate = self.ramp_down if target < self._current else self.ramp_up
        self._current += min(abs(target - self._current), rate * dt) * (1 if target >= self._current else -1)
        return self._current


class LineFollowerPilot:
    def __init__(self, cfg: Config, reset_after_gap_s: float = 0.3, clock=time.monotonic,
                 crawl: "AutoCrawl | None" = None, single_line_stop_s: float = 0.0,
                 coast_s: float = 0.0, slow_s: float = 0.0, speed_plan: "SpeedPlan | None" = None,
                 model: CarModel | None = None):
        self.cfg = cfg
        self.model = model or CarModel()
        self.bend_source = None     # callable -> Bend | None (drive_line.py: lambda: det.bend)
        self.path_source = None     # callable -> LineDetectorPart.path (edge ground points) for pure pursuit
        self.racing_active = False          # racing line steering this frame (cfg.racing_line)
        self.racing_fallback_reason = None  # why not, when it is switched on but normal pursuit is steering
        self.racing_offset_m = None         # distance from the inside tape the racing line rides at right now
        self.racing_target = None           # ... as a position on the lookahead row, for the PD correction
        self.racing_ride = None             # the ride line (ground points) for the overlay
        self._racing_k = None               # smoothed curvature of the inside tape
        self._racing_in = False             # turned in for the curve ahead (latched until the next straight)
        self._racing_force_t = None         # when the forced full-lock turn-in began (None = not forcing)
        self._racing_off = None             # smoothed ride distance
        self.conf_source = None     # callable -> detection confidence 0..1 (for the low-confidence slowdown)
        self.overlay_refresh = None # callable -> redraw detector overlay after this pilot update
        self.conf_cap = None        # throttle cap from a low-confidence detection this frame, or None
        self.pursuit_goal = None    # pure-pursuit goal point this frame (x, y) m, or None
        self.curvature = None       # smoothed path curvature ahead, 1/m (+ left)
        self.corner_cap = None      # throttle cap from the corner planner this frame (None = not limiting)
        self.speed_target = None    # section-aware target before the slew limiter
        self.speed_reason = "Speed plan off"
        self.ground = None          # GroundModel for the corner-aware target (built on first use)
        self.target_now = None      # corner-aware target this frame (None = target_x_ratio)
        self.reset_after_gap_s = reset_after_gap_s
        self.clock = clock
        self.ctrl = SteeringController(cfg)
        self.crawl = crawl          # None = fixed LINE_THROTTLE
        self.single_line_stop_s = single_line_stop_s  # 0 = off
        self._t_both = None         # last time both lines were seen
        self.single_line_stopped = False
        self.coast_s, self.slow_s = coast_s, slow_s   # 0, 0 = cut throttle the moment the line is lost
        self.speed_plan = speed_plan                   # None = auto-crawl / fixed throttle
        self._t_lost = None
        self._last_drive_thr = 0.0
        self._t_last = None
        self.last_cmd: Command | None = None
        self.motion_stopped = False
        self.section_now = "unknown"
        self._section_candidate = None
        self._section_since = None

    def stop_motion(self):
        self.motion_stopped = True
        if self.last_cmd is not None:
            self.last_cmd.throttle = 0.0

    def resume_motion(self):
        self.motion_stopped = False
        self._t_last = None
        self.section_now = "unknown"
        self._section_candidate = None

    def _confirmed_section(self, curvature, now):
        if curvature is None or not np.isfinite(curvature):
            self._section_candidate = None
            self.section_now = "unknown"
            return self.section_now
        lo, hi = sorted((self.cfg.corner_blend_start, self.cfg.corner_blend_full))
        w = min(1.0, max(0.0, (abs(curvature) - lo) / max(hi - lo, 1e-6)))
        candidate = "corner" if w >= 0.65 else "straight" if w <= 0.35 else self.section_now
        if candidate == "straight" and self._holding_corner():
            candidate = "corner"   # the road ahead looks straight but the car is still turning
        if candidate != self._section_candidate:
            self._section_candidate, self._section_since = candidate, now
        if now - self._section_since >= 0.15:
            self.section_now = candidate
        return self.section_now

    def _holding_corner(self):
        """Curve exit hold: the camera sees the straight before the car has finished turning.
        Keep curve gains/speed until the steering actually unwinds."""
        last = self.last_cmd
        thr = getattr(self.cfg, "corner_exit_steer", 0.0) or 0.0
        return (thr > 0 and self.section_now == "corner" and last is not None
                and abs(getattr(last, "angle", 0.0)) >= thr * self.cfg.max_steering)

    def _curvature(self, bend):
        """Smoothed curvature of the corner ahead; None when nothing usable is in view."""
        if bend is None or not np.isfinite(bend.curvature):
            self.curvature = None
            return None
        a = min(max(self.cfg.curvature_smoothing, 0.0), 0.95)
        k = bend.curvature
        self.curvature = k if self.curvature is None else a * self.curvature + (1 - a) * k
        return self.curvature

    def _section_gains(self, curvature):
        """Blend straight/corner gains continuously; zero section gains fall back to kp/kd."""
        c = self.cfg
        skp = c.straight_kp or c.kp
        skd = c.straight_kd or c.kd
        ckp = c.corner_kp or c.kp
        ckd = c.corner_kd or c.kd
        if curvature is None:
            return ckp, ckd, 1.0, "unknown"
        lo, hi = sorted((max(0.0, c.corner_blend_start), max(0.001, c.corner_blend_full)))
        if hi <= lo:
            w = 1.0 if abs(curvature) >= hi else 0.0
        else:
            w = min(1.0, max(0.0, (abs(curvature) - lo) / (hi - lo)))
        kp = skp + (ckp - skp) * w
        kd = skd + (ckd - skd) * w
        return kp, kd, w, ("corner" if w >= 0.5 else "straight")

    def _effective_steer_max_deg(self):
        """Racing Line can tune steering response without affecting Race/PD presets."""
        c = self.cfg
        return max(1.0, c.racing_steer_max_deg if c.racing_line else c.steer_max_deg)

    def _pursuit(self, path, bend=None):
        """Pure-pursuit steering command for this frame, or None when the edge can't be aimed at
        (then the frame falls back to plain PD). Never lets geometry trouble stop the car loop."""
        self.pursuit_goal = None
        try:
            if path is None:
                return None
            ground, source, lane_ratio = path
            if self.ground is None:
                self.ground = GroundModel(self.model)
            c = self.cfg
            full_lock_deg = self._effective_steer_max_deg()
            # where the tracked edge runs beside the car when it rides at the target
            lateral = self._racing_lateral(path, bend)
            if lateral is None:
                lateral = self.ground.lateral_at(c.lookahead_ratio, c.target_x_ratio + (lane_ratio if source == "right" else 0.0))
            if lateral is None:
                return None
            v = speed_of_cmd(self.model, self._last_drive_thr)
            ld = max(0.15, c.pursuit_ld_m + c.pursuit_ld_gain_s * v)
            goal = pursuit_goal(ground, lateral, ld)
            if goal is None:
                return None
            if c.pursuit_predict_s > 0:
                # the frame shows where the car WAS: aim from where it will be when this command lands,
                # assuming it holds its current speed and wheel angle (command + = right, wheel + = left)
                wheel = -self.ctrl.angle * full_lock_deg
                goal = predict_goal(goal, v, wheel, self.model.wheelbase_m, c.pursuit_predict_s)
            self.pursuit_goal = goal
            return pursuit_steer(goal, self.model.wheelbase_m, full_lock_deg)
        except Exception:
            return None

    def _racing_lateral(self, path, bend):
        """Racing line: the tracked edge's distance from the ride line for this frame, or None when
        normal pursuit (the fixed target) should steer. Wide on straights, at the apex distance in curves."""
        self.racing_active, self.racing_fallback_reason = False, None
        self.racing_offset_m = self.racing_target = self.racing_ride = None
        c = self.cfg
        if not c.racing_line:
            return None
        ground, source, lane_ratio = path
        a = self.ground.lateral_at(c.lookahead_ratio, c.target_x_ratio)
        b = self.ground.lateral_at(c.lookahead_ratio, c.target_x_ratio + lane_ratio)
        if a is None or b is None:
            self.racing_fallback_reason = "lane width unknown"
            return None
        lane_m = a - b
        if bend is not None and np.isfinite(bend.edge_curvature):
            k = inner_curvature(bend.edge_curvature, source, lane_m)
            self._racing_k = k if self._racing_k is None else 0.5 * self._racing_k + 0.5 * k
        if self._racing_k is None:
            self.racing_fallback_reason = "curve shape not visible yet"
            return None
        weight = None
        if c.racing_turn_in_m < 1.45:
            # Stay wide down the straight; go for the apex only once the curve starts within turn_in_m,
            # then hold it until the curve is behind (the visible curvature has gone). Aiming at the apex
            # whenever a curve is VISIBLE makes the car creep to the inside along the straight, arrive at
            # the tape at the curve's entry and run wide through its middle at full lock (real laps, Nov 11).
            seen = min(1.0, max(0.0, (self._racing_k - CURVE_START) / (CURVE_FULL - CURVE_START)))
            ahead = curve_start_distance(ground)
            if seen < 0.15:
                self._racing_in, self._racing_force_t = False, None
            if ahead is not None and ahead <= c.racing_turn_in_m and seen >= 0.15:
                if not self._racing_in:
                    self._racing_force_t = self._t_last        # turn in NOW: see run()
                self._racing_in = True
            weight = seen * seen * (3.0 - 2.0 * seen) if self._racing_in else 0.0
        # No steering-lock floor on the cut: the driver wants the car right over the inside tape, and on the real
        # car the floor (about -3 to -5 cm on this mat) silently capped every deeper setting.
        off, diag = ride_offset(self._racing_k, lane_m, c.racing_apex_m, c.racing_straight_m, None, weight)
        if off is None:
            self.racing_fallback_reason = diag["reason"]
            return None
        if weight is not None:                       # no step in the aim when a curve comes into view
            off = off if self._racing_off is None else 0.6 * self._racing_off + 0.4 * off
        self._racing_off = off
        self.racing_active, self.racing_offset_m = True, off
        self.racing_target = self.ground.x_ratio_for_lateral(c.lookahead_ratio, off)
        lateral = off - lane_m if source == "right" else off
        self.racing_ride = ride_line(ground, lateral)
        return lateral

    def run(self, mode, x_ratio, found, frame_age, motion=None, both=True, bend=None, path=None):
        if bend is None and self.bend_source is not None:
            bend = self.bend_source()
        if path is None and self.path_source is not None:
            path = self.path_source()
        now = self.clock()
        # not called while in 'user' mode -> start fresh after a mode switch
        if self._t_last is None or now - self._t_last > self.reset_after_gap_s:
            self.ctrl.reset()
            if self.crawl is not None:
                self.crawl.reset()
            self._t_both = now
            self._t_lost, self._last_drive_thr = None, 0.0
            self.curvature = self._racing_k = self._racing_off = None
            self._racing_in, self._racing_force_t = False, None
            if self.speed_plan is not None:
                self.speed_plan.reset()
            dt = 1.0 / 20
        else:
            dt = now - self._t_last
        self._t_last = now
        age = float("inf") if frame_age is None else frame_age
        ctl_mode = DRIVE if mode == "local" else STEER_ONLY
        k = self._curvature(bend) if found else None
        ff, target, ff_scale, gain_scale = 0.0, None, None, 1.0
        pursuit = self._pursuit(path, bend) if (found and self.cfg.pure_pursuit) else None
        if pursuit is None:
            self.racing_active = False
        elif self.racing_active and self._racing_force_t is not None:
            # Timed turn-in: full lock from the turn-in point, while the car is still out wide. The car's turning
            # circle at speed (about 0.78 m) is bigger than the inside tape's (0.58 m), so where the car is closest
            # to the tape is set only by where full lock begins: begin on the tape at the entry and the closest
            # point IS the entry; begin about a quarter metre before the curve from the wide line and it is the apex.
            # Pursuit alone asks for about 0.7 there and gets to full lock 0.3 s late. Hand back once pursuit itself
            # wants full lock, or after 0.6 s whatever happens.
            turn = -1.0 if (self._racing_k or 0.0) >= 0 else 1.0
            if abs(pursuit) >= 0.9 and pursuit * turn > 0 or self._t_last - self._racing_force_t > 0.6:
                self._racing_force_t = None
            else:
                pursuit = turn
        if not self.cfg.pure_pursuit:
            self.pursuit_goal = None
        if pursuit is not None:
            # pure pursuit steers; PD on the lookahead row stays as a small correction
            turn_in = min(1.0, max(0.0, getattr(self.cfg, "pursuit_turn_in", 0.0) or 0.0))
            if k is not None and turn_in > 0 and abs(k) >= self.cfg.corner_blend_start:
                # Curve entry: the inner edge leaves the image before the lookahead point, so pursuit aims at
                # the start of the curve and turns in late. The fitted curvature already shows the bend: pull
                # the command toward the steering that follows it, only ever further INTO the curve.
                want = steer_for_curvature(self.model, k, self._effective_steer_max_deg())
                if (want < 0 and want < pursuit) or (want > 0 and want > pursuit):
                    pursuit = pursuit + turn_in * (want - pursuit)
            ff, ff_scale = pursuit, 1.0
            gain_scale = min(1.0, max(0.0, self.cfg.pursuit_pd))
            # the PD correction must pull toward the line pursuit is riding, not the fixed target
            racing = self.racing_active and self.racing_target is not None
            base = self.racing_target if racing else self.cfg.target_x_ratio
            target = base if racing else None
            if k is not None and gain_scale > 0:
                # In a curve the edge sits left of the target even when the car is exactly on its
                # line. Aim the PD correction at where the edge SHOULD be, or it adds a second
                # helping of steering and the car cuts the inside line.
                xe = self.ground.expected_x_ratio(self.cfg.lookahead_ratio, base, k)
                if xe is not None:
                    target = xe
        elif k is not None and self.cfg.ff_gain > 0:
            ff = steer_for_curvature(self.model, k, self._effective_steer_max_deg())
            # aim PD at where the edge SHOULD appear in this corner, by the same fraction as the
            # feedforward (otherwise PD and feedforward both steer for the curve: too tight)
            if self.ground is None:
                self.ground = GroundModel(self.model)
            xe = self.ground.expected_x_ratio(self.cfg.lookahead_ratio, self.cfg.target_x_ratio, k)
            if xe is not None:
                target = self.cfg.target_x_ratio + self.cfg.ff_gain * (xe - self.cfg.target_x_ratio)
        self.target_now = target
        kp, kd, corner_weight, section = self._section_gains(k)
        if k is not None and self._holding_corner():
            kp, kd, corner_weight, _ = self._section_gains(float("inf"))
        section = self._confirmed_section(k, now)
        cmd = self.ctrl.update(x_ratio, bool(found), now, dt, age, ctl_mode, ff=ff, target=target,
                               gains=(kp * gain_scale, kd * gain_scale), corner_weight=corner_weight,
                               section=section, ff_scale=ff_scale)
        cmd.section = section
        cmd.corner_weight = corner_weight
        throttle = min(max(cmd.throttle, 0.0), self.cfg.max_throttle)
        # one line only: it may be the wrong one. The timer only runs while the car is
        # actually rolling (a parked car seeing one line is not drifting anywhere).
        moving = self.crawl.moving if self.crawl is not None else True
        if both or not moving:
            self._t_both = now
        self.single_line_stopped = bool(self.single_line_stop_s > 0 and not both
                                        and now - self._t_both > self.single_line_stop_s)
        if self.speed_plan is not None and mode == "local" and cmd.status == "TRACKING":
            # straight speed when steering is centred, easing to corner speed as steering demand rises;
            # if the car isn't rolling yet, let auto-crawl ramp it up past the motor's start point
            # React to the visible bend on entry; use filtered curvature on exit.
            # A single straight-looking frame should not release corner regulation.
            preview_weight = self._section_gains(bend.curvature)[2] if k is not None else 1.0
            speed_weight = max(preview_weight, corner_weight)
            if section != "straight":
                speed_weight = max(speed_weight, 0.5)
            self.speed_target = self.speed_plan.target(cmd.angle, self.cfg.max_steering, speed_weight, cmd.error)
            normal_target = self.speed_plan.target(0.0, self.cfg.max_steering, speed_weight)
            self.speed_reason = ("Unknown geometry" if k is None else "Recovery correction" if self.speed_target < normal_target - 1e-6
                                 else "Corner" if speed_weight >= 0.95 else "Transition" if speed_weight > 0 else "Settled straight")
            throttle = self.speed_plan.throttle(cmd.angle, self.cfg.max_steering, speed_weight, dt, cmd.error)
            if k is None:
                # Geometry loss must never promote the car to straight speed.
                throttle = min(throttle, self.speed_plan.corner)
            if self.crawl is not None and not self.crawl.moving:
                throttle = max(throttle, min(self.speed_target, self.crawl.update(motion, dt)))
            elif self.crawl is not None and motion is not None:
                self.crawl.hist = (self.crawl.hist + [motion])[-3:]
            throttle = min(throttle, self.cfg.max_throttle)
        elif self.crawl is not None and mode == "local":
            if cmd.status == "TRACKING":
                if both or not moving:
                    throttle = self.crawl.update(motion, dt)   # may ramp up to get going
                else:
                    if motion is not None:
                        self.crawl.hist = (self.crawl.hist + [motion])[-3:]
                    throttle = self.crawl.thr  # rolling on a single line: never speed up
                throttle = min(throttle, self.cfg.max_throttle)
            else:
                self.crawl.hist = []  # it will be stopping; re-measure before ramping again
        else:
            self.speed_target = None
        self.corner_cap = None
        if self.cfg.corner_plan and mode == "local" and cmd.status == "TRACKING" and k is not None and bend is not None:
            # brake early enough to reach the grip-limited speed of the corner in view
            v_now = speed_of_cmd(self.model, self._last_drive_thr or throttle)
            v_ok = allowed_speed(self.model, Bend(k, bend.edge_curvature, bend.lateral_m, bend.near_m, bend.far_m, bend.n, bend.resid_m),
                                 v_now, self.cfg.corner_margin)
            if v_ok < 1e6:
                cap = max(self.cfg.corner_min_throttle, cmd_for_speed(self.model, v_ok))
                if cap < throttle:
                    self.corner_cap, throttle = cap, cap
        self.conf_cap = None
        if (self.cfg.conf_slow_below > 0 and self.speed_plan is not None and mode == "local" and cmd.status == "TRACKING"
                and self.conf_source is not None):
            # a shaky detection (glare, half the line out of view) is not the moment for straight speed:
            # blend down to curve speed as confidence falls from conf_slow_below to min_confidence
            conf = self.conf_source()
            if conf is not None and conf < self.cfg.conf_slow_below:
                lo = min(self.cfg.min_confidence, self.cfg.conf_slow_below - 1e-3)
                w = min(1.0, max(0.0, (conf - lo) / (self.cfg.conf_slow_below - lo)))
                floor = self.speed_plan.corner
                top = self.speed_target if self.speed_target is not None else throttle   # scale the plan's target, not
                cap = floor + (max(top, floor) - floor) * w                               # last frame's capped throttle
                if cap < throttle:
                    self.conf_cap, throttle = cap, cap
        if self.speed_plan is not None and mode == "local" and cmd.status == "TRACKING" and k is None:
            # Apply after auto-crawl too: a startup boost cannot override unknown-geometry limits.
            throttle = min(throttle, self.speed_plan.corner)
        coasting = False
        if cmd.status == "LOST" and mode == "local" and not self.single_line_stopped and (self.coast_s or self.slow_s):
            # Team-10 style: hold heading and keep rolling briefly (lines flicker), then ramp down to a stop.
            self._t_lost = now if self._t_lost is None else self._t_lost
            blind = now - self._t_lost
            if blind <= self.coast_s:
                throttle, coasting = self._last_drive_thr, True
            elif blind <= self.coast_s + self.slow_s:
                throttle, coasting = self._last_drive_thr * (1 - (blind - self.coast_s) / max(self.slow_s, 1e-6)), True
        elif cmd.status == "TRACKING":
            self._t_lost = None
        if (cmd.status != "TRACKING" and not coasting) or self.single_line_stopped:
            throttle = 0.0
        if self.motion_stopped:
            throttle = 0.0
        if self.speed_plan is not None and mode == "local":
            self.speed_plan._current = throttle  # Keep the ramp aligned with applied safety caps.
        if cmd.status == "TRACKING" and not self.single_line_stopped:
            self._last_drive_thr = throttle
        cmd.throttle = throttle
        self.last_cmd = cmd
        if self.overlay_refresh is not None:
            self.overlay_refresh()
        return float(cmd.angle), float(throttle)

    def active_cmd(self) -> Command | None:
        """Last command, or None if the pilot isn't running (user mode)."""
        if self._t_last is None or self.clock() - self._t_last > self.reset_after_gap_s:
            return None
        return self.last_cmd


class ThrottleCap:
    """Last part before the motor: speed limit + slow-down in turns.

    - Every mode: forward throttle is clipped to TOP_THROTTLE, and to that mode's
      own maximum (MAX_USER / MAX_PILOT_STEERS / MAX_AUTO), whichever is lower.
    - Pilot modes (local_angle and local): forward throttle is also scaled down
      the harder the wheels are turned, so the car slows into curves instead of
      running wide (it is faster than it can turn):
          scale = max(MIN_TURN_SCALE, 1 - TURN_SLOWDOWN * |steering| / max_steering)
      This applies to the human's stick in local_angle too.
    - Negative values (reverse, and the gamepad E-stop's braking pulses) pass
      through untouched, so Y keeps working exactly as before.
    """

    def __init__(self, top: float, turn_slowdown: float = 0.0, min_turn_scale: float = 1.0,
                 max_steering: float = 1.0, mode_caps: dict | None = None):
        self.top = max(0.0, min(float(top), 1.0))
        # per-mode maximum forward throttle; a missing mode just uses `top`
        self.mode_caps = {m: max(0.0, min(float(v), 1.0)) for m, v in (mode_caps or {}).items()}
        self.turn_slowdown = max(0.0, float(turn_slowdown))
        self.min_turn_scale = max(0.0, min(float(min_turn_scale), 1.0))
        self.max_steering = max(1e-3, float(max_steering))

    def run(self, throttle, steering=None, mode=None):
        if throttle is None:
            return 0.0
        t = float(throttle)
        if t <= 0.0:
            return t
        t = min(t, self.limit(mode))  # cap first, so turn slow-down still bites at full stick
        if mode in ("local_angle", "local") and steering is not None:
            frac = min(abs(float(steering)) / self.max_steering, 1.0)
            t *= max(self.min_turn_scale, 1.0 - self.turn_slowdown * frac)
        return t

    def limit(self, mode=None) -> float:
        """The forward-throttle ceiling that applies in this mode right now."""
        return min(self.top, self.mode_caps.get(mode, self.top))


class LineRecorder:
    """Saves raw camera frames + what the pilot decided, while the pilot runs.

    Output: <root>/<YYYYmmdd-HHMMSS>/NNNNNN.jpg and log.csv. Only ever writes
    under <root> (~/linefollow/data). Replay a folder on the Mac with the
    dashboard: python3 -m linetrack --source <folder>.
    """

    def __init__(self, root: str, detector_part: LineDetectorPart, clock=time.time, battery_monitor=None):
        import os
        self.os = os
        self.root = root
        self.det = detector_part
        self.clock = clock
        self.dir = None
        self.csv = None
        self.n = 0
        self.pilot = detector_part.pilot  # for status / P / D / crawl columns
        self.battery_monitor = battery_monitor

    def _open(self):
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.dir = self.os.path.join(self.root, stamp)
        self.os.makedirs(self.dir, exist_ok=True)
        self.csv = open(self.os.path.join(self.dir, "log.csv"), "w", buffering=1)
        self.csv.write("n,t,mode,found,source,conf,x_ratio,frame_age,angle,throttle,motion,battery_v,"
                       "status,error,p,d,both,single_stop,crawl_thr,curv,bend_near_m,ff,section,corner_weight,kp_used,kd_used,speed_target\n")

    def run(self, img_rgb, mode, x_ratio, found, frame_age, angle, throttle, motion=None):
        if img_rgb is None:
            return
        if self.csv is None:
            self._open()
        d = self.det._det
        src = d.source if d is not None else "none"
        conf = d.confidence if d is not None else 0.0
        name = f"{self.n:06d}.jpg"
        cv2.imwrite(self.os.path.join(self.dir, name), cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        fmt = lambda v: "" if v is None else f"{v:.4f}"  # noqa: E731
        battery = self.battery_monitor.snapshot() if self.battery_monitor is not None else {}
        battery_v = battery.get("voltage_v") if battery.get("available") else None
        self.csv.write(f"{self.n},{self.clock():.3f},{mode},{int(bool(found))},{src},{conf:.3f},"
                       f"{fmt(x_ratio)},{fmt(frame_age)},{fmt(angle)},{fmt(throttle)},{fmt(motion)},{fmt(battery_v)},"
                       f"{self._extra()}\n")
        self.n += 1

    def _extra(self) -> str:
        p = self.pilot
        cmd = p.last_cmd if p is not None else None
        both = int(self.det.both_lines())
        b = self.det.bend          # raw corner curvature of this frame (validate it on clean laps)
        geo = "," if b is None else f"{b.curvature:.4f},{b.near_m:.3f}"
        if cmd is None:
            curv, near = ("", "") if b is None else (f"{b.curvature:.4f}", f"{b.near_m:.3f}")
            return ",".join(["", "", "", "", str(both), "", "", curv, near, "", "", "", "", "", ""])
        stop = int(bool(getattr(p, "single_line_stopped", False)))
        crawl = getattr(p, "crawl", None)
        cthr = "" if crawl is None else f"{crawl.thr:.4f}"
        curv, near = ("", "") if b is None else (f"{b.curvature:.4f}", f"{b.near_m:.3f}")
        return ",".join([cmd.status, f"{cmd.error:.4f}", f"{cmd.p:.4f}", f"{cmd.d:.4f}", str(both), str(stop), cthr,
                         curv, near, f"{cmd.ff:.4f}", cmd.section, f"{cmd.corner_weight:.4f}",
                         f"{cmd.kp_used:.4f}", f"{cmd.kd_used:.4f}",
                         "" if p.speed_target is None else f"{p.speed_target:.4f}"])

    def shutdown(self):
        if self.csv is not None:
            self.csv.close()


class KillSwitch:
    """Gamepad X = turn the line follower OFF completely.

    trigger() is called from the gamepad thread. From then on this part (the
    last one before the motors) forces steering 0 and throttle 0, and after
    `hold_s` it raises KeyboardInterrupt in the vehicle loop, which is exactly
    what Ctrl-C does: Donkeycar shuts every part down (PWM to neutral, camera
    closed) and drive_line.py exits.
    """

    def __init__(self, hold_s: float = 0.5, clock=time.monotonic):
        self.hold_s = hold_s
        self.clock = clock
        self.t_kill = None

    def trigger(self):
        if self.t_kill is None:
            self.t_kill = self.clock()
            print("[linefollow] X pressed: STOPPING and shutting down", flush=True)

    def run(self, steering, throttle):
        if self.t_kill is None:
            return steering, throttle
        if self.clock() - self.t_kill >= self.hold_s:
            raise KeyboardInterrupt("kill switch (gamepad X)")
        return 0.0, 0.0


class ThrottleMonitor:
    """Debug: prints every throttle stage to the log twice a second (and on any change of mode)."""

    def __init__(self, every_s: float = 0.5, clock=time.monotonic):
        self.every_s = every_s
        self.clock = clock
        self.t_last = 0.0
        self.last_mode = None

    def run(self, mode, user_throttle, pilot_throttle, drive_throttle, final_throttle, steering, motion=None):
        now = self.clock()
        if mode != self.last_mode or now - self.t_last >= self.every_s:
            self.t_last, self.last_mode = now, mode
            f = lambda v: "None" if v is None else f"{v:+.3f}"  # noqa: E731
            pwm = "" if final_throttle is None else f" pwm~{370 + 37 * max(final_throttle, 0):.0f}"
            print(f"[thr] mode={mode} stick={f(user_throttle)} pilot={f(pilot_throttle)} "
                  f"drivemode={f(drive_throttle)} final={f(final_throttle)}{pwm} steer={f(steering)} "
                  f"motion={'None' if motion is None else f'{motion:.1f}'}", flush=True)
