"""All tunable settings in one place. Every field can be changed live from the
dashboard (and saved to config.json), or overridden on the command line."""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields
from pathlib import Path


@dataclass
class Config:
    # --- Donkeycar server (verified live: GET /video = MJPEG, /wsDrive = WebSocket) ---
    car_url: str = "http://localhost:8887"
    video_path: str = "/video"
    ws_path: str = "/wsDrive"

    # --- Steering target: where the tracked edge should sit, as a fraction of width ---
    target_x_ratio: float = 0.20

    # --- Orange HSV threshold (OpenCV ranges: H 0-179, S/V 0-255) ---
    h_lo: int = 0
    h_hi: int = 9                 # wood floor/wall is H 4-11, tape H 0-6 or 170-179
    s_lo: int = 100
    s_hi: int = 255
    v_lo: int = 140               # wood is dimmer (V~150) than tape (V~190+)
    v_hi: int = 255
    h_wrap_lo: int = 170          # second hue band for reds that wrap past 179 (set 180 to disable)

    # --- Region of interest / geometry (fractions of image height/width) ---
    roi_top_ratio: float = 0.45   # process rows below this; horizon sits at ~0.42
    roi_bottom_ratio: float = 1.00
    lookahead_ratio: float = 0.62  # row (fraction of height) where steering is measured
    lookahead_from_points: bool = True  # use the measured edge near the lookahead row (not the fitted curve)
    lookahead_band_ratio: float = 0.07  # +/- this fraction of height around the lookahead row
    # --- Wrong-line protection (backtested before enabling; all off = old behaviour) ---
    reacquire_confirm_frames: int = 0   # a stripe picked WITHOUT continuity must hold its place this many
                                        # frames before the tracker locks onto it (0 = lock at once)
    left_lean_swap: bool = False        # if the tracked stripe leans like a RIGHT boundary and a stripe leaning
                                        # like a LEFT boundary is further left, switch to that one
    lean_swap_min_sep_ratio: float = 0.15
    # --- Tape-shape filter (rotation-invariant): reject blobs that aren't tape-shaped ---
    shape_filter: bool = False
    min_rect_fill: float = 0.4          # blob area / its tilted bounding box
    min_solidity: float = 0.6           # blob area / its convex hull (ragged edges score low)
    max_aspect: float = 25.0            # long side / short side of the tilted box
    # --- Lost line: hold heading instead of re-centring the wheels ---
    lost_hold_heading: bool = False
    num_scanlines: int = 32        # rows sampled for edge points (dense: small far fragments count)
    min_component_area: int = 25   # pixels, at processing resolution
    lean_tolerance: float = 0.15   # |dx/dy| below this counts as "vertical" when classifying a lone stripe
    reject_right_ratio: float = 0.62  # a lone stripe starting right of this is treated as the right boundary
    max_blob_width_ratio: float = 0.22  # reject components whose mean row width exceeds this (hands etc.)
    max_jump_ratio: float = 0.12   # max allowed edge shift between frames (continuity gate)
    process_width: int = 240
    # --- Right-line fallback (when the inside/left line leaves the view in turns) ---
    use_right_fallback: bool = True
    lane_width_ratio: float = 0.47     # initial lane width at the lookahead row; learned live when both lines show
    right_fallback_conf_scale: float = 0.9
    handoff_decay: float = 0.8         # per frame: how fast the left<->right switch offset fades (0 = instant jump)       # frames wider than this are downscaled before processing

    # --- Controller ---
    kp: float = 2.2
    kd: float = 0.25
    # Optional section-specific gains. Zero means "use kp/kd" for backwards compatibility.
    straight_kp: float = 0.0
    straight_kd: float = 0.0
    corner_kp: float = 0.0
    corner_kd: float = 0.0
    corner_blend_start: float = 0.25       # curvature (1/m) where corner gains begin blending in
    corner_blend_full: float = 0.90        # curvature (1/m) where corner gains are fully active
    max_steering: float = 0.8
    steering_rate_limit: float = 3.0   # max change in steering per second
    steering_smoothing: float = 0.35   # EMA factor on output (0 = none, 0.9 = very smooth)
    invert_steering: bool = False      # flip if wheels turn the wrong way in the lifted test
    # --- Corner geometry (linetrack/geometry.py) ---
    ff_gain: float = 0.0               # curvature feedforward: steer atan(wheelbase/R) directly; 0 = off, 1 = full
    corner_plan: bool = False          # Full auto: cap speed so the car can brake to the corner's grip limit in time
    corner_margin: float = 0.8         # fraction of the grip-limited corner speed to allow
    corner_min_throttle: float = 0.55  # never plan below this (the motor stalls lower)
    curvature_smoothing: float = 0.5   # EMA on the measured curvature (0 = raw)

    # --- Safety / timing ---
    min_confidence: float = 0.35
    max_frame_age_s: float = 0.35      # no fresh camera content for this long -> stale
    lost_timeout_s: float = 0.6        # after this long without a line, steering re-centres
    throttle: float = 0.18             # used only in DRIVE mode
    max_throttle: float = 0.30         # hard cap, cannot be exceeded from the dashboard
    send_hz: float = 20.0
    dashboard_heartbeat_s: float = 0.8  # dashboard must poll this often or car is stopped

    @classmethod
    def load(cls, path: str | Path | None) -> "Config":
        cfg = cls()
        if path and Path(path).exists():
            cfg.update(json.loads(Path(path).read_text()))
        return cfg

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2))

    def update(self, data: dict) -> dict:
        """Apply known keys with type coercion; return the keys actually applied."""
        applied = {}
        types = {f.name: f.type for f in fields(self)}
        for k, v in data.items():
            if k not in types:
                continue
            cur = getattr(self, k)
            if isinstance(cur, bool):
                v = v if isinstance(v, bool) else str(v).lower() in ("1", "true", "yes", "on")
            else:
                v = type(cur)(float(v)) if isinstance(cur, (int, float)) else type(cur)(v)
            setattr(self, k, v)
            applied[k] = v
        self.throttle = max(0.0, min(self.throttle, self.max_throttle))
        return applied

    def as_dict(self) -> dict:
        return asdict(self)
