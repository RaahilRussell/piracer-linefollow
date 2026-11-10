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
    # --- Mask growth: nearly-orange pixels count when they touch strictly-orange ones ---
    mask_grow_px: int = 3               # how far (px) the strict mask may grow into nearly-orange pixels; 0 = off
    weak_h_margin: int = 4              # "nearly orange": hue this much past h_hi / before h_wrap_lo ...
    weak_s_margin: int = 30             # ... saturation this much below s_lo ...
    weak_v_margin: int = 30             # ... brightness this much below v_lo
    # --- Line candidates: orange runs linked from scanline to scanline ---
    run_merge_gap_px: int = 3           # runs on one row closer than this are one run (speckle holes)
    chain_gap_ratio: float = 0.04       # a line may vanish for this fraction of the height and still be one line
    chain_link_px: float = 2.0          # sideways slack per scanline when linking runs
    max_run_width_ratio: float = 0.45   # rows where the stripe is wider than this give no edge point
    acquire_min_span_ratio: float = 0.08  # a NEW line must be at least this tall (fraction of height) ...
                                        # ... and reach into the lower 80% of the ROI (not a horizon fragment)
    max_edge_roughness_px: float = 2.5  # tape edges are smooth; raggeder candidates (wood grain, skin) are dropped
    local_fit_resid_px: float = 2.0     # if one curve misses the edge by more than this, fit near the lookahead only
    local_fit_window_ratio: float = 0.2
    # --- Telling the two boundaries apart ---
    horizon_ratio: float = 0.42         # image row of the horizon (lane width shrinks to 0 there)
    lane_min_ratio: float = 0.25        # two lines are one lane if their gap is within these multiples
    lane_max_ratio: float = 1.8         # of the expected lane width on that row
    track_memory_frames: int = 8        # a boundary that vanishes keeps its place this many frames
    side_memory_frames: int = 40        # a line reappearing where LEFT/RIGHT was last seen keeps that name
    lone_confirm_frames: int = 1        # a single line with no history must hold this many frames first (1 = at once)
    lone_center_margin: float = 0.15    # a single line passing this close to the car's centre is ambiguous
    bracket_margin: float = 0.15        # slack for "the car sits between these two lines"
    flip_confirm_frames: int = 3        # frames of evidence before a tracked line is renamed LEFT<->RIGHT
    # --- Lost line: hold heading instead of re-centring the wheels ---
    lost_hold_heading: bool = False
    num_scanlines: int = 48        # rows sampled in the ROI (dense: small far fragments count)
    min_component_area: int = 25   # pixels, at processing resolution
    lean_tolerance: float = 0.15   # |dx/dy| below this counts as "vertical" when classifying a lone stripe
    reject_right_ratio: float = 0.62  # a lone stripe starting right of this is treated as the right boundary
    max_blob_width_ratio: float = 0.22  # reject candidates thicker than this across the line (hands etc.)
    max_jump_ratio: float = 0.12   # max allowed edge shift between frames (continuity gate)
    process_width: int = 240       # frames wider than this are downscaled before processing
    # --- Right-line fallback (when the inside/left line leaves the view in turns) ---
    use_right_fallback: bool = True
    lane_width_ratio: float = 0.47     # initial lane width at the lookahead row; learned live when both lines show
    right_fallback_conf_scale: float = 0.9
    handoff_decay: float = 0.8         # per frame: how fast the left<->right switch offset fades (0 = instant jump)

    # --- Controller ---
    kp: float = 2.2
    kd: float = 0.25
    # Optional section-specific gains. Zero means "use kp/kd" for backwards compatibility.
    straight_kp: float = 0.0
    straight_kd: float = 0.0
    corner_kp: float = 0.0
    corner_kd: float = 0.0
    corner_blend_start: float = 0.60       # curvature (1/m) where corner gains begin blending in (0.6/1.2: 30% straight / 70% curve on real frames, matching the traced Waveshare track)
    corner_blend_full: float = 1.20        # curvature (1/m) where corner gains are fully active
    corner_exit_steer: float = 0.0         # curve exit hold: stay in curve mode while |steering| is above this fraction of max; 0 = off
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
    # Wheel angle at steering command +/-1 (full lock). ONE value for the car and the simulator:
    # sim/profile.json's steer_angle_table must agree (tests/test_geometry.py checks it).
    # 18 deg is FITTED to the recorded runs (tools/calibrate_sim.py), not measured with a ruler.
    steer_max_deg: float = 18.0
    # --- Pure pursuit (alternative steering law; False = PD on the lookahead row + feedforward) ---
    # Aim at the point of the ride line (tracked edge moved sideways by the target offset) that lies
    # pursuit_ld_m + pursuit_ld_gain_s * speed ahead: wheel angle = atan(2 * wheelbase * sin(alpha) / Ld).
    pure_pursuit: bool = False
    pursuit_ld_m: float = 0.55         # lookahead distance Ld0, metres from the rear axle
    pursuit_ld_gain_s: float = 0.0     # extra lookahead per m/s of (modelled) speed
    pursuit_pd: float = 0.3            # share of KP/KD kept as a correction on top of pure pursuit

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
