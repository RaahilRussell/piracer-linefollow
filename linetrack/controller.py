"""PD steering + safety gating.

Sign convention (verified in the Donkeycar web UI's main.js: angleLeft() decreases
angle, angleRight() increases it): angle -1 = full left, +1 = full right.

error = x_ratio - target_x_ratio
  edge appears RIGHT of target (car drifted left / toward the left line) -> error > 0
  -> steer RIGHT (positive angle). Same formula works for target 0.80.
"""
from __future__ import annotations

from dataclasses import dataclass

from .config import Config

PREVIEW, STEER_ONLY, DRIVE = "preview", "steer_only", "drive"
MODES = (PREVIEW, STEER_ONLY, DRIVE)


@dataclass
class Command:
    angle: float = 0.0
    throttle: float = 0.0
    status: str = "LOST"          # TRACKING / LOST / STALE
    error: float = 0.0
    raw: float = 0.0              # unclipped PD output
    p: float = 0.0                # proportional term  kp * error
    d: float = 0.0                # derivative term    kd * d(error)/dt (filtered)
    ff: float = 0.0               # feedforward term   ff_gain * steering for the measured curve
    kp_used: float = 0.0
    kd_used: float = 0.0
    corner_weight: float = 0.0
    section: str = "unknown"


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class SteeringController:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.angle = 0.0
        self._prev_err: float | None = None
        self._d_filt = 0.0
        self._last_seen_t: float | None = None

    def update(self, x_ratio: float | None, found: bool, now: float, dt: float,
               frame_age: float, mode: str, ff: float = 0.0, target: float | None = None,
               gains: tuple[float, float] | None = None, corner_weight: float = 0.0,
               section: str = "straight", ff_scale: float | None = None) -> Command:
        """ff: steering command that would follow the curve ahead exactly (from its curvature);
        added as ff_gain * ff, so PD only has to correct what's left. ff_scale replaces ff_gain
        when given (pure pursuit passes its whole steering command with ff_scale 1).
        target: where the edge should sit this frame (corner-aware); None = target_x_ratio."""
        c = self.cfg
        dt = clamp(dt, 1e-3, 0.5)
        stale = frame_age > c.max_frame_age_s
        cmd = Command()

        if stale:
            cmd.status = "STALE"
            target = 0.0 if self._last_seen_t is None or now - self._last_seen_t > c.lost_timeout_s else self.angle
            self._prev_err = None
        elif found and x_ratio is not None:
            cmd.status = "TRACKING"
            self._last_seen_t = now
            err = x_ratio - (c.target_x_ratio if target is None else target)
            if self._prev_err is None:
                d = 0.0
            else:
                d = (err - self._prev_err) / dt
            self._d_filt = 0.6 * self._d_filt + 0.4 * d
            self._prev_err = err
            kp, kd = gains if gains is not None else (c.kp, c.kd)
            p_term, d_term = kp * err, kd * self._d_filt
            ff_term = (c.ff_gain if ff_scale is None else ff_scale) * ff
            if c.invert_steering:          # mirrored actuator: every steering command flips
                p_term, d_term, ff_term = -p_term, -d_term, -ff_term
            raw = p_term + d_term + ff_term
            cmd.error, cmd.raw, cmd.p, cmd.d, cmd.ff = err, raw, p_term, d_term, ff_term
            cmd.kp_used, cmd.kd_used = kp, kd
            cmd.corner_weight, cmd.section = corner_weight, section
            target = clamp(raw, -c.max_steering, c.max_steering)
        else:
            cmd.status = "LOST"
            self._prev_err = None
            # hold last steering briefly (line flicker), then re-centre
            recent = self._last_seen_t is not None and now - self._last_seen_t <= c.lost_timeout_s
            # hold heading (Team 10): never re-centre while blind; the throttle side slows/stops the car
            target = self.angle if (recent or c.lost_hold_heading) else 0.0

        # smoothing (EMA) then rate limit
        a = clamp(c.steering_smoothing, 0.0, 0.95)
        smoothed = a * self.angle + (1 - a) * target
        max_step = c.steering_rate_limit * dt
        self.angle = clamp(self.angle + clamp(smoothed - self.angle, -max_step, max_step),
                           -c.max_steering, c.max_steering)
        cmd.angle = self.angle

        # throttle only in DRIVE mode, only while TRACKING with fresh frames
        if mode == DRIVE and cmd.status == "TRACKING":
            cmd.throttle = clamp(c.throttle, 0.0, c.max_throttle)
        else:
            cmd.throttle = 0.0
        return cmd
