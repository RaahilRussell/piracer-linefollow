"""Conservative, one-lap-at-a-time speed ladder assessment."""
from __future__ import annotations

import math


def assess_lap(samples, speed, *, step=0.05, minimum_samples=40):
    """Assess recorded telemetry; steering saturation alone never defines failure."""
    speed = float(speed)
    if not math.isfinite(speed) or not 0 <= speed <= 1:
        raise ValueError("speed must be between 0 and 1")
    step = float(step)
    if not math.isfinite(step) or not 0.01 <= step <= 0.12:
        raise ValueError("speed step must be between 0.01 and 0.12")
    clean = [s for s in (samples or []) if isinstance(s, dict)]
    if len(clean) < minimum_samples:
        return {"result": "inconclusive", "samples": len(clean), "speed": speed,
                "next_speed": speed, "tracking_pct": None, "error_p90": None,
                "steering_limit_pct": None,
                "message": "Not enough telemetry; repeat this speed before increasing."}

    tracked = [s for s in clean if s.get("status") == "TRACKING" and s.get("found")]
    errors = sorted(abs(float(s["error"])) for s in tracked
                    if isinstance(s.get("error"), (int, float)) and math.isfinite(float(s["error"])))
    tracking_pct = 100 * len(tracked) / len(clean)
    p90 = errors[min(len(errors) - 1, math.ceil(0.9 * len(errors)) - 1)] if errors else None
    saturated = [s for s in clean if isinstance(s.get("steering"), (int, float))
                 and abs(float(s["steering"])) >= 0.95]
    saturation_pct = 100 * len(saturated) / len(clean)

    if tracking_pct < 85 or p90 is None or p90 > 0.22:
        result, next_speed = "failed", speed
        message = "Line tracking degraded; this is the failure point. Keep this speed stopped and retry at the last clean speed."
    elif tracking_pct >= 95 and p90 <= 0.12:
        result, next_speed = "passed", min(1.0, round(speed + step, 2))
        message = (f"Good line tracking. Next run is staged at {next_speed:.2f}; start it when ready."
                   if next_speed > speed else "Good line tracking at the configured speed limit.")
    else:
        result, next_speed = "repeat", speed
        message = "Mixed tracking; repeat this speed before increasing it."
    return {"result": result, "samples": len(clean), "speed": speed, "next_speed": next_speed,
            "tracking_pct": round(tracking_pct, 1), "error_p90": round(p90, 3) if p90 is not None else None,
            "steering_limit_pct": round(saturation_pct, 1), "message": message}


def estimate_lap(straight, corner, speed_table, track_length_m, curve_share, ride_offset_m=0.0,
                 pwm_stop=370.0, pwm_full=407.0):
    """ESTIMATED lap time (s) for a throttle pair, before any lap is driven.

    Track: length of the traced inside line and the share of it that is curve. The car rides
    ride_offset_m outside that line, so its path is longer by 2*pi*offset per lap. Speed per throttle
    comes from the speed table [(PWM, m/s), ...], which is ASSUMED until the car is timed. Throttle
    becomes a whole PWM tick exactly as Donkeycar does it, so 0.92 and 0.93 are the same speed.
    Returns None when either throttle doesn't move the car."""
    xs = [float(p[0]) for p in speed_table]
    ys = [float(p[1]) for p in speed_table]

    def speed(cmd):
        pwm = int(pwm_stop + max(0.0, min(1.0, float(cmd))) * (pwm_full - pwm_stop))
        if pwm <= xs[0]:
            return ys[0]
        for (x0, y0), (x1, y1) in zip(zip(xs, ys), zip(xs[1:], ys[1:])):
            if pwm <= x1:
                return y0 + (y1 - y0) * (pwm - x0) / max(x1 - x0, 1e-9)
        return ys[-1]
    vs, vc = speed(straight), speed(min(corner, straight))
    if vs <= 0.01 or vc <= 0.01:
        return None
    share = min(1.0, max(0.0, float(curve_share)))
    path = float(track_length_m) + 2 * math.pi * max(0.0, float(ride_offset_m))
    return round(path * (share / vc + (1 - share) / vs), 2)


OFF_TRACK_PENALTY_S = 5.0      # competition rule: off track
REPOSITION_PENALTY_S = 10.0    # competition rule: car put back by hand


class LapTimer:
    """Lap times without a start line: one lap = `sections_per_lap` confirmed straight/curve changes
    (4 on an oval: straight, curve, straight, curve). Only counts while Full Auto is driving.

    Made robust against the two things that made lap times jump around:
      - a brief flicker between straight and curve: a change only counts once the new section has
        held for debounce_s (the lap clock still uses the moment the change began);
      - an impossible lap: if the changes add up before min_lap_s has passed, two of them were a
        flicker pair, so they are dropped and the lap keeps running.
      - a curve the pilot could not fit: with the inner line out of view the section reads "unknown",
        sometimes for the whole curve. Counted as a curve (on the straights both lines are in view), or
        the timer sees straight -> straight and merges two laps into one.
    A lap far from the recent median (a missed or extra section) is kept but marked suspect and left
    out of the best lap. Also counts line losses per lap and laps cut short by a stop, so the race
    total can include the penalties they would cost."""

    def __init__(self, sections_per_lap=4, race_laps=10, debounce_s=0.10, min_lap_s=2.0):
        self.sections_per_lap = int(sections_per_lap)
        self.race_laps = int(race_laps)
        self.debounce_s = float(debounce_s)
        self.min_lap_s = float(min_lap_s)
        self.reset()

    def reset(self):
        self.laps = []                  # [{"lap", "seconds", "lost", "suspect"}]
        self.stops = 0                  # laps cut short (car stopped / taken over): a reposition each
        self._clear_lap()

    def _clear_lap(self):
        self._last_section = None
        self._pending = None            # (section, time it first appeared) while it is being confirmed
        self._changes = 0
        self._lap_start = None
        self._lost_now = False
        self._lost_this_lap = 0

    def update(self, now, mode, section, tracking):
        if mode != "local":
            if self._lap_start is not None:
                self.stops += 1          # a lap was under way and the car stopped driving itself
            self._clear_lap()
            return
        if not tracking:
            if not self._lost_now and self._lap_start is not None:
                self._lost_this_lap += 1
            self._lost_now = True
            return
        self._lost_now = False
        if section == "unknown":
            section = "corner"
        if section not in ("straight", "corner"):
            return
        if self._last_section is None:
            self._last_section = section
            return
        if section == self._last_section:
            self._pending = None         # it flickered back: never a change
            return
        if self._pending is None or self._pending[0] != section:
            self._pending = (section, now)
        t_change = self._pending[1]
        if now - t_change < self.debounce_s:
            return
        self._last_section, self._pending = section, None
        if self._lap_start is None:      # the first change starts the clock at a section boundary
            self._lap_start, self._changes, self._lost_this_lap = t_change, 0, 0
            return
        self._changes += 1
        if self._changes < max(1, self.sections_per_lap):
            return
        seconds = t_change - self._lap_start
        if seconds < self.min_lap_s and self._changes >= 2:
            self._changes -= 2           # impossible lap: two of those changes were a flicker pair
            return
        recent = sorted(l["seconds"] for l in self.laps[-5:] if not l["suspect"])
        median = recent[len(recent) // 2] if len(recent) >= 3 else None
        suspect = bool(median and not 0.7 * median <= seconds <= 1.4 * median)
        self.laps.append({"lap": len(self.laps) + 1, "seconds": round(seconds, 2),
                          "lost": self._lost_this_lap, "suspect": suspect})
        self.laps = self.laps[-200:]
        self._lap_start, self._changes, self._lost_this_lap = t_change, 0, 0

    def status(self, now):
        times = [l["seconds"] for l in self.laps]
        good = [l["seconds"] for l in self.laps if not l["suspect"]]
        last = self.laps[-self.race_laps:]
        last_n = [l["seconds"] for l in last]
        lost = sum(l["lost"] for l in last)
        penalty = OFF_TRACK_PENALTY_S * lost + REPOSITION_PENALTY_S * self.stops
        full = len(last_n) >= self.race_laps
        return {"laps": self.laps[-30:], "count": len(self.laps),
                "current": None if self._lap_start is None else round(now - self._lap_start, 2),
                "current_sections": self._changes, "sections_per_lap": self.sections_per_lap,
                "last": times[-1] if times else None, "best": min(good) if good else None,
                "mean": round(sum(good) / len(good), 2) if good else None,
                "race_laps": self.race_laps,
                "race_total": round(sum(last_n), 2) if full else None,
                "race_so_far": round(sum(last_n), 2) if last_n else None,
                # each line loss is counted as an off-track, each stopped lap as a reposition
                "race_lost": lost, "race_stops": self.stops, "race_penalty_s": penalty,
                "race_total_with_penalties": round(sum(last_n) + penalty, 2) if full else None,
                "lost_this_lap": self._lost_this_lap}
