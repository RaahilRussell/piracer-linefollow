"""Tuning health: are KP / KD (and the speed) right, judged from the driving data?

Works on a window of closed-loop samples (the pilot actually steering a moving
car): time, error, steering, P term, D term, tracking status. Signs it looks for:

  weave growing      error swings across zero and each swing is bigger      -> P too high
  steady weave       swings keep the same size                              -> P a bit high / D too low
  lagging            error stays on one side, steering NOT at its limit      -> P too low
  can't turn         error one-sided while steering IS at its limit         -> too fast for the corner
  jitter             steering chatters frame to frame while error is small  -> D too high (noise)
  sluggish           error returns to zero slowly with no overshoot         -> over-damped (D/smoothing)

Plus a damping ratio estimate (log decrement between successive swings);
around 0.4-0.8 is the target. Pure functions, no car needed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Health:
    sufficient: bool = False
    n: int = 0
    seconds: float = 0.0
    tracked: float = 0.0              # fraction of samples tracking
    mean_abs_error: float = 0.0
    err_p90: float = 0.0              # 90th percentile |error|
    swing_rate: float = 0.0           # swings (zero crossings) per second
    swings: int = 0                   # half-oscillations (zero crossings with hysteresis)
    swing_period_s: float | None = None
    growth: float | None = None       # median ratio of successive swing sizes (>1 = growing)
    damping: float | None = None      # damping ratio from log decrement (None if not decaying)
    jitter: float = 0.0               # mean |2nd difference of steering| per sample
    d_share: float = 0.0              # |D| / (|P|+|D|) while error is small
    one_sided: float = 0.0            # fraction of time in long one-sided error stretches (normal in curves)
    falling_behind: float = 0.0       # ... of which the error kept GROWING (car losing the corner)
    saturated: float = 0.0            # fraction of time steering at its limit
    findings: list = field(default_factory=list)   # (severity, code, text)
    suggest: dict = field(default_factory=dict)    # {"kp": factor, "kd": factor, "speed": delta}

    @property
    def ok(self) -> bool:
        return not any(sev == "bad" for sev, _, _ in self.findings)

    @property
    def healthy(self) -> bool:
        return self.sufficient and not any(sev in ("bad", "warn") for sev, _, _ in self.findings)


def _swing_peaks(t, e, hyst):
    """Split the error into swings between zero crossings (with hysteresis) and
    return (time of each crossing, peak |error| of each swing)."""
    side, start, peak, cross_t, peaks = 0, 0, 0.0, [], []
    for i, v in enumerate(e):
        s = 1 if v > hyst else (-1 if v < -hyst else 0)
        if s == 0:
            peak = max(peak, abs(v))
            continue
        if side == 0:
            side, peak = s, abs(v)
        elif s != side:
            peaks.append(peak)
            cross_t.append(t[i])
            side, peak = s, abs(v)
        else:
            peak = max(peak, abs(v))
    return cross_t, peaks


def analyze(t, error, steering, p=None, d=None, tracking=None, max_steering=0.8,
            hyst=0.02, min_samples=40, min_tracked=0.85, sat_limit=0.20, weave_fast=0.9) -> Health:
    """min_tracked: below this tracked fraction = line failure. sat_limit: share of time at full
    steering that counts as too fast (curves legitimately sit near full lock, so the lap tuner
    passes a higher limit there). weave_fast: swings/s that count as a fast weave."""
    t = np.asarray(t, float)
    e = np.asarray(error, float)
    s = np.asarray(steering, float)
    h = Health(n=len(t))
    if len(t) < 2:
        return h
    h.seconds = float(t[-1] - t[0])
    trk = np.ones(len(t), bool) if tracking is None else np.asarray(tracking, bool)
    h.tracked = float(trk.mean())
    ok = trk & ~np.isnan(e) & ~np.isnan(s)
    if ok.sum() < min_samples:
        h.findings.append(("info", "few", "Not enough tracking data in this window to judge"))
        return h
    h.sufficient = True
    t, e, s = t[ok], e[ok], s[ok]
    h.mean_abs_error = float(np.mean(np.abs(e)))
    h.err_p90 = float(np.percentile(np.abs(e), 90))
    h.saturated = float(np.mean(np.abs(s) >= 0.95 * max_steering))

    # --- oscillation: swings, their period and whether they grow
    cross_t, peaks = _swing_peaks(t, e, hyst)
    h.swings = len(peaks)
    h.swing_rate = h.swings / max(h.seconds, 1e-6)
    if len(cross_t) >= 3:
        h.swing_period_s = float(2 * np.median(np.diff(cross_t)))
    big = [pk for pk in peaks if pk > 2 * hyst]
    if len(big) >= 3:
        ratios = [b / a for a, b in zip(big, big[1:]) if a > 0]
        h.growth = float(np.median(ratios))
        if h.growth < 0.95:
            delta = -math.log(max(h.growth, 1e-3))
            h.damping = float(delta / math.sqrt(4 * math.pi ** 2 + delta ** 2))

    # --- jitter: frame-to-frame chatter (2nd difference) and how much of it is D
    if len(s) >= 3:
        h.jitter = float(np.mean(np.abs(np.diff(s, 2))))
    if p is not None and d is not None:
        P = np.asarray(p, float)[ok]
        D = np.asarray(d, float)[ok]
        small = np.abs(e) < 0.05
        if small.sum() >= 10:
            h.d_share = float(np.mean(np.abs(D[small])) / (np.mean(np.abs(P[small])) + np.mean(np.abs(D[small])) + 1e-9))

    # --- one-sided stretches (curves): same sign, |e| > 0.08, longer than 1 s. A steady offset in a
    # curve is normal for P control; it's a problem when the error keeps GROWING through the stretch.
    long_t, grow_t, run_start, run_sign = 0.0, 0.0, None, 0

    def close(i0, i1):
        nonlocal long_t, grow_t
        dur = t[i1] - t[i0]
        if dur <= 1.0:
            return
        long_t += dur
        seg = np.abs(e[i0:i1 + 1])
        k = max(2, len(seg) // 4)
        if np.median(seg[-k:]) > np.median(seg[:k]) + 0.05:
            grow_t += dur
    for i in range(len(e)):
        sg = int(np.sign(e[i])) if abs(e[i]) > 0.08 else 0
        if sg != 0 and sg == run_sign:
            continue
        if run_start is not None:
            close(run_start, i - 1)
        run_start, run_sign = (i, sg) if sg != 0 else (None, 0)
    if run_start is not None:
        close(run_start, len(e) - 1)
    h.one_sided = float(long_t / max(h.seconds, 1e-6))
    h.falling_behind = float(grow_t / max(h.seconds, 1e-6))

    # --- verdicts (worst first) and what to change.
    # Thresholds calibrated by sweeping KP/KD/speed in tools/sim_track.py (see HANDOFF.md);
    # re-check them against real runs before trusting them blindly.
    f, sug = h.findings, h.suggest
    if h.tracked < min_tracked:
        f.append(("bad", "lost", f"Line lost {100 * (1 - h.tracked):.0f}% of the time: too fast, or the line leaves view"))
        sug["speed"] = -1
    if h.saturated > sat_limit:
        f.append(("bad", "too_fast", f"Steering at its limit {100 * h.saturated:.0f}% of the time: too fast for the corners"))
        sug["speed"] = -1
    elif h.falling_behind > 0.08 and h.saturated > min(0.10, sat_limit):
        f.append(("bad", "cant_turn", "Steering at its limit and still falling behind in turns: too fast for the corners"))
        sug["speed"] = -1
    if h.swing_rate > weave_fast:
        f.append(("bad", "weave", f"Weaving fast ({h.swing_rate:.1f} swings/s): KP too high"))
        sug["kp"] = 0.8
    elif h.swing_rate > 0.55 or (h.growth is not None and h.growth > 1.2 and h.swings >= 6 and h.swing_rate > 0.4):
        f.append(("warn", "weave", f"Weaving ({h.swing_rate:.1f} swings/s): lower KP a little"))
        sug["kp"] = 0.9
    elif h.err_p90 > 0.22 and h.saturated < 0.10:
        f.append(("warn", "lagging", f"Large errors (90% under {h.err_p90:.2f}) without using full steering: KP too low"))
        sug["kp"] = 1.25
    elif h.falling_behind > 0.08:
        f.append(("warn", "lagging", "Falls further behind through turns without using full steering: KP too low"))
        sug["kp"] = 1.2
    if h.jitter > 0.09 and h.d_share > 0.6:
        f.append(("warn", "jitter", "Steering chatters frame to frame and D dominates: KD too high (amplifies camera noise)"))
        sug["kd"] = 0.7
    if not f and 0.10 < h.saturated <= 0.20:
        f.append(("info", "near_limit", f"Steering at its limit {100 * h.saturated:.0f}% of the time: little margin left"))
    return h


def summary(h: Health) -> str:
    bits = [f"{h.seconds:.0f}s", f"tracked {100 * h.tracked:.0f}%", f"|err| p90 {h.err_p90:.2f}",
            f"{h.swing_rate:.2f} swings/s"]
    if h.growth is not None:
        bits.append(f"growth x{h.growth:.2f}")
    if h.damping is not None:
        bits.append(f"damping {h.damping:.2f}")
    bits += [f"jitter {h.jitter:.3f}", f"sat {100 * h.saturated:.0f}%", f"behind {100 * h.falling_behind:.0f}%"]
    verdict = "; ".join(txt for _, _, txt in h.findings) or "healthy"
    return " · ".join(bits) + " -> " + verdict
