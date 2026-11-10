"""Simulator answers for the dashboard, fast enough to call on demand.

  speed_info(cmd)            m/s for a throttle command, corner speed limit, corner checks
  evaluate(cmd, kp, kd)      what these gains do at this speed: stability margins, an offset-step
                             manoeuvre and a simulated lap, with traces for charts
  tuning_map(cmd)            KP x KD grid: unstable / stable / meets every margin
  recommend(cmd)             best starting KP/KD for one speed: linear screen -> simulated laps ->
                             robustness over the unmeasured parameters (same method as sim/tune.py)

`over` (optional) carries the dashboard's live steering settings (target, lookahead, smoothing,
max steering, rate limit) so answers match what the car is set to now.

Everything comes from sim/profile.json. Values marked "assumed" there are placeholders until the
car is measured (sim/CALIBRATION.md); each answer lists them so nobody mistakes it for the car.
"""
from __future__ import annotations

import copy
import functools
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (os.path.join(ROOT, "pi"), ROOT):
    if _p not in sys.path:
        sys.path.append(_p)

import line_config  # noqa: E402
from donkey_part import load_line_config  # noqa: E402

from .actuators import ThrottleMap  # noqa: E402
from .camera import Camera, target_offset  # noqa: E402
from .controllers import PD_STEERING, PilotController  # noqa: E402
from .feasibility import corner_report  # noqa: E402
from .params import Profile  # noqa: E402
from .track import Track  # noqa: E402
from .tune import KD_GRID, REQ, LinearLoop, cmd_for_speed, monte_carlo_profiles  # noqa: E402
from .world import World  # noqa: E402

TUNABLE = ("target_x_ratio", "lookahead_ratio", "steering_smoothing", "max_steering", "steering_rate_limit",
           "ff_gain", "corner_plan", "corner_margin")
KP_MAP = np.round(np.arange(0.5, 10.01, 0.5), 2)


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def _num(x):
    """JSON-safe copy (numpy scalars/arrays, inf/nan -> None)."""
    if isinstance(x, dict):
        return {k: _num(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_num(v) for v in x]
    if isinstance(x, np.ndarray):
        return _num(x.tolist())
    if isinstance(x, (np.floating, float)):
        x = float(x)
        return x if math.isfinite(x) else None
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


@functools.lru_cache(maxsize=1)
def _base():
    prof = Profile.load()
    cfg, _ = load_line_config(line_config)
    return prof, cfg, ThrottleMap(prof), Track.stadium(prof)


def reload():
    """Forget the cached profile (after sim/profile.json changes)."""
    _base.cache_clear()
    _map_cached.cache_clear()


def _clean_over(over):
    return {k: float(over[k]) for k in TUNABLE if over and over.get(k) is not None}


def _cfg(over):
    _, cfg, _, _ = _base()
    c = copy.copy(cfg)
    for k, v in _clean_over(over).items():
        setattr(c, k, v)
    return c


def assumed():
    return _base()[0].assumed_in_use()


def speed_of(cmd: float) -> float:
    _, _, tm, _ = _base()
    return float(tm.steady_speed(tm.pwm(cmd), True))


def speed_info(cmd: float, over=None) -> dict:
    prof, _, tm, track = _base()
    c = _cfg(over)
    v = speed_of(cmd)
    ride = target_offset(Camera(prof), None, c.target_x_ratio, c.lookahead_ratio, prof.tape_width_m)
    feas, _ = corner_report(prof, track, ride, v)
    vmax = min(f["v_max_mps"] for f in feas) if feas else v
    corner_cmd = cmd if v <= 0.8 * vmax else cmd_for_speed(tm, 0.8 * vmax)
    return _num({"cmd": round(float(cmd), 2), "pwm": tm.pwm(cmd), "mps": round(v, 2),
                 "corner_mps_limit": round(vmax, 2), "corner_cmd": corner_cmd,
                 "moves": v > 0.02, "corners": feas})


def _drive(prof, speed, kp, kd, over, seconds, seed=3, offset_m=None, every=2):
    """Drive the simulated car; return (world, trace, path)."""
    clk = _Clock()
    ov = {**PD_STEERING, "kp": float(kp), "kd": float(kd)}      # the advisor judges KP/KD: PD steering
    ov.update(_clean_over(over))
    ctrl = PilotController("dashboard", ov, speed=speed, clock=clk)
    w = World(prof, seed=seed, target_ratio=ctrl.cfg.target_x_ratio, row_ratio=ctrl.cfg.lookahead_ratio)
    w.want_edges = ctrl.needs_edges
    if offset_m:
        w.veh.s.y += offset_m                 # shifted toward the left line
    tr = {"t": [], "err_cm": [], "steer": [], "mps": []}
    path, i = [], 0
    while w.t < seconds:
        clk.t = w.t
        x, found, both, age, motion, img = w.observe()
        a, th, _ = ctrl.step(x, found, both, age, motion, img, edges=w.edges, source=w.edges_source)
        w.tick(a, th)
        s = w.veh.s
        if i % every == 0:
            off = w.track.offset_from_left(s.x, s.y) - w._ride
            tr["t"].append(round(w.t, 2))
            tr["err_cm"].append(round(100 * off, 2))
            tr["steer"].append(round(float(a), 3))
            tr["mps"].append(round(float(s.v), 2))
            path.append([round(float(s.x), 3), round(float(s.y), 3)])
        i += 1
        if offset_m and w.metrics.off_track:
            break
    return w, tr, path


def _step(prof, speed, kp, kd, over, offset_m=0.06, seconds=6.0):
    """Start 6 cm off the ride line on a long straight; measure overshoot and settling."""
    q = prof.with_values(track_straight_m=max(prof.track_straight_m, 12.0))
    w, tr, _ = _drive(q, speed, kp, kd, over, seconds, seed=5, offset_m=offset_m, every=1)
    if w.metrics.off_track:
        return {"overshoot_pct": 200.0, "settle_s": seconds, "off_track": True, "trace": tr}
    e = np.array(tr["err_cm"]) / 100.0
    first = e[0] if abs(e[0]) > 1e-3 else offset_m
    over_pct = max(0.0, float(np.max(-np.sign(first) * e)) / abs(first) * 100)
    band = 0.15 * abs(first)
    outside = np.nonzero(np.abs(e) > band)[0]
    settle = tr["t"][outside[-1]] if len(outside) else 0.0
    return {"overshoot_pct": round(over_pct, 1), "settle_s": round(float(settle), 2), "off_track": False, "trace": tr}


def _verdict(lin, step, lap, info):
    out = []

    def add(level, text):
        out.append({"level": level, "text": text})
    if not info["moves"]:
        add("bad", f"Throttle {info['cmd']:.2f} doesn't move the car in the model (PWM {info['pwm']}). Pick a higher speed.")
        return out
    if not lin.get("stable"):
        add("bad", "Unstable at this speed: the car will weave harder and harder until it loses the line. Lower KP.")
    else:
        pm, gm, dm, z = lin.get("pm_deg"), lin.get("gm_db"), lin.get("delay_margin_ms"), lin.get("zeta")
        if pm is not None and pm < REQ["pm_deg"]:
            add("warn", f"Phase margin {pm:.0f}° (want ≥ {REQ['pm_deg']:.0f}°): it will overshoot and ring. Lower KP or raise KD.")
        if z is not None and z < REQ["zeta"]:
            add("warn", f"Lightly damped (ζ {z:.2f}, want ≥ {REQ['zeta']}): swings die out slowly. Raise KD a little.")
        if gm is not None and gm < REQ["gm_db"]:
            add("warn", f"Gain margin {gm:.1f} dB (want ≥ {REQ['gm_db']:.0f}): little room before it goes unstable.")
        if dm is not None and dm < REQ["delay_margin_ms"]:
            add("warn", f"Only {dm:.0f} ms of extra delay tolerated: one slow camera frame could start a weave.")
    if lap["off_track"]:
        add("bad", f"Left the track {lap['off_track']}× in {lap['seconds']:.0f} simulated seconds.")
    clearance = min(lap.get("min_body_clear_left_cm") or 0.0, lap.get("min_body_clear_right_cm") or 0.0)
    if clearance < 2.0:
        add("warn", f"Chassis clearance falls to {clearance:.1f} cm from a tape's outer edge; keep at least 2 cm of modelled body margin.")
    if step["overshoot_pct"] > 35 and not step["off_track"]:
        add("warn", f"Overshoots by {step['overshoot_pct']:.0f}% after a 6 cm nudge.")
    if step["settle_s"] > 3.5 and not step["off_track"]:
        add("warn", f"Slow to settle ({step['settle_s']:.1f} s after a 6 cm nudge).")
    if (lap.get("steer_sat_pct") or 0) > 20:
        add("warn", f"Steering pinned at its limit {lap['steer_sat_pct']:.0f}% of the time: too fast for the corners.")
    if not out:
        add("ok", f"Stable with margin (phase {lin.get('pm_deg', 0):.0f}°, damping {lin.get('zeta', 0):.2f}); "
                  f"settles in {step['settle_s']:.1f} s; average error {lap['err_mean_cm']:.1f} cm.")
    return out


def evaluate(cmd: float, kp: float, kd: float, over=None, corner_cmd=None) -> dict:
    t0 = time.time()
    prof, _, _, track = _base()
    info = speed_info(cmd, over)
    speed = (cmd, corner_cmd) if corner_cmd is not None and corner_cmd < cmd else cmd
    v = info["mps"]
    lin = LinearLoop(prof, max(v, 0.1), _cfg(over)).analyze(kp, kd)
    step = _step(prof, speed, kp, kd, over)
    secs = float(min(60.0, max(15.0, 2.3 * track.length / max(v, 0.05))))
    w, lap_tr, path = _drive(prof, speed, kp, kd, over, secs, seed=3, every=2)
    lap = w.metrics.summary()
    k = max(1, len(track.left) // 160)
    return _num({
        "cmd": cmd, "kp": kp, "kd": kd, "speed": info, "linear": lin, "requirements": REQ,
        "step": step, "lap": lap, "lap_trace": lap_tr, "path": path[::2],
        "track": {"left": track.left[::k].round(3), "right": track.right[::k].round(3)},
        "verdict": _verdict(lin, step, lap, info), "assumed": assumed(), "seconds": round(time.time() - t0, 2),
    })


@functools.lru_cache(maxsize=32)
def _map_cached(cmd, over_items):
    prof = _base()[0]
    over = dict(over_items)
    v = speed_of(cmd)
    lin = LinearLoop(prof, max(v, 0.1), _cfg(over))
    cells = []
    for kp in KP_MAP:
        for kd in KD_GRID:
            a = lin.analyze(float(kp), float(kd))
            if not a["stable"]:
                cls = 0
            else:
                ok = all((a.get(k) or 0) >= REQ[k] for k in REQ)
                cls = 2 if ok else 1
            cells.append([float(kp), float(kd), cls, a.get("pm_deg")])
    return {"cmd": cmd, "mps": round(v, 2), "kp": KP_MAP, "kd": KD_GRID, "cells": cells, "requirements": REQ}


def tuning_map(cmd: float, over=None) -> dict:
    t0 = time.time()
    out = dict(_map_cached(round(float(cmd), 2), tuple(sorted(_clean_over(over).items()))))
    out["seconds"] = round(time.time() - t0, 2)
    return _num(out)


def _cost(prof, speed, kp, kd, over):
    w, _, _ = _drive(prof, speed, kp, kd, over, 25.0, seed=0, every=50)
    m = w.metrics.summary()
    st = _step(prof, speed, kp, kd, over)
    clearance = min(m.get("min_body_clear_left_cm") or 0.0, m.get("min_body_clear_right_cm") or 0.0)
    cost = (m["err_mean_cm"] + 0.5 * m["err_p95_cm"] + 40.0 * m["off_track"] + 0.4 * m["slip_pct"]
            + 0.3 * m["steer_sat_pct"] + 0.2 * m["lost_pct"] + 0.15 * st["overshoot_pct"] + 2.0 * st["settle_s"]
            + 2.0 * max(0.0, 2.0 - clearance))
    return cost, m, st


def recommend(cmd: float, over=None, mc: int = 6) -> dict:
    """Best starting KP/KD for one speed (the sim/tune.py method, sized for an on-demand answer)."""
    t0 = time.time()
    prof, _, tm, _ = _base()
    info = speed_info(cmd, over)
    if not info["moves"]:
        return _num({"cmd": cmd, "speed": info, "error": "This throttle doesn't move the car in the model."})
    v, corner_cmd = info["mps"], info["corner_cmd"]
    speed = (cmd, corner_cmd) if corner_cmd < cmd else cmd
    v_c = speed_of(corner_cmd) if corner_cmd < cmd else v
    c = _cfg(over)
    lin, lin_c = LinearLoop(prof, max(v, 0.1), c), LinearLoop(prof, max(v_c, 0.1), c)
    cands = []
    for kp in KP_MAP:
        for kd in KD_GRID:
            a1 = lin.analyze(float(kp), float(kd))
            if not a1["stable"]:
                continue
            a2 = lin_c.analyze(float(kp), float(kd)) if lin_c.v != lin.v else a1
            if not a2["stable"]:
                continue
            worst = {k: min(a1.get(k) or 0, a2.get(k) or 0) for k in REQ}
            cands.append((float(kp), float(kd), all(worst[k] >= REQ[k] for k in REQ), worst, a1.get("crossover_hz") or 0))
    if not cands:
        return _num({"cmd": cmd, "speed": info, "error": "No stable KP/KD at this speed in the model: slow down."})
    passing = [x for x in cands if x[2]]
    pool = passing or sorted(cands, key=lambda x: -x[3]["pm_deg"])[:10]
    pool = sorted(pool, key=lambda x: -x[4])[:10]
    scored = sorted(((*_cost(prof, speed, kp, kd, over), kp, kd, ok, worst) for kp, kd, ok, worst, _ in pool),
                    key=lambda s: s[0])
    draws = monte_carlo_profiles(prof, n=mc)
    chosen = None
    for cost, m, st, kp, kd, ok, worst in scored[:4]:
        fails, off, clearance_failures, min_clearance = 0, 0, 0, float("inf")
        for q in draws:
            if not LinearLoop(q, max(v_c, 0.1), c).analyze(kp, kd)["stable"]:
                fails += 1
        for q in draws[:2]:
            w, _, _ = _drive(q, speed, kp, kd, over, 20.0, seed=3, every=50)
            off += w.metrics.off_track
            m_draw = w.metrics.summary()
            clear = min(m_draw.get("min_body_clear_left_cm") or 0.0, m_draw.get("min_body_clear_right_cm") or 0.0)
            min_clearance = min(min_clearance, clear)
            clearance_failures += int(clear < 2.0)
        rob = {"draws": len(draws), "unstable": fails, "off_track_in_2_runs": off,
               "clearance_failures_in_2_runs": clearance_failures,
               "min_body_clearance_cm": round(min_clearance, 1) if math.isfinite(min_clearance) else None}
        entry = dict(kp=kp, kd=kd, meets_margins=ok, margins=worst, lap=m, step={k: st[k] for k in ("overshoot_pct", "settle_s")},
                     robustness=rob, robust=(fails == 0 and off == 0 and clearance_failures == 0))
        if entry["robust"]:
            chosen = entry
            break
        if chosen is None or (fails, off, clearance_failures) < (chosen["robustness"]["unstable"], chosen["robustness"]["off_track_in_2_runs"], chosen["robustness"]["clearance_failures_in_2_runs"]):
            chosen = entry
    chosen.update(cmd=cmd, corner_cmd=corner_cmd, speed=info, candidates_passing=len(passing),
                  assumed=assumed(), seconds=round(time.time() - t0, 1))
    return _num(chosen)
