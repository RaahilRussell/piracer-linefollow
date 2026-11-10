"""One shared way to drive the simulated car through the REAL detector + pilot.

Used by tools/sim_limits.py, tools/sim_shootout.py, tools/calibrate_sim.py and tools/race_eval.py so
they all build the pilot exactly like pi/drive_line.py does (same line_config, same car model).
Rendered camera frames -> LineDetectorPart -> LineFollowerPilot with a speed plan -> simulated car.

A "setup" is a dict of Config overrides, plus optional:
    "pilot": {"coast_s": .., "slow_s": ..}        pilot constructor extras
    "plan":  {"transition": .., "ramp_down": .., "ramp_up": ..}   speed plan extras

Everything here is SIMULATED: physics come from sim/profile.json (several values assumed).
"""
from __future__ import annotations

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (os.path.join(ROOT, "pi"), ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import line_config  # noqa: E402
from donkey_part import LineDetectorPart, LineFollowerPilot, SpeedPlan, car_model, load_line_config  # noqa: E402
from sim.params import Profile  # noqa: E402
from sim.world import World  # noqa: E402

# Physical values the simulator only assumes. Each variation changes one of them inside its
# plausible range; a setup is robust when it stays on the track through all of them.
VARIATIONS = {
    "nominal": {},
    "lock 16 deg": {"steer_angle_table": [[300, 16.0], [375, 0.0], [450, -16.0]]},
    "lock 20 deg": {"steer_angle_table": [[300, 20.0], [375, 0.0], [450, -20.0]]},
    "grip 0.40": {"mu_lat": 0.40},
    "grip 0.70": {"mu_lat": 0.70},
    "delay +40 ms": {"actuator_delay_s": 0.06},
    "slow camera pipe": {"processing_s": 0.051},
    "speed -15%": {"speed_table": [[370, 0.0], [387, 0.0], [390, 0.30], [407, 1.53]]},
    "speed +15%": {"speed_table": [[370, 0.0], [387, 0.0], [390, 0.40], [407, 2.07]]},
    "slow servo": {"servo_rate_dps": 150.0, "servo_tau_s": 0.08},
    "camera pitch 6": {"cam_pitch_deg": 6.0},
    "camera pitch 10": {"cam_pitch_deg": 10.0},
    "camera 13 cm high": {"cam_height_m": 0.13},
    "camera 17 cm high": {"cam_height_m": 0.17},
    "noisy, 5% dropout": {"meas_noise": 0.012, "meas_dropout": 0.05},
}


def build(setup=None, straight=None, corner=None):
    """(cfg, pilot, detector) built like pi/drive_line.py's build_parts(), with setup overrides."""
    setup = setup or {}
    cfg, ex = load_line_config(line_config)
    for k, v in setup.items():
        if k not in ("pilot", "plan"):
            if not hasattr(cfg, k):
                raise KeyError(f"unknown setting '{k}'")
            setattr(cfg, k, v)
    cfg.max_throttle = 1.0
    extras = {"coast_s": ex["LOST_COAST_S"], "slow_s": ex["LOST_SLOW_S"], **setup.get("pilot", {})}
    model = car_model(ex)
    pilot = LineFollowerPilot(cfg, single_line_stop_s=0.0, model=model, **extras)
    st = ex["SPEED_STRAIGHT"] if straight is None else straight
    co = ex["SPEED_CORNER"] if corner is None else corner
    plan = {"transition": ex["SPEED_TRANSITION"] if straight is None else None,
            "ramp_down": ex["SPEED_RAMP_DOWN"], "ramp_up": ex["SPEED_RAMP_UP"], **setup.get("plan", {})}
    pilot.speed_plan = SpeedPlan(st, co, **plan)
    det = LineDetectorPart(cfg, show_overlay=False, model=model)
    det.attach(pilot)
    return cfg, pilot, det


# The race punishes leaving the track; riding over the inside line is accepted (user's call, Nov 10).
# So the race tools fail a run for running WIDE or for going this far past the inside tape's outer
# edge (0.10 m = the car's centre is past the tape), and report shallower inside cuts separately.
INSIDE_TOLERANCE_M = 0.10


def drive(setup=None, straight=None, corner=None, seconds=30.0, seed=0, prof=None, vary=None, trace=False,
          inside_tolerance_m=INSIDE_TOLERANCE_M):
    """Drive one simulated run. Returns a dict of results (and the per-tick trace if asked)."""
    prof = prof or Profile.load()
    if vary:
        prof = prof.with_values(**vary)
    cfg, pilot, det = build(setup, straight, corner)
    w = World(prof, vision="rendered", seed=seed, target_ratio=cfg.target_x_ratio, row_ratio=cfg.lookahead_ratio,
              inside_tolerance_m=inside_tolerance_m)
    pilot.clock = det.clock = lambda: w.t
    hist, where = [], {"curve": 0, "curve exit": 0, "straight": 0, "unknown": 0}
    curve_steer, steer, rows = [], [], []
    lost = n = seen = 0
    while w.t < seconds:
        _, _, _, _, motion_proxy, img = w.observe()
        if img is None:
            w.tick(0.0, 0.0)
            continue
        x, found, age, _, _, both = det.run(img)
        angle, thr = pilot.run("local", x, found, age, motion_proxy, both)
        angle, thr = angle or 0.0, thr or 0.0
        n += 1
        lost += not found
        cmd = pilot.last_cmd
        sec = cmd.section if cmd is not None else "unknown"
        hist.append(sec)
        steer.append(angle)
        if found and sec == "corner":
            curve_steer.append(angle)
        if trace:
            _, s_now = w.track.nearest(w.veh.s.x, w.veh.s.y)
            rows.append({"t": w.t, "s": s_now, "section": sec, "found": bool(found), "angle": angle, "thr": thr,
                         "v": w.veh.s.v, "curv": pilot.curvature, "status": cmd.status if cmd else "LOST",
                         "offset": w.track.offset_from_left(w.veh.s.x, w.veh.s.y)})
        w.tick(angle, thr)
        if w.metrics.off_track > seen:
            seen = w.metrics.off_track
            recent = hist[-30:]
            key = ("curve" if recent[-1] == "corner" else "curve exit" if "corner" in recent
                   else "straight" if recent[-1] == "straight" else "unknown")
            where[key] += 1
            pilot.ctrl.reset()
    s = w.metrics.summary()
    cs = np.abs(np.array(curve_steer)) if curve_steer else np.zeros(1)
    a = np.array(steer) if steer else np.zeros(2)
    out = {"off": s["off_track"], "wide": s["off_wide"], "inside": s["off_inside"], "inside_pct": s["inside_pct"],
           "where": where, "lap": s["lap_s"], "best_lap": s["best_lap_s"], "laps": s["laps"],
           "lap_times": list(w.metrics.lap_times), "lost": 100.0 * lost / max(n, 1),
           "err_cm": s["err_mean_cm"], "err_p95_cm": s["err_p95_cm"],
           "clear_left_cm": s["min_body_clear_left_cm"], "clear_right_cm": s["min_body_clear_right_cm"],
           "lock": 100.0 * float(np.mean(cs >= 0.95 * cfg.max_steering)), "steer": float(np.mean(cs)),
           "steer_jump": float(np.percentile(np.abs(np.diff(a)), 99)) if len(a) > 2 else 0.0,
           "slip": s["slip_pct"], "speed": s["mean_speed"],
           "side": "left" if curve_steer and np.mean(curve_steer) < 0 else "right" if curve_steer else "-"}
    if trace:
        out["trace"] = rows
    return out


def _job(args):
    setup, straight, corner, seconds, seed, vary = args
    return drive(setup, straight, corner, seconds, seed, vary=vary)


def sweep(setup, straight, corner, seconds=30.0, seeds=2, variations=None, pool=None):
    """Run a setup over seeds x physics variations. Returns {variation: aggregate}."""
    names = list(variations) if variations is not None else ["nominal"]
    jobs = [(setup, straight, corner, seconds, sd, VARIATIONS[v]) for v in names for sd in range(seeds)]
    res = pool.map(_job, jobs) if pool is not None else [_job(j) for j in jobs]
    out = {}
    for i, v in enumerate(names):
        rs = res[i * seeds:(i + 1) * seeds]
        laps = [r["lap"] for r in rs if r["lap"]]
        out[v] = {"off": sum(r["off"] for r in rs), "wide": sum(r["wide"] for r in rs),
                  "inside": sum(r["inside"] for r in rs), "inside_pct": float(np.mean([r["inside_pct"] for r in rs])),
                  "lost": float(np.mean([r["lost"] for r in rs])),
                  "lap": float(np.mean(laps)) if laps else float("nan"),
                  "err_cm": float(np.mean([r["err_cm"] for r in rs])),
                  "clear_left_cm": min(r["clear_left_cm"] for r in rs), "clear_right_cm": min(r["clear_right_cm"] for r in rs),
                  "lock": float(np.mean([r["lock"] for r in rs])), "steer": float(np.mean([r["steer"] for r in rs])),
                  "steer_jump": float(np.max([r["steer_jump"] for r in rs])),
                  "where": {k: sum(r["where"][k] for r in rs) for k in rs[0]["where"]}, "side": rs[0]["side"]}
    return out


def total(result):
    """Collapse a sweep() result: total off-tracks, worst lost %, nominal lap, smallest clearance."""
    vals = list(result.values())
    nominal = result.get("nominal", vals[0])
    return {"off": sum(v["off"] for v in vals), "wide": sum(v["wide"] for v in vals),
            "inside": sum(v["inside"] for v in vals), "inside_pct": max(v["inside_pct"] for v in vals),
            "lost": max(v["lost"] for v in vals), "lap": nominal["lap"],
            "worst_lap": max([v["lap"] for v in vals if v["lap"] == v["lap"]], default=float("nan")),
            "clear_cm": min(min(v["clear_left_cm"], v["clear_right_cm"]) for v in vals),
            "failed": [k for k, v in result.items() if v["off"]]}
