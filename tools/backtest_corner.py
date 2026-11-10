"""Backtest corner geometry (curvature feedforward + corner speed planning) in the simulator.

For each speed profile, compare controller variants on the nominal model AND on Monte-Carlo
variations of every unmeasured parameter (the pilot always uses the NOMINAL car model, like the
real car will until it's measured):

  python3 tools/backtest_corner.py                 # all speeds, default variants
  python3 tools/backtest_corner.py --draws 10 --seconds 25

Metrics per variant: off-track events (nominal / across draws), mean |lateral error| (cm),
mean lap time (s), steering pinned at its limit (%).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "pi"), ROOT]

from sim.__main__ import run_one  # noqa: E402
from sim.controllers import PD_STEERING  # noqa: E402
from sim.params import Profile  # noqa: E402
from sim.tune import monte_carlo_profiles  # noqa: E402

with open(os.path.join(ROOT, "pi", "gain_schedule.json")) as _f:
    SCHEDULE = json.load(_f)["profiles"]

VARIANTS = {
    "PD (schedule)": lambda kp, kd: {"kp": kp, "kd": kd},
    "PD + ff 0.6": lambda kp, kd: {"kp": kp, "kd": kd, "ff_gain": 0.6},
    "PD + ff 1.0": lambda kp, kd: {"kp": kp, "kd": kd, "ff_gain": 1.0},
    "PD + ff 0.6 + corner (mu .4)": lambda kp, kd: {"kp": kp, "kd": kd, "ff_gain": 0.6, "corner_plan": True, "grip_mu": 0.4},
}


def run(prof, speed, ov, seconds, seed):
    w, _ = run_one(prof, seconds=seconds, speed=speed, overrides={**PD_STEERING, **ov}, seed=seed)   # PD vs PD + feedforward
    return w.metrics.summary()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--draws", type=int, default=10)
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--profiles", default="")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    prof = Profile.load()
    draws = monte_carlo_profiles(prof, n=a.draws, seed=7)
    rows = []
    for p in SCHEDULE:
        if a.profiles and p["profile"] not in a.profiles.split(","):
            continue
        speed = (p["straight_cmd"], p["corner_cmd"]) if p["corner_cmd"] < p["straight_cmd"] else p["straight_cmd"]
        print(f"\n== {p['profile']} (throttle {p['straight_cmd']}, ~{p['straight_mps']} m/s), schedule KP {p['kp']} KD {p['kd']}")
        print(f"{'variant':<30}{'off nominal':>12}{'off in draws':>14}{'draws w/ off':>14}{'err cm':>8}{'lap s':>8}{'sat %':>7}")
        for name, mk in VARIANTS.items():
            ov = mk(p["kp"], p["kd"])
            nom = run(prof, speed, ov, a.seconds, 3)
            ds = [run(q, speed, ov, a.seconds, 3) for q in draws]
            off_d = sum(d["off_track"] for d in ds)
            bad_d = sum(d["off_track"] > 0 for d in ds)
            laps = [x["lap_s"] for x in [nom] + ds if x["lap_s"]]
            err = np.mean([x["err_mean_cm"] for x in [nom] + ds])
            sat = np.mean([x["steer_sat_pct"] for x in [nom] + ds])
            print(f"{name:<30}{nom['off_track']:>12}{off_d:>14}{bad_d:>11}/{len(ds):<2}{err:>8.1f}{(np.mean(laps) if laps else float('nan')):>8.2f}{sat:>7.1f}")
            rows.append({"profile": p["profile"], "variant": name, "off_nominal": nom["off_track"], "off_draws": off_d,
                         "draws_with_off": bad_d, "draws": len(ds), "err_cm": round(float(err), 2),
                         "lap_s": round(float(np.mean(laps)), 2) if laps else None, "sat_pct": round(float(sat), 1),
                         "nominal_lap_s": nom["lap_s"]})
    if a.json:
        json.dump(rows, open(a.json, "w"), indent=2)


if __name__ == "__main__":
    main()
