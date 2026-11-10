#!/usr/bin/env python3
"""Compare whole pilot setups in the simulator through the REAL detector + pilot (same path as
pi/sim_dashboard.py --sim), using the shared driver in tools/simdrive.py. Reports off-track events, where they happened (curve, curve exit,
straight), lateral error and lap time. Simulated physics use sim/profile.json (19 values assumed).

    python3 tools/sim_shootout.py [--seconds 40] [--seeds 2]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "pi"), ROOT]

sys.path.insert(0, os.path.join(ROOT, "tools"))
import simdrive  # noqa: E402
from sim.params import Profile  # noqa: E402

PD = {"pure_pursuit": False, "target_x_ratio": 0.40, "steering_smoothing": 0.35, "ff_gain": 0.0, "corner_exit_steer": 0.0}


def variants():
    return {
        "race default (pure pursuit)": {},
        "pursuit + exit hold": {"corner_exit_steer": 0.45},
        "pursuit, fixed lookahead .75": {"pursuit_ld_m": 0.75, "pursuit_ld_gain_s": 0.0},
        "PD fallback (KP 3.5)": PD,
        "PD + exit hold": {**PD, "corner_exit_steer": 0.45},
        "PD + feedforward 0.6": {**PD, "ff_gain": 0.6},
        "PD, steering capped 0.8": {**PD, "max_steering": 0.8},
    }


def run(setup, straight, corner, seconds, seed, prof):
    r = simdrive.drive(setup, straight, corner, seconds, seed, prof)
    return {"off_track": r["off"], "err_mean_cm": r["err_cm"], "lap_s": r["lap"]}, r["where"], r["lost"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--speeds", default="0.70/0.65,0.90/0.85,1.00/0.92")
    ap.add_argument("--radius", type=float, help="Override the (assumed) curve radius of the simulated track, m")
    ap.add_argument("--straight", type=float, help="Override the (assumed) straight length, m")
    a = ap.parse_args()
    prof = Profile.load()
    if a.radius or a.straight:
        prof._e.pop("track_polyline", None)      # an explicit stadium replaces the traced track
    if a.radius:
        prof._e["track_radius_m"]["value"] = a.radius
    if a.straight:
        prof._e["track_straight_m"]["value"] = a.straight
    speeds = [tuple(float(x) for x in p.split("/")) for p in a.speeds.split(",")]
    print(f"Track radius {prof._e['track_radius_m']['value']} m, straight {prof._e['track_straight_m']['value']} m. "
          f"Simulator: {len(prof.assumed_in_use())} assumed physical values. {a.seconds:.0f} s x {a.seeds} seeds each.")
    for st, co in speeds:
        print(f"\nstraight {st:.2f} / curve {co:.2f}")
        print(f"{'setup':28s} {'off':>4s} {'curve':>6s} {'exit':>5s} {'strt':>5s} {'lost%':>6s} {'err cm':>7s} {'lap s':>6s}")
        for name, setup in variants().items():
            offs, wh, lost, err, laps = 0, {"curve": 0, "curve exit": 0, "straight": 0, "unknown": 0}, [], [], []
            for seed in range(a.seeds):
                s, where, lp = run(setup, st, co, a.seconds, seed, prof)
                offs += s["off_track"]
                for k in wh:
                    wh[k] += where[k]
                lost.append(lp)
                err.append(s["err_mean_cm"])
                if s.get("lap_s"):
                    laps.append(s["lap_s"])
            print(f"{name:28s} {offs:4d} {wh['curve']:6d} {wh['curve exit']:5d} {wh['straight']:5d} "
                  f"{np.mean(lost):6.1f} {np.mean(err):7.1f} {np.mean(laps) if laps else float('nan'):6.2f}")


if __name__ == "__main__":
    main()
