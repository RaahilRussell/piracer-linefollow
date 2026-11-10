#!/usr/bin/env python3
"""Find the simulator's speed limits the way the lap tuner should:

  1. curve speed: run the whole lap at one speed and raise it until full steering lock can no longer
     hold the curve (off-track or line lost). Reports how much of each curve is spent at full lock.
  2. straight speed: hold curves at the best curve speed and raise straight speed until braking into
     the curve fails.

Goes through the real detector + pilot (tools/simdrive.py). Off track = running wide, or more than
10 cm past the inside tape; riding over the inside line is accepted. For the full robustness check
across the assumed physical values use tools/race_eval.py. Physics are sim/profile.json
(several values assumed), so treat the numbers as a starting point for the car's lap tuner.

    python3 tools/sim_limits.py [--seconds 25] [--seeds 2]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "tools"), os.path.join(ROOT, "pi"), ROOT]

import simdrive  # noqa: E402
from sim.params import Profile  # noqa: E402

# PD steering as driven on the real car (Nov 9): the base the full-lock angle was fitted with.
BASE = {"pure_pursuit": False, "kp": 3.5, "kd": 0.4, "straight_kp": 3.5, "straight_kd": 0.4, "corner_kp": 3.5,
        "corner_kd": 0.4, "target_x_ratio": 0.40, "steering_smoothing": 0.35, "max_steering": 1.0,
        "corner_exit_steer": 0.0, "pilot": {"coast_s": 0.5, "slow_s": 0.5}}
# The race defaults in pi/line_config.py (pure pursuit), unchanged.
RACE = {}


def drive(setup, straight, corner, seconds, seed, prof, ramp_down=2.0):
    """One simulated run through the shared driver (tools/simdrive.py)."""
    setup = {**setup, "plan": {**setup.get("plan", {}), "ramp_down": ramp_down}}
    return simdrive.drive(setup, straight, corner, seconds, seed, prof)


def agg(setup, st, co, seconds, seeds, prof):
    rs = [drive(setup, st, co, seconds, sd, prof) for sd in range(seeds)]
    laps = [r["lap"] for r in rs if r["lap"]]
    return {"off": sum(r["off"] for r in rs), "lost": np.mean([r["lost"] for r in rs]),
            "lock": np.mean([r["lock"] for r in rs]), "steer": np.mean([r["steer"] for r in rs]),
            "lap": np.mean(laps) if laps else float("nan"), "side": rs[0]["side"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--ff", default="pursuit,0,0.4", help="'pursuit' = the race defaults; a number = PD steering with that feedforward gain")
    a = ap.parse_args()
    prof = Profile.load()
    print(f"Sim track: curve radius {prof._e['track_radius_m']['value']} m, straight {prof._e['track_straight_m']['value']} m "
          f"({len(prof.assumed_in_use())} assumed physical values)")
    best = {}
    for ff in a.ff.split(","):
        setup = RACE if ff == "pursuit" else {**BASE, "ff_gain": float(ff)}
        ff = "pure pursuit" if ff == "pursuit" else f"PD + feedforward {ff}"
        print(f"\n== {ff}: 1) curve speed (whole lap at one speed)")
        print(f"{'speed':>6s} {'off':>4s} {'lost%':>6s} {'full lock% in curves':>21s} {'|steer|':>8s} {'lap s':>6s}")
        top = None
        for sp in np.round(np.arange(0.60, 1.001, 0.05), 2):
            r = agg(setup, sp, sp, a.seconds, a.seeds, prof)
            print(f"{sp:6.2f} {r['off']:4d} {r['lost']:6.1f} {r['lock']:16.0f} ({r['side']}) {r['steer']:8.2f} {r['lap']:6.2f}", flush=True)
            if r["off"] == 0 and r["lost"] < 2.0:
                top = sp
            elif top is not None:
                break
        if top is None:
            print("   no clean curve speed found")
            continue
        print(f"-> fastest clean curve speed {top:.2f}")
        print(f"== {ff}: 2) straight speed with curves at {top:.2f}")
        fast = top
        for st in np.round(np.arange(top + 0.05, 1.001, 0.05), 2):
            r = agg(setup, st, top, a.seconds, a.seeds, prof)
            print(f"{st:6.2f} {r['off']:4d} {r['lost']:6.1f} {r['lock']:16.0f} ({r['side']}) {r['steer']:8.2f} {r['lap']:6.2f}", flush=True)
            if r["off"] == 0 and r["lost"] < 2.0:
                fast = st
            else:
                break
        print(f"-> fastest clean straight speed {fast:.2f} with curves at {top:.2f}")
        best[ff] = (fast, top)
    print("\nsummary (straight / curve):", {k: f"{v[0]:.2f} / {v[1]:.2f}" for k, v in best.items()})


if __name__ == "__main__":
    main()
