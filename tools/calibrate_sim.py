#!/usr/bin/env python3
"""Fit the simulator's unmeasured car values to what the REAL car showed, on the Waveshare track.

Real-car targets (from recorded frames and runs on Nov 8-9):
  - in curves the steering sits at full lock most of the time (84-96% of curve frames)
  - this morning's setup (0.70 throttle, KP 3.5, target 0.40) held the line
For each candidate full-lock wheel angle, drive the morning setup on the Waveshare-estimated track
and report curve steering use and off-tracks. The best candidate matches both targets.

    python3 tools/calibrate_sim.py [--angles 12,15,18,21,25] [--radius 0.55 --straight 0.76 --lane 0.35]
"""
import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "tools"), os.path.join(ROOT, "pi"), ROOT]
from sim.params import Profile  # noqa: E402
from sim_limits import BASE, drive  # noqa: E402

REAL_LOCK_PCT = (84, 96)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angles", default="12,14,16,18,21,25")
    ap.add_argument("--radius", type=float, default=0.55)
    ap.add_argument("--straight", type=float, default=0.76)
    ap.add_argument("--lane", type=float, default=0.35)
    ap.add_argument("--seconds", type=float, default=25)
    ap.add_argument("--seeds", type=int, default=2)
    a = ap.parse_args()
    print(f"Waveshare estimate: curve radius {a.radius} m (inner line), straight {a.straight} m, lane {a.lane} m")
    print(f"Real car: full lock {REAL_LOCK_PCT[0]}-{REAL_LOCK_PCT[1]}% of curve frames; 0.70 / KP 3.5 held the line\n")
    print(f"{'full-lock angle':>15s} {'speed':>6s} {'off':>4s} {'full lock%':>11s} {'|steer|':>8s} {'lap s':>6s}")
    for ang in [float(x) for x in a.angles.split(",")]:
        for sp in (0.55, 0.70):
            prof = Profile.load()
            prof._e.pop("track_polyline", None)     # this fit uses the estimated stadium, not the traced line
            prof._e["track_radius_m"]["value"] = a.radius
            prof._e["track_straight_m"]["value"] = a.straight
            prof._e["lane_width_m"]["value"] = a.lane
            prof._e["steer_angle_table"]["value"] = [[300, ang], [375, 0.0], [450, -ang]]
            rs = [drive({**BASE, "max_steering": 0.8, "ff_gain": 0.0}, sp, sp, a.seconds, sd, prof) for sd in range(a.seeds)]
            laps = [r["lap"] for r in rs if r["lap"]]
            print(f"{ang:13.0f}° {sp:6.2f} {sum(r['off'] for r in rs):4d} {np.mean([r['lock'] for r in rs]):11.0f} "
                  f"{np.mean([r['steer'] for r in rs]):8.2f} {np.mean(laps) if laps else float('nan'):6.2f}", flush=True)


if __name__ == "__main__":
    main()
