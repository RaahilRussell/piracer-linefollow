"""Can the car physically take each corner at a given speed?

Two separate limits:
  geometry  the path radius is tighter than the steering linkage allows at full lock
            (impossible at ANY speed): R_path < wheelbase / tan(max angle)
  traction  holding radius R at speed v needs v^2/R of sideways grip; above mu*g the
            tyres slide and the car runs wide: v > sqrt(mu * g * R)
Both use profile values; assumed ones are flagged in the report.
"""
from __future__ import annotations

import math

from .actuators import SteeringMap
from .vehicle import G


def corner_report(prof, track, ride_offset_m, speed_mps):
    steer = SteeringMap(prof)
    a_l, a_r = steer.max_angle_rad()
    rows = []
    for c in track.corners():
        # the car rides `ride_offset_m` inside the lane from the left line's inside edge;
        # in a LEFT turn that puts it on a larger radius than the left line
        side = c["direction"]
        r_line = c["radius_left_line"]
        r_path = r_line + track.tape_w / 2 + ride_offset_m if side == "left" else r_line - track.tape_w / 2 - ride_offset_m
        a_max = a_l if side == "left" else a_r
        r_min = prof.wheelbase_m / math.tan(a_max) if a_max > 1e-6 else float("inf")
        v_max = math.sqrt(prof.mu_lat * G * r_path)
        need_deg = math.degrees(math.atan(prof.wheelbase_m / r_path))
        if r_path < r_min:
            verdict = f"IMPOSSIBLE at any speed: path radius {r_path:.2f} m < tightest turn {r_min:.2f} m"
        elif speed_mps > v_max:
            verdict = f"TOO FAST: slides above {v_max:.2f} m/s (asked {speed_mps:.2f})"
        else:
            verdict = f"ok ({100 * (1 - speed_mps / v_max):.0f}% speed margin)"
        rows.append({"s0": round(c["s0"], 2), "direction": side, "radius_path_m": round(r_path, 3),
                     "min_radius_m": round(r_min, 3), "steer_needed_deg": round(need_deg, 1),
                     "steer_available_deg": round(math.degrees(a_max), 1), "v_max_mps": round(v_max, 2),
                     "verdict": verdict})
    assumed = [k for k in ("steer_angle_table", "mu_lat", "track_radius_m", "lane_width_m") if prof.entry(k)["status"] == "assumed"]
    return rows, assumed
