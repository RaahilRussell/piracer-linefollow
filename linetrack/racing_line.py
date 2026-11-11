"""Racing line: how far from the inside tape the car should ride, as the curve comes and goes.

The lap is about 70% curve and the car is limited by throttle and steering lock, not by grip, so
the time is mostly path length: every centimetre further from the inside tape adds 2*pi cm per
lap. The racing line therefore rides WIDE on the short straights and pulls right in to the inside
tape for the curve:

    straight in view   -> ride near the outer tape (opens the turn, no weave back to the inside)
    curve coming up    -> the ride line slides to the inside tape: an early, smooth turn-in
    in the curve       -> hold the apex distance from the inside tape (the shortest way round)
    straight in view   -> let the car run out wide again

It is not a separate path planner: the result is the `lateral` distance that pure pursuit already
uses to shift the tracked edge into a ride line (linetrack.geometry.pursuit_goal), so it works
from either boundary and needs no pairing of the two.

Distances are metres from the inside (left) tape's lane-side edge to the car's centre line.
"""
from __future__ import annotations

import numpy as np

CAR_HALF_WIDTH_M = 0.07      # Waveshare PiRacer Pro body is 14 cm wide
CURVE_START = 0.4            # inside-edge curvature (1/m) where the line starts to move in
CURVE_FULL = 1.2             # ... and where it is fully at the apex distance (the mat's curves are ~1.8)
LOCK_SHARE = 1.0             # never plan a curve tighter than this share of full steering lock
OUTER_GAP_M = 0.03           # the car's side never planned closer than this to the outer tape
MIN_LANE_M, MAX_LANE_M = 0.20, 0.65


def inner_curvature(edge_curvature: float, source: str, lane_width_m: float) -> float:
    """Curvature of the INSIDE tape from the tracked edge's curvature (+ = left bend). The right
    (outer) tape of a left bend has the larger radius: R_inner = R_outer - lane width."""
    k = float(edge_curvature)
    if source == "right" and k > 0:
        k = k / max(0.2, 1.0 - k * lane_width_m)
    return k


def curve_start_distance(ground, dev_m: float = 0.025) -> float | None:
    """How far ahead (m from the rear axle) the tracked edge starts to bend, or None if it runs
    straight for as far as it is seen. 0 = the car is already in the curve.

    The edge next to the car gives the straight-ahead direction; the curve "starts" where the
    edge has moved `dev_m` off that line. On an arc of radius R that is sqrt(2 R dev) (about
    0.17 m on this track) past the true start, which is taken off again."""
    g = np.asarray(ground, float)
    g = g[np.isfinite(g).all(1)]
    g = g[(g[:, 0] > 0.05) & (np.hypot(g[:, 0], g[:, 1]) < 1.6)]
    if len(g) < 8:
        return None
    d = np.hypot(g[:, 0] - g[0, 0], g[:, 1] - g[0, 1])
    near = g[d < 0.25] if (d < 0.25).sum() >= 4 else g[:6]
    c = near.mean(0)
    _, _, vt = np.linalg.svd(near - c)
    t = vt[0] if vt[0][0] >= 0 else -vt[0]                 # direction of the edge beside the car, pointing ahead
    dev = np.abs((g[:, 0] - c[0]) * t[1] - (g[:, 1] - c[1]) * t[0])
    off = np.nonzero(dev > dev_m)[0]
    off = [i for i in off if i + 1 >= len(g) or dev[i + 1] > dev_m]      # not a single noisy point
    if not off:
        return None
    i = off[0]
    if d[i] < 0.40:                                        # bends right from the nearest point seen: already in the curve
        return 0.0
    return max(0.0, float(np.hypot(g[i, 0], g[i, 1])) - 0.17)


def ride_offset(k_inner: float, lane_width_m: float, apex_m: float, straight_m: float,
                k_max: float | None = None, weight: float | None = None) -> tuple[float | None, dict]:
    """(distance from the inside tape to ride at, diagnostics). None = don't use the racing line.

    k_inner: curvature of the inside tape in view (1/m, + = left bend).
    apex_m: distance to hold in a curve (0 = car centred on the tape edge, negative = over it).
    straight_m: distance to hold on the straights (a little wider than the apex opens the turn).
    k_max: tightest curvature the car can steer (tan(full lock) / wheelbase). The apex distance is
    raised where the inside tape is tighter than the car can turn, so the plan stays drivable."""
    w_lane = float(lane_width_m)
    if not (MIN_LANE_M <= w_lane <= MAX_LANE_M):
        return None, {"reason": "lane width implausible", "lane_m": w_lane}
    limit = w_lane - CAR_HALF_WIDTH_M - OUTER_GAP_M          # never plan the car's side closer than this to the outer tape
    if limit < CAR_HALF_WIDTH_M:
        return None, {"reason": "lane too narrow for the car", "lane_m": w_lane}
    wide = min(float(straight_m), limit)
    apex = min(float(apex_m), wide)
    if k_max and k_inner > 1e-3:
        # ride radius = inside radius + distance; keep it at or above the car's own turning radius
        apex = min(wide, max(apex, 1.0 / (LOCK_SHARE * k_max) - 1.0 / k_inner))
    if weight is None:
        w = min(1.0, max(0.0, (k_inner - CURVE_START) / (CURVE_FULL - CURVE_START)))
        w = w * w * (3.0 - 2.0 * w)                  # smoothstep: no kink at either end
    else:
        w = min(1.0, max(0.0, float(weight)))        # timed through the curve (apex_weight)
    return wide + (apex - wide) * w, {"reason": "ok", "lane_m": w_lane, "wide_m": wide, "apex_m": apex, "curve_weight": w}
