#!/usr/bin/env python3
"""Audit recorded camera frames (any driver) for the three failures seen during auto-tuning:

  1. line not recognised at speed       -> where/when tracking drops, vs recorded motion/throttle
  2. running off at the end of a curve  -> what the pilot does in the frames after a curve ends
  3. not following the curve            -> error and steering use while inside a curve

Replays every frame through the current detector + pilot (the same code as the car) and
compares with what the car logged. Nothing here moves the car.

    python3 tools/curve_audit.py analysis_runs/<id>/recordings [--json out.json]
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "tools"), os.path.join(ROOT, "pi"), ROOT]
from backtest import run  # noqa: E402


def load_log(d):
    path = os.path.join(d, "log.csv")
    rows = {}
    if not os.path.exists(path):
        return rows
    with open(path, errors="replace") as f:
        for r in csv.DictReader(line.replace("\0", "") for line in f):
            try:
                rows[int(r["n"])] = r
            except (TypeError, ValueError, KeyError):
                continue
    return rows


def fnum(v):
    try:
        x = float(v)
        return x if np.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def segments(flags):
    """[(start, end_exclusive)] of runs of True."""
    out, start = [], None
    for i, f in enumerate(flags):
        if f and start is None:
            start = i
        elif not f and start is not None:
            out.append((start, i)); start = None
    if start is not None:
        out.append((start, len(flags)))
    return out


def audit_run(d, variant=None):
    files = sorted(glob.glob(os.path.join(d, "*.jpg")))
    log = load_log(d)
    times = []
    for i, p in enumerate(files):
        n = int(os.path.splitext(os.path.basename(p))[0])
        times.append(fnum(log.get(n, {}).get("t")))
    if any(t is None for t in times) or not times:
        times = [i * 0.05 for i in range(len(files))]
    else:
        t0 = times[0]; times = [t - t0 for t in times]
    rows = run(variant or {}, None, mode="local", frames=(files, times))
    n = len(rows)
    rec = [log.get(int(os.path.splitext(os.path.basename(p))[0]), {}) for p in files]
    found = np.array([r["found"] for r in rows])
    sect = [r["section"] for r in rows]
    ang = np.array([r["angle"] or 0.0 for r in rows])
    curv = np.array([np.nan if r["curvature"] is None else r["curvature"] for r in rows])
    motion = np.array([np.nan if fnum(x.get("motion")) is None else fnum(x.get("motion")) for x in rec])
    thr_rec = np.array([np.nan if fnum(x.get("throttle")) is None else fnum(x.get("throttle")) for x in rec])
    found_rec = np.array([str(x.get("found", "")).lower() in ("1", "true") for x in rec])

    # 1. tracking loss: who loses it, and how fast was the car going
    losses = [(a, b) for a, b in segments(~found) if b - a >= 2]
    loss_info = []
    for a, b in losses:
        before = [s for s in sect[max(0, a - 10):a] if s in ("straight", "corner")]
        where = max(set(before), key=before.count) if before else "unknown"
        m = motion[max(0, a - 5):a]
        loss_info.append({"start": a, "frames": b - a, "after": where,
                          "steer_before": round(float(np.mean(np.abs(ang[max(0, a - 5):a]))), 3) if a else None,
                          "motion_before": None if not np.isfinite(m).any() else round(float(np.nanmean(m)), 2)})
    mv = np.isfinite(motion)
    speed_split = None
    if mv.sum() > 100:
        q = np.nanpercentile(motion, [33, 67])
        bands = {"slow": motion <= q[0], "mid": (motion > q[0]) & (motion <= q[1]), "fast": motion > q[1]}
        speed_split = {k: {"frames": int(v.sum()), "tracked_pct": round(100 * found[v].mean(), 1),
                           "car_tracked_pct": round(100 * found_rec[v].mean(), 1)} for k, v in bands.items()}

    # 2. curve exits: the 20 frames after the pilot calls a curve finished
    exits = []
    for i in range(1, n):
        if sect[i - 1] == "corner" and sect[i] != "corner":
            w = slice(i, min(n, i + 20))
            lost = (~found[w]).sum()
            # still turning hard after the curve was declared over = released too early
            hard = (np.abs(ang[w]) > 0.6).sum()
            exits.append({"frame": i, "lost_next20": int(lost), "hard_steer_next20": int(hard),
                          "to": sect[i]})
    early = [e for e in exits if e["hard_steer_next20"] >= 5]

    # 3. curve following: in corners, how much steering is used and how big the error is
    in_c = np.array([s == "corner" for s in sect]) & found
    corner = None
    if in_c.sum():
        err = np.array([np.nan if r["x"] is None else r["x"] for r in rows])
        corner = {"frames": int(in_c.sum()),
                  "steer_mean": round(float(np.mean(np.abs(ang[in_c]))), 3),
                  "steer_at_limit_pct": round(100 * float(np.mean(np.abs(ang[in_c]) >= 0.95)), 1),
                  "radius_median_m": round(float(np.nanmedian(1 / np.abs(curv[in_c]))), 2) if np.isfinite(curv[in_c]).any() else None}
    return {"run": os.path.basename(d.rstrip("/")), "frames": n,
            "tracked_pct": round(100 * found.mean(), 1), "car_tracked_pct": round(100 * found_rec.mean(), 1),
            "sections": {s: sect.count(s) for s in ("straight", "corner", "unknown")},
            "losses": len(loss_info), "losses_after": {k: sum(1 for l in loss_info if l["after"] == k) for k in ("straight", "corner", "unknown")},
            "loss_detail": loss_info[:40], "by_speed": speed_split,
            "curve_exits": len(exits), "exits_still_turning_hard": len(early),
            "exits_lost_line": sum(1 for e in exits if e["lost_next20"] >= 3),
            "exit_detail": exits[:40], "corner": corner}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("--json")
    a = ap.parse_args()
    dirs = sorted(d for d in glob.glob(os.path.join(a.root, "*")) if os.path.isdir(d) and glob.glob(os.path.join(d, "*.jpg")))
    out = []
    for d in dirs:
        r = audit_run(d)
        out.append(r)
        c = r["corner"] or {}
        print(f"{r['run']}: {r['frames']} frames, tracked {r['tracked_pct']}% (old code on the day: {r["car_tracked_pct"]}%), "
              f"sections {r['sections']}")
        print(f"   line losses {r['losses']} (after {r['losses_after']})"
              + (f"; by speed {r['by_speed']}" if r['by_speed'] else ""))
        print(f"   curve exits {r['curve_exits']}: still turning hard after exit {r['exits_still_turning_hard']}, "
              f"lost line within 1 s {r['exits_lost_line']}")
        if c:
            print(f"   in curves: {c['frames']} frames, |steer| {c['steer_mean']}, at limit {c['steer_at_limit_pct']}%, "
                  f"median radius {c['radius_median_m']} m")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=1, default=float)


if __name__ == "__main__":
    main()
