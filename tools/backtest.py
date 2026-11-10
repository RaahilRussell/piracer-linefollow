#!/usr/bin/env python3
"""Backtest line-follower changes on recorded frames before they go on the car.

Replays saved camera frames through the real detector + pilot (exactly the code
that runs on the Pi) for several "variants" (setting overrides) and scores each:

  tracked%      frames where a line was found
  left%         ... found from the left line itself (not the right-line backup)
  wrong         labelled frames where it picked a stripe that is NOT the left line
  missed        labelled frames where the left line was visible but not used
  jumps/1k      tracked position teleporting > 0.12 of the width between frames
  lock%         steering at its limit
  flips/100     steering sign changes (weaving)
  stops         (local-mode replay) times throttle was cut to 0
  blind_err     mean |steering while blind - steering once the line came back|
  ms            detector+pilot time per frame on this machine

Labels (tools/labels.json) say, for sampled frames, which stripe is really the
left line (its bottom x as a fraction of width) or null if none is visible.

  python3 tools/backtest.py                    # baseline vs all variants, all datasets
  python3 tools/backtest.py --variants baseline,confirm
  python3 tools/backtest.py sheets OUTDIR      # contact sheets for labelling
"""
from __future__ import annotations

import argparse
import copy
import csv
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "pi"), ROOT]

import line_config  # noqa: E402
from donkey_part import AutoCrawl, LineDetectorPart, LineFollowerPilot, SpeedPlan, car_model, load_line_config  # noqa: E402
from linetrack.config import Config  # noqa: E402
from linetrack.detector import LeftLineDetector  # noqa: E402

LABELS = os.path.join(ROOT, "tools", "labels.json")

# name -> (frame glob, seconds per frame when there's no log, recorded log or None)
DATASETS = {
    "track_run": (os.path.join(ROOT, "runs", "20261108-185226", "*.jpg"), 0.05,
                  os.path.join(ROOT, "runs", "20261108-185226", "log.csv")),
    "carpet_run": (os.path.join(ROOT, "runs", "20261109-193803", "*.jpg"), 0.05,
                   os.path.join(ROOT, "runs", "20261109-193803", "log.csv")),
    "stage_laps": (os.path.join(ROOT, "tests", "frames", "stage", "*.jpg"), 0.2, None),
    "hallway": (os.path.join(ROOT, "tests", "frames", "live2", "*.jpg"), 0.2, None),
}

# Each variant = Config overrides (+ pilot options: pilot_coast_s, pilot_slow_s, ...).
VARIANTS = {
    "baseline": {},   # = line_config.py as it is now
    # mask: strict threshold only (no growth into nearly-orange pixels)
    "no_grow": {"mask_grow_px": 0},
    # candidates: no ragged-edge filter (wood grain / skin are allowed through)
    "no_rough": {"max_edge_roughness_px": 0.0},
    # identity: forget a vanished boundary at once / hold it longer
    "memory0": {"track_memory_frames": 0, "side_memory_frames": 0},
    "memory20": {"track_memory_frames": 20},
    # a single line with no history must hold 2 or 3 frames before it is trusted
    "confirm2": {"lone_confirm_frames": 2},
    "confirm3": {"lone_confirm_frames": 3},
    # left line only (no right-line backup)
    "left_only": {"use_right_fallback": False},
    # lost line: cut throttle at once and re-centre / coast longer
    "no_coast": {"lost_hold_heading": False, "pilot_coast_s": 0.0, "pilot_slow_s": 0.0},
    "coast1/1": {"lost_hold_heading": True, "pilot_coast_s": 1.0, "pilot_slow_s": 1.0},
}


def load_frames(name):
    pattern, dt, log = DATASETS[name]
    files = sorted(glob.glob(pattern))
    times = None
    if log and os.path.exists(log):
        rows = {}
        with open(log, errors="replace") as f:
            for r in csv.DictReader(line.replace("\0", "") for line in f):
                try:                       # a power cut mid-write leaves junk rows: skip them
                    rows[int(r["n"])] = float(r["t"])
                except (TypeError, ValueError):
                    continue
        times = []
        for p in files:
            n = int(os.path.splitext(os.path.basename(p))[0])
            times.append(rows.get(n))
        t0 = next(t for t in times if t is not None)
        times = [(t - t0) if t is not None else None for t in times]
    if times is None or any(t is None for t in times):
        times = [i * dt for i in range(len(files))]
    return files, times


def build(variant: dict):
    cfg, ex = load_line_config(line_config)
    pilot_opts = {}
    for k, v in variant.items():
        if k.startswith("speed_") or k == "auto_crawl":
            ex[k.upper()] = v
        elif k.startswith("pilot_"):
            pilot_opts[k[6:]] = v
        else:
            if not hasattr(cfg, k):
                raise KeyError(f"unknown setting {k}")
            setattr(cfg, k, v)

    class Clock:
        t = 0.0

        def __call__(self):
            return self.t
    clk = Clock()
    crawl = AutoCrawl(start=ex["CRAWL_START"], hi=min(ex["CRAWL_MAX"], cfg.max_throttle),
                      ramp_up=ex["CRAWL_RAMP_UP"], move_thresh=ex["CRAWL_MOVE_MOTION"],
                      fast_thresh=ex["CRAWL_FAST_MOTION"])
    if not ex["AUTO_CRAWL"]:
        crawl = None
    opts = {"coast_s": ex["LOST_COAST_S"], "slow_s": ex["LOST_SLOW_S"], "single_line_stop_s": ex["SINGLE_LINE_STOP_S"]}
    opts.update(pilot_opts)
    pilot = LineFollowerPilot(cfg, clock=clk, crawl=crawl, **opts)
    pilot.model = car_model(ex)
    if ex["SPEED_STRAIGHT"] > 0:
        pilot.speed_plan = SpeedPlan(ex["SPEED_STRAIGHT"], ex["SPEED_CORNER"] or ex["SPEED_STRAIGHT"],
                                     transition=ex["SPEED_TRANSITION"] or None,
                                     ramp_down=ex["SPEED_RAMP_DOWN"], ramp_up=ex["SPEED_RAMP_UP"])
    det = LineDetectorPart(cfg, clock=clk, show_overlay=False, model=pilot.model)
    det.attach(pilot)
    return cfg, clk, det, pilot


def run(variant: dict, dataset: str, mode="local_angle", labels=None, frames=None, progress=None):
    files, times = frames or load_frames(dataset)
    cfg, clk, det, pilot = build(variant)
    out = []
    for index, (f, t) in enumerate(zip(files, times)):
        if progress is not None and index % 100 == 0:
            progress(index)
        clk.t = 1000.0 + t
        bgr = cv2.imread(f)
        if bgr is None:
            out.append({"name": os.path.basename(f), "found": False, "x": None, "source": "none",
                        "status": "CORRUPT", "angle": 0., "throttle": 0., "ms": 0., "sel_near": None,
                        "n_stripes": 0, "section": "unknown", "corner_weight": 1., "curvature": None,
                        "speed_target": None})
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        t0 = time.perf_counter()
        x, found, age, _, motion, both = det.run(rgb)
        angle, thr = pilot.run(mode, x, found, age, motion, both)
        ms = (time.perf_counter() - t0) * 1000
        d = det._det
        sel = d.selected
        out.append({
            "name": os.path.basename(f), "found": bool(found), "x": x, "source": d.source,
            "status": pilot.last_cmd.status, "angle": angle, "throttle": thr, "ms": ms,
            "sel_near": (sel.near_x / (d.width / d.scale)) if (sel is not None and d.source == "left") else None,
            "n_stripes": len(d.stripes),
            "section": pilot.last_cmd.section, "corner_weight": pilot.last_cmd.corner_weight,
            "curvature": pilot.curvature, "speed_target": pilot.speed_target,
        })
    return out


def score(rows, max_steer, labels=None):
    n = len(rows)
    found = [r for r in rows if r["found"]]
    xs = [r["x"] for r in rows]
    jumps = sum(1 for a, b in zip(xs, xs[1:]) if a is not None and b is not None and abs(a - b) > 0.12)
    ang = np.array([r["angle"] for r in rows])
    signs = np.sign(np.where(np.abs(ang) > 0.05, ang, 0))
    nz = signs[signs != 0]
    flips = int(np.sum(nz[1:] != nz[:-1])) if len(nz) > 1 else 0
    thr = [r["throttle"] for r in rows]
    stops = sum(1 for a, b in zip(thr, thr[1:]) if a > 0 and b == 0)
    # blind steering error: steering held while LOST vs steering once tracking resumes
    blind, held = [], None
    for r in rows:
        if r["status"] != "TRACKING":
            held = r["angle"] if held is None else held
            last_blind = r["angle"]
        elif held is not None:
            blind.append(abs(last_blind - r["angle"]))
            held = None
    s = {
        "frames": n, "tracked%": 100 * len(found) / n,
        "left%": 100 * sum(1 for r in found if r["source"] == "left") / n,
        "jumps/1k": 1000 * jumps / n, "lock%": 100 * float(np.mean(np.abs(ang) >= 0.95 * max_steer)),
        "flips/100": 100 * flips / n, "stops": stops,
        "blind_err": float(np.mean(blind)) if blind else 0.0,
        "ms": float(np.mean([r["ms"] for r in rows])),
        "clutter": float(np.mean([r["n_stripes"] for r in rows])),
    }
    if labels:
        wrong = missed = checked = phantom = 0
        for r in rows:
            if r["name"] not in labels:
                continue
            truth = labels[r["name"]]
            checked += 1
            if truth is None:
                if r["sel_near"] is not None:
                    phantom += 1       # claims a left line where there is none
            else:
                if r["sel_near"] is None:
                    missed += 1
                elif abs(r["sel_near"] - truth) > 0.07:
                    wrong += 1
        s.update({"labelled": checked, "wrong": wrong, "phantom": phantom, "missed": missed})
    return s


def table(results):
    cols = ["tracked%", "left%", "wrong", "phantom", "missed", "jumps/1k", "lock%", "flips/100", "stops", "blind_err", "clutter", "ms"]
    for ds, by_variant in results.items():
        print(f"\n== {ds} ({next(iter(by_variant.values()))['frames']} frames"
              f"{', ' + str(next(iter(by_variant.values())).get('labelled', 0)) + ' labelled' if 'labelled' in next(iter(by_variant.values())) else ''})")
        print(f"{'variant':<16}" + "".join(f"{c:>10}" for c in cols))
        for v, s in by_variant.items():
            cells = []
            for c in cols:
                val = s.get(c, "")
                cells.append(f"{val:>10.1f}" if isinstance(val, float) else f"{val!s:>10}")
            print(f"{v:<16}" + "".join(cells))


def load_labels():
    if os.path.exists(LABELS):
        with open(LABELS) as f:
            return json.load(f)
    return {}


def sheets(outdir, every=None):
    """Contact sheets: every candidate stripe numbered with its bottom x, for labelling."""
    os.makedirs(outdir, exist_ok=True)
    cfg = Config()
    cfg.s_lo = 90  # permissive, so the true stripe is always among the candidates
    plan = {"track_run": 20, "stage_laps": 5, "hallway": 2}
    index = {}
    for ds, step in plan.items():
        files, _ = load_frames(ds)
        det = LeftLineDetector(cfg)
        tiles = []
        for i, f in enumerate(files):
            bgr = cv2.imread(f)
            d = det.detect(bgr)
            if i % (every or step):
                continue
            img = cv2.resize(bgr, None, fx=1.5, fy=1.5)
            sc = d.scale * 1.5
            W = d.width / d.scale
            cands = []
            for k, s in enumerate(sorted(d.stripes, key=lambda s: s.near_x)):
                x, y, w, h = s.bbox
                cv2.rectangle(img, (int(x * sc), int(y * sc)), (int((x + w) * sc), int((y + h) * sc)), (255, 255, 0), 1)
                cv2.putText(img, str(k), (int(s.near_x * sc) - 4, int(s.near_y * sc) - 2), 0, 0.55, (0, 255, 255), 2)
                cands.append(round(s.near_x / W, 3))
            cv2.rectangle(img, (0, 0), (img.shape[1], 16), (0, 0, 0), -1)
            cv2.putText(img, f"{ds}/{os.path.basename(f)}", (3, 12), 0, 0.42, (255, 255, 255), 1)
            index[f"{ds}/{os.path.basename(f)}"] = cands
            tiles.append(img)
        for j in range(0, len(tiles), 12):
            chunk = tiles[j:j + 12]
            while len(chunk) % 4:
                chunk.append(np.zeros_like(tiles[0]))
            rows = [np.hstack(chunk[k:k + 4]) for k in range(0, len(chunk), 4)]
            cv2.imwrite(os.path.join(outdir, f"{ds}_{j // 12:02d}.jpg"), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 80])
    with open(os.path.join(outdir, "candidates.json"), "w") as f:
        json.dump(index, f, indent=1)
    print(f"wrote sheets + candidates.json to {outdir}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", nargs="?", default="score")
    ap.add_argument("outdir", nargs="?")
    ap.add_argument("--variants", default=",".join(VARIANTS))
    ap.add_argument("--datasets", default=",".join(DATASETS))
    ap.add_argument("--frames-dir", help="Replay every .jpg in this folder as one extra dataset")
    ap.add_argument("--dt", type=float, default=0.05, help="Seconds between frames for --frames-dir")
    ap.add_argument("--labels", help="Optional labels.json containing labels for the custom dataset")
    ap.add_argument("--json")
    ap.add_argument("--speed-straight", type=float, help="Replay a Full Auto straight throttle target")
    ap.add_argument("--speed-corner", type=float, help="Replay a Full Auto corner throttle target")
    ap.add_argument("--speed-transition", type=float, help="Replay a Full Auto transition throttle target")
    a = ap.parse_args()
    if a.cmd == "sheets":
        return sheets(a.outdir)
    labels = load_labels()
    if a.labels:
        with open(a.labels) as f:
            labels = json.load(f)
    if a.frames_dir:
        folder = os.path.abspath(a.frames_dir)
        DATASETS["new_frames"] = (os.path.join(folder, "*.jpg"), a.dt, os.path.join(folder, "log.csv"))
    datasets = DATASETS
    results = {}
    requested = ["new_frames"] if a.frames_dir and a.datasets == "track_run,stage_laps,hallway" else a.datasets.split(",")
    for ds in requested:
        if ds not in datasets:
            raise SystemExit(f"unknown dataset {ds!r}; choose from {', '.join(datasets)}")
        frames = load_frames(ds)
        results[ds] = {}
        for v in a.variants.split(","):
            variant = dict(VARIANTS[v])
            for key in ("speed_straight", "speed_corner", "speed_transition"):
                value = getattr(a, key)
                if value is not None:
                    variant[key] = value
            cfg, *_ = build(variant)
            rows_a = run(variant, ds, "local_angle", frames=frames)
            s = score(rows_a, cfg.max_steering, labels.get(ds))
            rows_l = run(variant, ds, "local", frames=frames)       # throttle behaviour
            sl = score(rows_l, cfg.max_steering)
            s["stops"], s["blind_err"] = sl["stops"], sl["blind_err"]
            s["section_frames"] = {name: sum(r["section"] == name for r in rows_l)
                                   for name in ("straight", "corner", "unknown")}
            s["throttle_mean_abs_change"] = float(np.mean(np.abs(np.diff([r["throttle"] for r in rows_l])))) if len(rows_l) > 1 else 0.0
            s["interpretation"] = "Recorded-image replay: control outputs only; no predicted physical lap time"
            results[ds][v] = s
    table(results)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(results, f, indent=1)


if __name__ == "__main__":
    main()
