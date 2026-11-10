"""Self-calibrate the camera model from recorded frames: no car needed.

On the real track the two tape lines are parallel and a constant lane width apart. Project the
detected inside edges of BOTH lines onto the ground with a candidate camera model; the right
model makes them come out parallel (constant distance along the line) and equally far apart in
every frame. Fit lens FOV, tilt (pitch), roll, fisheye distortion k1 and the optical-centre offset
to that. Both measures are relative (coefficient of variation), so the fit can't cheat by shrinking
the ground.

Height only scales everything (a higher camera sees the same picture of a proportionally bigger
track), so it can't come from parallelism: it's set from ONE real measurement, the lane width
(--lane-width-m, measured with a tape measure between the lines' inside edges), or left as assumed.

  python3 tools/calibrate_camera.py                   # fit, report train/test, write nothing
  python3 tools/calibrate_camera.py --write           # also store the result in sim/profile.json + pi/line_config.py
  python3 tools/calibrate_camera.py --lane-width-m 0.55 --write
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import sys

import cv2
import numpy as np
from scipy.optimize import minimize

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "pi"), ROOT]

import line_config  # noqa: E402
from donkey_part import load_line_config  # noqa: E402
from linetrack.detector import LeftLineDetector  # noqa: E402
from linetrack.geometry import project_pixels  # noqa: E402

DATASETS = {"runs/20261108-185226": None, "tests/frames/stage": None, "runs/20261108-183610": 5}   # parked run: 5 frames
PARAMS = ["camera_fov_deg", "cam_pitch_deg", "cam_roll_deg", "lens_k1", "cam_cx_px"]
START = [160.0, 8.0, 0.0, 0.0, 0.0]
BOUNDS = [(110.0, 200.0), (-5.0, 25.0), (-12.0, 12.0), (-0.35, 0.35), (-25.0, 25.0)]
HEIGHT, FWD = 0.17, 0.15          # assumed (height only sets the scale; see the docstring)
W, H = 240, 192


def collect(max_per_set=None):
    cfg, _ = load_line_config(line_config)
    frames = []
    for d, cap in DATASETS.items():
        det = LeftLineDetector(cfg)
        fs = sorted(glob.glob(os.path.join(ROOT, d, "*.jpg")))
        got = []
        for f in fs:
            r = det.detect(cv2.imread(f))
            if (r.found and r.source == "left" and len(r.edge_points) >= 8 and len(r.right_points) >= 8
                    and r.left_confidence > 0.5 and r.right_confidence > 0.5):
                got.append((d, f, np.asarray(r.edge_points, float), np.asarray(r.right_points, float)))
        if cap and len(got) > cap:
            got = [got[i] for i in np.linspace(0, len(got) - 1, cap).astype(int)]
        frames += got
    return frames


def _ground(pts, p):
    gx, gy = project_pixels(pts[:, 0], pts[:, 1], W, H, p[0], p[1], HEIGHT, FWD, p[2], p[3], p[4])
    g = np.stack([gx, gy], 1)
    ok = np.isfinite(g).all(1) & (g[:, 0] > 0.05) & (g[:, 0] < 1.6)
    return g[ok]


def _dist_to_polyline(pts, line):
    """Distance from each point to a polyline (sorted by X), for points within its X span."""
    line = line[np.argsort(line[:, 0])]
    a, b = line[:-1], line[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = []
    for q in pts:
        if q[0] < line[0, 0] - 0.05 or q[0] > line[-1, 0] + 0.05:
            continue
        t = np.clip(((q - a) * ab).sum(1) / L2, 0, 1)
        proj = a + t[:, None] * ab
        out.append(float(np.min(np.hypot(*(q - proj).T))))
    return np.array(out)


def frame_stats(frames, p):
    """Per frame: mean separation (m) and its coefficient of variation along the line."""
    means, cvs = [], []
    for _, _, L, R in frames:
        gl, gr = _ground(L, p), _ground(R, p)
        if len(gl) < 5 or len(gr) < 5:
            continue
        d = _dist_to_polyline(gr, gl)
        if len(d) < 4 or d.mean() < 1e-3:
            continue
        means.append(d.mean())
        cvs.append(d.std() / d.mean())
    return np.array(means), np.array(cvs)


def cost(p, frames):
    means, cvs = frame_stats(frames, p)
    if len(means) < 0.6 * len(frames):
        return 10.0
    across = means.std() / means.mean()
    return float(np.mean(cvs ** 2) + across ** 2)


def report(name, frames, p):
    means, cvs = frame_stats(frames, p)
    return {"set": name, "frames": len(means), "parallel_cv_median": round(float(np.median(cvs)), 3),
            "width_cv_across_frames": round(float(means.std() / means.mean()), 3),
            "lane_width_m_at_assumed_height": round(float(np.median(means)), 3)}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--lane-width-m", type=float, help="measured lane width (inside edge to inside edge) -> camera height")
    a = ap.parse_args(argv)
    frames = collect()
    blocks = np.arange(len(frames)) // 8              # alternate blocks of 8 frames: train / test
    train = [f for f, b in zip(frames, blocks) if b % 3 != 2]
    test = [f for f, b in zip(frames, blocks) if b % 3 == 2]
    print(f"{len(frames)} frames with both lines ({len(train)} fit, {len(test)} held out)")
    before = (report("fit", train, START), report("held out", test, START))
    best = None
    for start in (START, [150, 12, 0, 0.1, 0], [170, 4, 0, -0.1, 0]):
        r = minimize(cost, start, args=(train,), method="Powell", bounds=BOUNDS,
                     options={"maxiter": 4000, "xtol": 1e-3, "ftol": 1e-6})
        if best is None or r.fun < best.fun:
            best = r
    p = list(best.x)
    after = (report("fit", train, p), report("held out", test, p))
    print("\nparameter        before   after")
    for n, s0, s1 in zip(PARAMS, START, p):
        print(f"{n:<16}{s0:>7.2f}{s1:>8.2f}")
    print("\n(parallel CV: how much the gap between the lines varies along a frame; width CV: how much it")
    print(" varies between frames. Both should be small for a correct camera model.)")
    for b, c in zip(before, after):
        print(f"{b['set']:<9} parallel CV {b['parallel_cv_median']:.3f} -> {c['parallel_cv_median']:.3f}   "
              f"width CV {b['width_cv_across_frames']:.3f} -> {c['width_cv_across_frames']:.3f}   "
              f"lane width at h={HEIGHT} m: {b['lane_width_m_at_assumed_height']} -> {c['lane_width_m_at_assumed_height']} m")
    height = HEIGHT
    if a.lane_width_m:
        height = HEIGHT * a.lane_width_m / after[1]["lane_width_m_at_assumed_height"]
        print(f"\ncamera height from the measured lane width {a.lane_width_m} m: {height:.3f} m")
    result = dict(zip(PARAMS, [round(float(v), 3) for v in p]), cam_height_m=round(height, 3),
                  before=before, after=after, frames=len(frames))
    out = os.path.join(ROOT, "tools", "camera_calibration.json")
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nsaved {out}")
    if a.write:
        write(result, measured_height=bool(a.lane_width_m))
    return result


def write(res, measured_height=False):
    """Store the fit in sim/profile.json (status: measured) and pi/line_config.py."""
    path = os.path.join(ROOT, "sim", "profile.json")
    with open(path) as f:
        prof = json.load(f)
    src = "Self-calibrated from recorded frames (tools/calibrate_camera.py): both tape edges projected parallel and a constant width apart."
    defaults = {"cam_roll_deg": ("deg", [-12, 12]), "lens_k1": ("", [-0.35, 0.35]), "cam_cx_px": ("px", [-25, 25])}
    for k in PARAMS + (["cam_height_m"] if measured_height else []):
        e = prof.get(k) or {"value": 0.0, "unit": defaults.get(k, ("", None))[0], "range": defaults.get(k, ("", None))[1]}
        e.update(value=res[k], status="measured", source=src + (" Height from the measured lane width." if k == "cam_height_m" else ""))
        prof[k] = e
    with open(path, "w") as f:
        json.dump(prof, f, indent=2)
    lc = os.path.join(ROOT, "pi", "line_config.py")
    s = open(lc).read()
    for k, name in (("camera_fov_deg", "CAMERA_FOV_DEG"), ("cam_pitch_deg", "CAM_PITCH_DEG"), ("cam_roll_deg", "CAM_ROLL_DEG"),
                    ("lens_k1", "LENS_K1"), ("cam_cx_px", "CAM_CX_PX")) + ((("cam_height_m", "CAM_HEIGHT_M"),) if measured_height else ()):
        line = f"{name} = {res[k]}"
        if re.search(rf"^{name} = .*$", s, re.M):
            s = re.sub(rf"^{name} = .*$", line + "   # self-calibrated (tools/calibrate_camera.py)", s, flags=re.M)
        else:
            s = s.replace("CAM_FWD_M = ", line + "   # self-calibrated (tools/calibrate_camera.py)\nCAM_FWD_M = ", 1)
    open(lc, "w").write(s)
    print("wrote sim/profile.json and pi/line_config.py")


if __name__ == "__main__":
    main()
