"""Camera model: the PiRacer's 160-degree wide-angle camera looking at the ground.

Projection: equidistant fisheye (r = f * theta), with f set so the image diagonal
spans camera_fov_deg (ASSUMED interpretation of Waveshare's "160 deg"; see the
profile). Mounted cam_fwd_m ahead of the rear axle, cam_height_m up, tilted
cam_pitch_deg down (all ASSUMED until measured).

Every pixel's ray is intersected with the ground once (it only depends on the
mount), giving a table pixel -> ground point in the car frame. Then per frame:
  geometric mode : read the lookahead row only (where is the left tape's inside edge)
  rendered mode  : paint the whole image (orange tape on lavender mat) and run the
                   REAL detector (linetrack.detector) on it
"""
from __future__ import annotations

import math

import numpy as np

MAT_RGB = np.array([200, 190, 225], np.uint8)
TAPE_RGB = np.array([235, 80, 45], np.uint8)
WOOD_RGB = np.array([205, 160, 120], np.uint8)


class Camera:
    def __init__(self, prof):
        self.p = prof
        self.W, self.H = int(prof.image_w_px), int(prof.image_h_px)
        self._build()

    def _build(self):
        from linetrack.geometry import project_pixels
        p, W, H = self.p, self.W, self.H
        opt = lambda k, d: getattr(p, k) if k in p else d       # optional profile entries (calibration)
        u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
        self.gx, self.gy = project_pixels(u, v, W, H, p.camera_fov_deg, p.cam_pitch_deg, p.cam_height_m, p.cam_fwd_m,
                                          opt("cam_roll_deg", 0.0), opt("lens_k1", 0.0), opt("cam_cx_px", 0.0))
        self.f = math.hypot(W / 2, H / 2) / math.radians(p.camera_fov_deg / 2)
        far = np.hypot(self.gx - p.cam_fwd_m, self.gy) > 4.0      # past the mat: background
        self.gx[far] = np.nan
        self.gy[far] = np.nan

    def row_distance(self, row_ratio):
        """Ground distance (m ahead of the rear axle) seen at the centre of an image row."""
        r = min(self.H - 1, int(row_ratio * self.H))
        return float(self.gx[r, self.W // 2])

    def labels(self, pose, track, rows=None):
        """Tape label per pixel (0 mat, 1 left tape, 2 right tape, 255 off-ground)."""
        x, y, yaw = pose
        gx, gy = (self.gx, self.gy) if rows is None else (self.gx[rows], self.gy[rows])
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + gx * c - gy * s
        wy = y + gx * s + gy * c
        grid, lo, res = track.raster()
        ix = np.floor((wx - lo[0]) / res)
        iy = np.floor((wy - lo[1]) / res)
        ok = np.isfinite(ix) & (ix >= 0) & (iy >= 0) & (ix < grid.shape[1]) & (iy < grid.shape[0])
        out = np.full(gx.shape, 255, np.uint8)
        out[ok] = grid[iy[ok].astype(int), ix[ok].astype(int)]
        out[np.isfinite(gx) & ~ok] = 0
        return out

    def render(self, pose, track, rng=None):
        """Synthetic RGB frame (what the real detector sees)."""
        lab = self.labels(pose, track)
        img = np.empty((self.H, self.W, 3), np.uint8)
        img[:] = WOOD_RGB
        img[lab == 0] = MAT_RGB
        img[(lab == 1) | (lab == 2)] = TAPE_RGB
        if rng is not None:
            img = np.clip(img.astype(np.int16) + rng.normal(0, 4, img.shape), 0, 255).astype(np.uint8)
        return img

    def measure_row(self, pose, track, row_ratio, lane_ratio):
        """Geometric fast path: what the detector reports at the lookahead row.
        Returns (x_ratio or None, source 'left'/'right'/None, both_visible, lane_ratio_seen)."""
        r = min(self.H - 1, int(row_ratio * self.H))
        lab = self.labels(pose, track, rows=slice(r, r + 1))[0]
        left = np.nonzero(lab == 1)[0]
        right = np.nonzero(lab == 2)[0]
        xl = (left.max() + 1) / self.W if len(left) else None          # inside edge = right edge of the left tape
        xr = right.min() / self.W if len(right) else None               # inside edge = left edge of the right tape
        both = xl is not None and xr is not None and xr > xl
        seen = (xr - xl) if both else None
        if xl is not None and xl < 0.995:
            return xl, "left", both, seen
        if xr is not None:
            return xr - lane_ratio, "right", False, None
        return None, None, False, None


EDGE_ROWS = tuple(round(0.48 + 0.04 * i, 2) for i in range(12))   # 0.48 .. 0.92 of the height


def measure_edges(cam: "Camera", pose, track, rows=EDGE_ROWS):
    """Geometric fast path for corner geometry: the inside-edge pixels of the left and right tape
    on several image rows (what the real detector's edge points are). Returns (left_pts, right_pts),
    each an array of (x_px, y_px)."""
    left, right = [], []
    for rr in rows:
        r = min(cam.H - 1, int(rr * cam.H))
        lab = cam.labels(pose, track, rows=slice(r, r + 1))[0]
        lx = np.nonzero(lab == 1)[0]
        rx = np.nonzero(lab == 2)[0]
        if len(lx) and lx.max() < cam.W - 1:
            left.append((lx.max() + 1.0, r + 0.5))
        if len(rx) and (not len(lx) or rx.min() > lx.max()):
            right.append((float(rx.min()), r + 0.5))
    return np.array(left, float).reshape(-1, 2), np.array(right, float).reshape(-1, 2)


def target_offset(cam: Camera, track_like_straight_lane_w, target_ratio, row_ratio, tape_w):
    """Lateral distance from the left tape's inside edge at which, driving straight and
    parallel, that edge appears at `target_ratio` on the lookahead row. (Where the car
    actually rides when it holds the target.)"""
    r = min(cam.H - 1, int(row_ratio * cam.H))
    u = target_ratio * cam.W
    i = int(np.clip(round(u - 0.5), 0, cam.W - 1))
    gy = cam.gy[r, i]          # ground point seen at that pixel, car frame (left +)
    return float(gy) if np.isfinite(gy) else None
