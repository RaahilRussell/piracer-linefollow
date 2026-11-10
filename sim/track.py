"""Track: two orange tape lines; the car drives counter-clockwise (all turns LEFT),
following the LEFT (inside) line. Built as a stadium from the profile's measured
(or, until measured, assumed) straight length, turn radius, lane width and tape width.
A measured polyline can replace it later (Track.from_points)."""
from __future__ import annotations

import math

import numpy as np


class Track:
    def __init__(self, left_pts: np.ndarray, lane_w: float, tape_w: float, closed=True):
        self.left = np.asarray(left_pts, float)              # centreline of the LEFT tape
        self.lane_w, self.tape_w = lane_w, tape_w
        nxt = np.roll(self.left, -1, axis=0)
        t = nxt - self.left
        self.seg_len = np.hypot(t[:, 0], t[:, 1])
        t /= np.maximum(self.seg_len[:, None], 1e-12)
        self.tangent = t
        self.normal_right = np.c_[t[:, 1], -t[:, 0]]           # right of travel
        # RIGHT tape centreline: lane_w + tape_w to the right (lane width = inside edge to inside edge)
        self.right = self.left + (lane_w + tape_w) * self.normal_right
        self.s = np.concatenate([[0.0], np.cumsum(self.seg_len)[:-1]])
        self.length = float(self.seg_len.sum())
        # signed curvature of the left line (+ = left turn)
        h = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
        dh = np.diff(np.r_[h, h[0] + 2 * math.pi])
        self.curv = dh / np.maximum(self.seg_len, 1e-9)
        self._raster = None

    @classmethod
    def stadium(cls, prof, step=0.01):
        L, R = prof.track_straight_m, prof.track_radius_m
        pts = []
        for s in np.arange(0, L, step):
            pts.append((s, 0.0))
        for a in np.arange(-math.pi / 2, math.pi / 2, step / R):
            pts.append((L + R * math.cos(a), R + R * math.sin(a)))
        for s in np.arange(0, L, step):
            pts.append((L - s, 2 * R))
        for a in np.arange(math.pi / 2, 3 * math.pi / 2, step / R):
            pts.append((R * math.cos(a), R + R * math.sin(a)))
        return cls(np.array(pts), prof.lane_width_m, prof.tape_width_m)

    @classmethod
    def from_points(cls, pts, prof):
        return cls(np.asarray(pts, float), prof.lane_width_m, prof.tape_width_m)

    # ------------------------------------------------------------ queries
    def nearest(self, x, y):
        d2 = (self.left[:, 0] - x) ** 2 + (self.left[:, 1] - y) ** 2
        i = int(np.argmin(d2))
        return i, float(self.s[i])

    def offset_from_left(self, x, y):
        """Signed distance from the LEFT tape's INSIDE edge to (x, y), metres:
        + = in the lane side (to the right of the left line), - = over/beyond the left line."""
        i, _ = self.nearest(x, y)
        n = self.normal_right[i]
        d_centre = (x - self.left[i, 0]) * n[0] + (y - self.left[i, 1]) * n[1]
        return d_centre - self.tape_w / 2

    def offsets_from_left(self, points):
        """Vectorized signed offsets for body points, measured from the left tape's inside edge."""
        pts = np.asarray(points, float).reshape(-1, 2)
        delta = pts[:, None, :] - self.left[None, :, :]
        nearest = np.argmin(np.einsum("ijk,ijk->ij", delta, delta), axis=1)
        normals = self.normal_right[nearest]
        return np.einsum("ij,ij->i", pts - self.left[nearest], normals) - self.tape_w / 2

    def corners(self):
        """Constant-curvature stretches: list of dicts (start s, length, radius of the left line, direction)."""
        out, cur = [], None
        for i, k in enumerate(self.curv):
            turning = abs(k) > 0.2
            if turning and cur is None:
                cur = {"i0": i, "k": [k]}
            elif turning:
                cur["k"].append(k)
            elif cur is not None:
                cur["i1"] = i
                out.append(cur)
                cur = None
        if cur is not None:
            cur["i1"] = len(self.curv)
            out.append(cur)
        res = []
        for c in out:
            k = float(np.median(c["k"]))
            res.append({"s0": float(self.s[c["i0"]]), "length": float(self.s[c["i1"] - 1] - self.s[c["i0"]]),
                        "radius_left_line": 1.0 / abs(k), "direction": "left" if k > 0 else "right"})
        return res

    def bounds(self, margin=0.3):
        a = np.vstack([self.left, self.right])
        return a.min(0) - margin, a.max(0) + margin

    def raster(self, res=0.005):
        """Occupancy grid of the tapes (1 = left tape, 2 = right tape) for camera rendering."""
        if self._raster is not None:
            return self._raster
        lo, hi = self.bounds()
        nx, ny = int((hi[0] - lo[0]) / res) + 1, int((hi[1] - lo[1]) / res) + 1
        grid = np.zeros((ny, nx), np.uint8)
        import cv2
        half = max(1, int(round(self.tape_w / 2 / res)))
        for label, line in ((1, self.left), (2, self.right)):
            pix = np.c_[(line[:, 0] - lo[0]) / res, (line[:, 1] - lo[1]) / res].astype(np.int32)
            cv2.polylines(grid, [pix], True, int(label), thickness=2 * half)
        self._raster = (grid, lo, res)
        return self._raster
