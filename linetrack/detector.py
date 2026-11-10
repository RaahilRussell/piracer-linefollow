"""Orange LEFT-boundary detector.

Pipeline (per frame, at <=240 px width; the live Donkeycar stream is 240x192):
  BGR -> HSV -> threshold (two hue bands) -> close -> ROI -> connected components
  -> classify each stripe as LEFT or RIGHT boundary -> pick the left one
     (continuity with the previous fit first, geometry second)
  -> sample its RIGHT (inside) edge on many scanlines -> robust poly fit x(y)
  -> evaluate at the lookahead row.

How left vs right boundary is decided, seen from inside the track:
  * the LEFT boundary leans RIGHT going up the image (x grows as y shrinks),
  * the RIGHT boundary leans LEFT going up the image,
  * with two or more stripes, the left boundary is the leftmost one.
A lone stripe that leans left (or starts far right) is treated as the right
boundary and ignored, so the tracker reports LOST instead of switching lines.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from .config import Config


@dataclass
class Stripe:
    label: int
    area: int
    bbox: tuple            # x, y, w, h (processing coords)
    near_x: float          # stripe centre x at its lowest (nearest) rows
    near_y: float
    lean: float            # dx/d(up): + = leans right going up (left-boundary-like)
    role: str = "?"        # "LEFT" (tracked), "RIGHT_REF" (right boundary used as fallback),
                           # "RIGHT" (right-boundary-like, not used), "other"


@dataclass
class Detection:
    found: bool = False
    confidence: float = 0.0
    lookahead_x: float | None = None        # px, original image coords
    lookahead_y: float = 0.0
    x_ratio: float | None = None            # lookahead_x / width
    edge_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))  # orig coords
    curve: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))        # orig coords
    stripes: list = field(default_factory=list)
    selected: Stripe | None = None
    reason: str = ""
    mask: np.ndarray | None = None
    width: int = 0
    height: int = 0
    scale: float = 1.0                      # orig px per processing px
    source: str = "none"                    # "left" | "right" (fallback) | "none"
    left_confidence: float = 0.0
    right_confidence: float = 0.0
    right_selected: Stripe | None = None
    right_x_ratio: float | None = None      # right boundary inside edge at lookahead
    right_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    right_curve: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    lane_width_ratio: float = 0.0
    bend: float | None = None               # corner ahead: far end of the line vs. its near-end direction,
                                            # fraction of width (negative = bends left, positive = right)


class LeftLineDetector:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._prev_poly: np.ndarray | None = None   # x(y) in processing coords
        self._prev_h = 0
        self._last_x: float | None = None   # last good edge x near the car (processing px)
        self._frames_since_good = 10**6
        self._prev_right_poly: np.ndarray | None = None
        self._lane_w: float | None = None    # learned lane width at lookahead row (fraction of W)
        self._prev_out: float | None = None  # last output x_ratio (for bumpless left<->right handoff)
        self._prev_src = "none"
        self._handoff = 0.0
        self._kernel = np.ones((3, 3), np.uint8)
        self._last_path = "none"          # how _choose picked: continuity / geometry / lone / swap / none
        self._cand_x: float | None = None  # stripe being confirmed (re-acquire), processing px
        self._cand_n = 0

    def reset(self) -> None:
        self._cand_x, self._cand_n = None, 0
        self._prev_poly = None
        self._prev_right_poly = None
        self._prev_out, self._prev_src, self._handoff = None, "none", 0.0
        self._last_x = None
        self._frames_since_good = 10**6

    # ------------------------------------------------------------------ mask
    def mask(self, hsv: np.ndarray) -> np.ndarray:
        c = self.cfg
        m = cv2.inRange(hsv, (c.h_lo, c.s_lo, c.v_lo), (c.h_hi, c.s_hi, c.v_hi))
        if c.h_wrap_lo < 180:
            m |= cv2.inRange(hsv, (c.h_wrap_lo, c.s_lo, c.v_lo), (179, c.s_hi, c.v_hi))
        return cv2.morphologyEx(m, cv2.MORPH_CLOSE, self._kernel)

    # ------------------------------------------------------------------ main
    def detect(self, bgr: np.ndarray) -> Detection:
        c = self.cfg
        H0, W0 = bgr.shape[:2]
        scale = 1.0
        if W0 > c.process_width:
            scale = W0 / c.process_width
            bgr = cv2.resize(bgr, (c.process_width, int(round(H0 / scale))), interpolation=cv2.INTER_AREA)
        H, W = bgr.shape[:2]
        det = Detection(width=W0, height=H0, scale=scale, lookahead_y=c.lookahead_ratio * H0)

        y0, y1 = int(c.roi_top_ratio * H), int(c.roi_bottom_ratio * H)
        mask = np.zeros((H, W), np.uint8)
        mask[y0:y1] = self.mask(cv2.cvtColor(bgr[y0:y1], cv2.COLOR_BGR2HSV))
        det.mask = mask

        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        stripes = []
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < c.min_component_area or h < 4:
                continue
            sub = (labels[y:y + h, x:x + w] == i)
            n_runs = int((np.diff(sub.astype(np.int8), axis=1, prepend=0) == 1).sum())
            if area / max(n_runs, 1) > c.max_blob_width_ratio * W:   # fat blob (hand, shoe): tape is a thin band
                continue
            if c.shape_filter and not self._tape_shaped(sub):
                continue
            ys, xs = np.nonzero(sub)
            ys = ys + y
            xs = xs + x
            lean = self._lean(xs, ys)
            near = ys >= ys.max() - max(2, h * 0.15)
            stripes.append(Stripe(i, int(area), (int(x), int(y), int(w), int(h)),
                                  float(xs[near].mean()), float(ys.max()), lean))
        det.stripes = stripes
        if self._prev_h != H:
            self.reset()
            self._prev_h = H
        self._frames_since_good += 1

        # ---- LEFT boundary (primary): inside edge = its RIGHT edge
        left_fit = None
        chosen, continuity = self._choose(stripes, labels, W, H)
        if chosen is not None:
            chosen.role = "LEFT"
            det.selected = chosen
            pts = self._inner_edge(labels, chosen, y0, y1, W, "left", self._prev_poly)
            left_fit = self._fit(pts, H, y0, y1, continuity)
        if left_fit is not None and c.reacquire_confirm_frames > 0:
            if self._last_path == "continuity":
                self._cand_x, self._cand_n = None, 0          # already trusted
            else:
                # newly picked (geometry / lone / swap): must hold its place before we lock on
                nx = chosen.near_x
                if self._cand_x is not None and abs(nx - self._cand_x) <= c.max_jump_ratio * W:
                    self._cand_n += 1
                else:
                    self._cand_n = 1
                self._cand_x = nx
                if self._cand_n < c.reacquire_confirm_frames:
                    det.reason = f"confirming new line {self._cand_n}/{c.reacquire_confirm_frames}"
                    chosen.role = "other"
                    det.selected = None
                    left_fit = None                            # not trusted yet: no output, no lock
                else:
                    self._cand_x, self._cand_n = None, 0
        if left_fit is not None:
            self._prev_poly = left_fit["poly"]
            if left_fit["conf"] >= c.min_confidence:
                self._frames_since_good = 0
                self._last_x = float(np.polyval(left_fit["poly"], left_fit["pts"][:, 1].max()))
        else:
            self._prev_poly = None

        # ---- RIGHT boundary (fallback reference): inside edge = its LEFT edge
        right_fit = None
        rsel, rcont = self._choose_right(stripes, labels, W, chosen, y0, y1)
        if rsel is not None:
            rpts = self._inner_edge(labels, rsel, y0, y1, W, "right", self._prev_right_poly)
            right_fit = self._fit(rpts, H, y0, y1, rcont)
            if right_fit is not None:
                rsel.role = "RIGHT_REF"
                det.right_selected = rsel
        self._prev_right_poly = right_fit["poly"] if right_fit is not None else None

        left_ok = left_fit is not None and left_fit["conf"] >= c.min_confidence
        right_ok = right_fit is not None and right_fit["conf"] >= c.min_confidence

        # learn lane width (at the lookahead row) whenever both inside edges are solid
        if left_ok and right_ok:
            wl = (right_fit["x"] - left_fit["x"]) / W
            if 0.6 * c.lane_width_ratio < wl < 1.6 * c.lane_width_ratio:
                self._lane_w = wl if self._lane_w is None else 0.8 * self._lane_w + 0.2 * wl
        det.lane_width_ratio = self._lane_w if self._lane_w is not None else c.lane_width_ratio

        if right_fit is not None:
            det.right_points = right_fit["pts"] * scale
            det.right_curve = right_fit["curve"] * scale
            det.right_x_ratio = right_fit["x"] / W
            det.right_confidence = right_fit["conf"]
        det.bend = self._bend(left_fit if left_ok else None, right_fit if right_ok else None, W)
        if left_fit is not None:
            det.edge_points = left_fit["pts"] * scale
            det.curve = left_fit["curve"] * scale
            det.left_confidence = left_fit["conf"]

        if left_ok or (left_fit is not None and not (c.use_right_fallback and right_ok)):
            # primary: left line (also reported when weak and no usable fallback)
            det.source = "left"
            det.lookahead_x = left_fit["x"] * scale
            det.lookahead_y = left_fit["y"] * scale
            det.x_ratio = left_fit["x"] / W
            det.confidence = left_fit["conf"]
        elif c.use_right_fallback and right_ok:
            # fallback: virtual left line = right inside edge minus lane width
            det.source = "right"
            xv = right_fit["x"] - det.lane_width_ratio * W
            det.lookahead_x = xv * scale
            det.lookahead_y = right_fit["y"] * scale
            det.x_ratio = xv / W
            det.confidence = right_fit["conf"] * c.right_fallback_conf_scale
        else:
            det.reason = ("no orange" if not stripes else
                          "edge too short" if chosen is not None else "only right boundary visible")
            return det
        det.confidence = float(np.clip(det.confidence, 0, 1))
        det.found = det.confidence >= c.min_confidence

        # bumpless handoff: when the source switches (left <-> right), start from the
        # previous output and fade the difference out, so steering doesn't kick
        if det.found:
            raw = det.x_ratio
            if self._prev_out is not None and det.source != self._prev_src:
                self._handoff = float(np.clip(self._prev_out - raw, -0.3, 0.3))
            else:
                self._handoff *= c.handoff_decay
            det.x_ratio = raw + self._handoff
            det.lookahead_x = det.x_ratio * W * scale
            self._prev_out, self._prev_src = det.x_ratio, det.source
        else:
            self._prev_out, self._prev_src, self._handoff = None, "none", 0.0
        if det.found:
            det.reason = "ok" if det.source == "left" else "left lost: using right line - lane width"
        else:
            det.reason = f"low confidence {det.confidence:.2f}"
        return det

    def _fit(self, pts, H, y0, y1, continuity):
        """Robust fit of an inside edge, evaluated at the lookahead row."""
        if len(pts) < 4:
            return None
        c = self.cfg
        poly, resid = self._robust_fit(pts, H)
        ys_pts = pts[:, 1]
        span = ys_pts.max() - ys_pts.min()
        yl = c.lookahead_ratio * H
        margin = 0.12 * H                       # extrapolate at most a little beyond the data
        yl_eval = float(np.clip(yl, ys_pts.min() - margin, ys_pts.max() + margin))
        extrap_pen = 1.0 if abs(yl_eval - yl) < 1 else 0.7
        conf = (min(1.0, len(pts) / 8.0) * min(1.0, span / (0.12 * H))
                * float(np.exp(-resid / 4.0)) * continuity * extrap_pen)
        cy = np.linspace(max(y0, ys_pts.min() - margin), min(y1 - 1, ys_pts.max() + margin), 24)
        x_look = float(np.polyval(poly, yl_eval))
        if c.lookahead_from_points:
            # follow the line where it really is: median of measured edge points near the
            # lookahead row. A quadratic can swing the wrong way on a bend (seen on the stage:
            # a left curve read as "line on the right" -> steered right).
            band = c.lookahead_band_ratio * H
            near = np.abs(ys_pts - yl_eval) <= band
            if near.sum() >= 3:
                x_look = float(np.median(pts[near, 0]))
        return {"poly": poly, "pts": pts, "x": x_look, "y": yl_eval,
                "conf": float(np.clip(conf, 0, 1)),
                "curve": np.stack([np.polyval(poly, cy), cy], 1)}

    @staticmethod
    def _bend(left_fit, right_fit, W):
        """How much the line curves before the far end of what we can see.

        Perspective keeps straight lines straight, so on a straight the edge is a
        straight line in the image; a corner makes it bend. Measured as: where the
        line actually is at its far end, minus where it would be if it kept the
        direction it has near the car. Uses whichever fit spans more rows.
        """
        best = None
        for f in (left_fit, right_fit):
            if f is None or len(f["poly"]) < 3:
                continue
            ys = f["pts"][:, 1]
            span = float(np.ptp(ys))
            if best is None or span > best[1]:
                best = (f, span)
        if best is None or best[1] < 8:
            return None
        f, _ = best
        poly = f["poly"]
        ys = f["pts"][:, 1]
        y_near, y_far = float(ys.max()), float(ys.min())
        slope_near = 2 * poly[0] * y_near + poly[1]
        x_near = np.polyval(poly, y_near)
        x_far_straight = x_near + slope_near * (y_far - y_near)
        return float((np.polyval(poly, y_far) - x_far_straight) / W)

    def _tape_shaped(self, sub: np.ndarray) -> bool:
        """Rotation-invariant tape test (from UCSD Team 10): tape fills its tilted
        bounding box and has a smooth outline; wood-grain fragments and glare don't."""
        c = self.cfg
        cnts, _ = cv2.findContours(sub.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return False
        cnt = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area <= 0:
            return True  # too thin for a contour area (1-px line): leave it to the other filters
        (_, _), (rw, rh), _ = cv2.minAreaRect(cnt)
        long_s, short_s = max(rw, rh), max(min(rw, rh), 1e-6)
        if area / (long_s * short_s) < c.min_rect_fill:
            return False
        hull = cv2.contourArea(cv2.convexHull(cnt))
        if hull > 0 and area / hull < c.min_solidity:
            return False
        return long_s / short_s <= c.max_aspect

    # ------------------------------------------------------------- helpers
    @staticmethod
    def _lean(xs: np.ndarray, ys: np.ndarray) -> float:
        """dx per pixel moving UP, from the lower 60% of the stripe (perspective dominates near the car)."""
        lo = ys >= ys.min() + 0.4 * (ys.max() - ys.min())
        yy, xx = ys[lo], xs[lo]
        if np.ptp(yy) < 3:
            return 0.0
        rows = np.unique(yy)
        cx = np.array([xx[yy == r].mean() for r in rows])
        slope = np.polyfit(rows, cx, 1)[0]          # dx/dy (y down)
        return float(-slope)

    def _choose(self, stripes, labels, W, H):
        c = self.cfg
        for s in stripes:
            s.role = "other"
        # invariant: a stripe in the far-right zone that leans left going up is the
        # right boundary and is never eligible, whatever continuity says
        eligible = [s for s in stripes
                    if not (s.near_x > c.reject_right_ratio * W and s.lean < -c.lean_tolerance)]
        for s in stripes:
            if s not in eligible:
                s.role = "RIGHT"
        if not eligible:
            self._last_path = "none"
            return None, 0.0

        # 1) continuity with previous fit
        if self._prev_poly is not None:
            best, best_d = None, 1e9
            for s in eligible:
                x, y, w, h = s.bbox
                rows = np.linspace(y, y + h - 1, 6).astype(int)
                d = []
                for r in rows:
                    xs = np.nonzero(labels[r] == s.label)[0]
                    if len(xs):
                        d.append(abs(xs.max() - np.polyval(self._prev_poly, r)))
                if d and np.median(d) < best_d:
                    best, best_d = s, float(np.median(d))
            if best is not None and best_d <= c.max_jump_ratio * W:
                swapped = self._lean_swap(best, eligible, W)
                if swapped is not None:
                    self._last_path = "swap"
                    self._mark_right(stripes, swapped)
                    return swapped, 0.8
                self._last_path = "continuity"
                self._mark_right(stripes, best)
                return best, 1.0 - 0.5 * best_d / (c.max_jump_ratio * W)
            # fall through to re-acquire

        # 2) geometric acquisition
        cand = sorted(eligible, key=lambda s: s.near_x - 8.0 * s.lean)   # leftmost, ties by lean
        right_seen = len(eligible) < len(stripes)    # a right boundary was positively identified
        if cand and (len(cand) >= 2 or right_seen):
            left = cand[0]
            others = cand[1:] + [s for s in stripes if s not in eligible]
            # two boundaries visible -> the leftmost is the left one, unless every stripe
            # leans left AND we have no right boundary to anchor against
            if right_seen or not (left.lean < -c.lean_tolerance
                                  and all(o.lean < -c.lean_tolerance for o in others)):
                left = self._lean_swap(left, eligible, W) or left
                self._last_path = "geometry"
                self._mark_right(stripes, left)
                return left, 0.8
        # lone stripe: accept only if it clearly leans right going up (a near-vertical
        # stripe is ambiguous), and, if we lost the line recently, only if it is not far
        # RIGHT of where the line was.
        good = [s for s in cand if s.lean > c.lean_tolerance and s.near_x < c.reject_right_ratio * W]
        if self._last_x is not None and self._frames_since_good < 60:
            good = [s for s in good if s.near_x <= self._last_x + 2 * c.max_jump_ratio * W]
        if good:
            left = min(good, key=lambda s: s.near_x)
            self._last_path = "lone"
            self._mark_right(stripes, left)
            return left, 0.75
        for s in stripes:
            s.role = "RIGHT"
        self._last_path = "none"
        return None, 0.0

    def _lean_swap(self, chosen, eligible, W):
        """Team-7-style geometry check: a stripe leaning like a RIGHT boundary can't be the
        left line if a stripe leaning like a LEFT boundary sits clearly to its left."""
        c = self.cfg
        if not c.left_lean_swap or chosen.lean >= -c.lean_tolerance:
            return None
        alts = [s for s in eligible if s is not chosen and s.lean > c.lean_tolerance
                and s.near_x < chosen.near_x - c.lean_swap_min_sep_ratio * W]
        return max(alts, key=lambda s: s.area) if alts else None

    @staticmethod
    def _mark_right(stripes, chosen):
        for s in stripes:
            if s is not chosen and s.role != "RIGHT":
                s.role = "RIGHT" if s.near_x > chosen.near_x else "other"

    def _choose_right(self, stripes, labels, W, left, y0, y1):
        """Pick the RIGHT boundary used as a fallback reference. Never the LEFT stripe.
        It must reach down into the lower 80% of the ROI: thin fragments hugging the
        horizon (wood floor/wall edges, far track segments) are not usable references."""
        c = self.cfg
        min_near_y = y0 + 0.2 * (y1 - y0)
        cands = [s for s in stripes if s is not left and s.near_y >= min_near_y
                 and (left is None or s.near_x > left.near_x + 0.10 * W)]
        if not cands:
            return None, 0.0
        if self._prev_right_poly is not None:
            best, best_d = None, 1e9
            for s in cands:
                x, y, w, h = s.bbox
                d = []
                for r in np.linspace(y, y + h - 1, 6).astype(int):
                    xs = np.nonzero(labels[r] == s.label)[0]
                    if len(xs):
                        d.append(abs(xs.min() - np.polyval(self._prev_right_poly, r)))
                if d and np.median(d) < best_d:
                    best, best_d = s, float(np.median(d))
            if best is not None and best_d <= c.max_jump_ratio * W:
                return best, 1.0 - 0.5 * best_d / (c.max_jump_ratio * W)
        if left is not None:
            # both visible: the right boundary is the nearest stripe to the right of the left one
            return min(cands, key=lambda s: s.near_x), 0.8
        # left not visible: only accept a stripe that clearly leans left going up,
        # preferring the one reaching closest to the car
        rl = [s for s in cands if s.lean < -c.lean_tolerance]
        if not rl:
            return None, 0.0
        return max(rl, key=lambda s: (s.near_y, -s.near_x)), 0.75

    def _inner_edge(self, labels, s: Stripe, y0, y1, W, side, prev_poly) -> np.ndarray:
        """Inside edge of a boundary on each scanline: the RIGHT edge of the left stripe,
        the LEFT edge of the right stripe. If a row has several runs (stripe merging with
        something), take the run whose inside edge is closest to the previous fit."""
        x, y, w, h = s.bbox
        rows = np.unique(np.linspace(max(y, y0), min(y + h, y1) - 1, self.cfg.num_scanlines).astype(int))
        pts = []
        max_w = 0.35 * W
        for r in rows:
            on = (labels[r, x:x + w] == s.label).astype(np.int8)
            if not on.any():
                continue
            d = np.diff(np.concatenate(([0], on, [0])))
            starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]  # end exclusive
            runs = [(a + x, b - 1 + x) for a, b in zip(starts, ends) if (b - a) <= max_w]
            if not runs:
                continue
            edge = (lambda ab: ab[1] + 0.5) if side == "left" else (lambda ab: ab[0] - 0.5)
            if prev_poly is not None and len(runs) > 1:
                p = np.polyval(prev_poly, r)
                ab = min(runs, key=lambda ab: abs(edge(ab) - p))
            else:
                ab = runs[0]                         # leftmost run
            pts.append((edge(ab), r))
        return np.array(pts, float).reshape(-1, 2)

    @staticmethod
    def _robust_fit(pts: np.ndarray, H: int):
        ys, xs = pts[:, 1], pts[:, 0]
        deg = 2 if (len(pts) >= 8 and np.ptp(ys) > 0.10 * H) else 1
        keep = np.ones(len(pts), bool)
        poly = np.polyfit(ys, xs, deg)
        for _ in range(2):
            r = np.abs(np.polyval(poly, ys) - xs)
            mad = np.median(r[keep]) + 1e-6
            keep = r <= max(2.0, 3.0 * 1.4826 * mad)
            if keep.sum() < deg + 2:
                break
            poly = np.polyfit(ys[keep], xs[keep], deg)
        resid = float(np.sqrt(np.mean((np.polyval(poly, ys[keep]) - xs[keep]) ** 2)))
        return poly, resid
