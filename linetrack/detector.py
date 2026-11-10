"""Lane-boundary tracker for the two orange tape lines.

The car follows the LEFT boundary's inside edge. The right boundary is tracked as an
equal: it is the backup measurement (right edge minus the learned lane width) and it
is what tells the two lines apart.

Per frame (at <= 240 px width; the live Donkeycar stream is 240x192):

  1. Mask.   BGR -> HSV -> strict orange threshold, then grown a few pixels into
             "nearly orange" pixels. Glare and lighting push parts of the tape just
             outside the strict range; growing fills those holes without admitting
             look-alike surfaces (wood) that have no strictly-orange pixels nearby.
  2. Chains. Orange runs are found on every sampled scanline and linked from row to
             row into line candidates. Small gaps are bridged, so a line broken into
             fragments is still one candidate (connected components made each
             fragment a separate "stripe", and a stray fragment could win).
  3. Tracks. Each boundary keeps the fit of its inside edge from the previous frames.
             A candidate lying on a boundary's previous fit IS that boundary. Identity
             comes from history first; geometry is only used to start a track:
               - two candidates one lane width apart: the left one is LEFT;
               - one candidate next to a tracked boundary, one lane width away;
               - one candidate alone: the side of the car it passes on, and it must
                 hold for a few frames before it is trusted.
  4. Output. LEFT inside edge at the lookahead row; if LEFT is not usable, RIGHT inside
             edge minus the lane width (learned while both are visible), with the
             switch-over offset faded out so steering does not kick.

Seen from inside the lane: LEFT leans right going up the image and passes left of the
car; RIGHT leans left and passes right of the car.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from .config import Config


@dataclass
class Stripe:
    """One line candidate: a chain of orange runs linked across scanlines."""
    label: int
    area: int
    bbox: tuple            # x, y, w, h (processing coords)
    near_x: float          # centre x at its lowest (nearest) rows
    near_y: float
    lean: float            # dx/d(up): + = leans right going up (left-boundary-like)
    role: str = "other"    # "LEFT" (tracked), "RIGHT_REF" (right boundary in use),
                           # "RIGHT" (right of the tracked line, not used), "other"
    ys: np.ndarray = field(default_factory=lambda: np.zeros(0), repr=False)   # scanline rows, ascending
    xl: np.ndarray = field(default_factory=lambda: np.zeros(0), repr=False)   # left edge per row
    xr: np.ndarray = field(default_factory=lambda: np.zeros(0), repr=False)   # right edge per row
    ok_l: np.ndarray = field(default_factory=lambda: np.zeros(0, bool), repr=False)  # left edge usable
    ok_r: np.ndarray = field(default_factory=lambda: np.zeros(0, bool), repr=False)  # right edge usable
    x_bottom: float = 0.0  # centre line extended to the bottom row (which side of the car it passes)
    substantial: bool = False

    def edge(self, side: str):
        """Inside-edge points when this is the `side` boundary: (xs, ys, usable)."""
        return (self.xr, self.ok_r) if side == "L" else (self.xl, self.ok_l)

    def x_at(self, y: float, side: str) -> float:
        """Inside edge at row y (straight-line extension outside the rows it covers)."""
        xs, ok = self.edge(side)
        if ok.sum() < 2:
            xs, ok = (self.xl + self.xr) / 2, np.ones(len(self.ys), bool)
        ys, xs = self.ys[ok], xs[ok]
        if ys[0] <= y <= ys[-1]:
            return float(np.interp(y, ys, xs))
        if np.ptp(ys) < 1:
            return float(xs.mean())
        a, b = np.polyfit(ys, xs, 1)
        return float(a * y + b)


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
    sure: bool = False                      # the line in use was seen next to the other boundary since it
                                            # was picked up, so it is known which of the two it is
    bend: float | None = None               # corner ahead: far end of the line vs. its near-end direction,
                                            # fraction of width (negative = bends left, positive = right)


class _Track:
    """One boundary's inside edge, remembered between frames."""

    def __init__(self):
        self.poly: np.ndarray | None = None
        self.y_lo = self.y_hi = 0.0
        self.miss = 0          # frames since a candidate was matched
        self.hits = 0          # consecutive frames matched
        self.near_x = 0.0
        self.paired = False    # seen together with the other boundary since this track began

    @property
    def live(self) -> bool:
        return self.poly is not None

    def clear(self):
        self.poly, self.miss, self.hits, self.paired = None, 0, 0, False

    def x(self, y):
        """Fit inside the rows it was measured on, straight-line extension outside
        (a quadratic swings wildly when extrapolated)."""
        y = np.asarray(y, float)
        yc = np.clip(y, self.y_lo, self.y_hi)
        return np.polyval(self.poly, yc) + np.polyval(np.polyder(self.poly), yc) * (y - yc)


class LeftLineDetector:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._prev_h = 0
        self._trk = {"L": _Track(), "R": _Track()}
        self._lane_w: float | None = None    # learned lane width at lookahead row (fraction of W)
        self._prev_out: float | None = None  # last output x_ratio (for bumpless left<->right handoff)
        self._prev_src = "none"
        self._handoff = 0.0
        self._pending = None                 # lone candidate being confirmed: (side, near_x, frames)
        self._flip_n = 0                     # frames in a row the tracked line looked like the other boundary
        self._frame = 0
        self._last_seen = {}                 # side -> (frame, near_x) of the last trusted fit
        self._k3 = np.ones((3, 3), np.uint8)

    def reset(self) -> None:
        for t in self._trk.values():
            t.clear()
        self._prev_out, self._prev_src, self._handoff = None, "none", 0.0
        self._pending, self._flip_n = None, 0
        self._last_seen = {}

    # compatibility with tools that peek at the previous left fit
    @property
    def _prev_poly(self):
        return self._trk["L"].poly

    # ------------------------------------------------------------------ mask
    def mask(self, hsv: np.ndarray) -> np.ndarray:
        c = self.cfg
        m = cv2.inRange(hsv, (c.h_lo, c.s_lo, c.v_lo), (c.h_hi, c.s_hi, c.v_hi))
        if c.h_wrap_lo < 180:
            m |= cv2.inRange(hsv, (c.h_wrap_lo, c.s_lo, c.v_lo), (179, c.s_hi, c.v_hi))
        if c.mask_grow_px > 0 and m.any():
            # nearly-orange pixels count only within a few px of strictly-orange ones
            s_lo, v_lo = max(0, c.s_lo - c.weak_s_margin), max(0, c.v_lo - c.weak_v_margin)
            weak = cv2.inRange(hsv, (c.h_lo, s_lo, v_lo), (min(179, c.h_hi + c.weak_h_margin), c.s_hi, c.v_hi))
            if c.h_wrap_lo < 180:
                weak |= cv2.inRange(hsv, (max(0, c.h_wrap_lo - c.weak_h_margin), s_lo, v_lo), (179, c.s_hi, c.v_hi))
            k = np.ones((2 * c.mask_grow_px + 1,) * 2, np.uint8)
            for _ in range(2):
                m |= weak & cv2.dilate(m, k)
        return cv2.morphologyEx(m, cv2.MORPH_CLOSE, self._k3)

    # ---------------------------------------------------------------- chains
    def _chains(self, mask: np.ndarray, y0: int, y1: int, W: int, H: int) -> list[Stripe]:
        """Link orange runs on sampled scanlines (bottom to top) into line candidates."""
        c = self.cfg
        step = max(1, int(round((y1 - y0) / max(c.num_scanlines, 1))))
        rows = np.arange(y0, y1, step)
        if not len(rows):
            return []
        pad = np.zeros((len(rows), W + 2), np.int8)
        pad[:, 1:-1] = mask[rows] > 0
        d = np.diff(pad, axis=1)
        rs, xs = np.nonzero(d == 1)
        _, xe = np.nonzero(d == -1)               # exclusive end; same order as the starts
        if not len(rs):
            return []
        # runs on one row separated by a pixel-sized hole are one run
        if len(rs) > 1:
            join = (rs[1:] == rs[:-1]) & (xs[1:] - xe[:-1] <= c.run_merge_gap_px)
            keep_s = np.concatenate(([True], ~join))
            keep_e = np.concatenate((~join, [True]))
            rs, xs, xe = rs[keep_s], xs[keep_s], xe[keep_e]
        bounds = np.searchsorted(rs, np.arange(len(rows) + 1))
        max_gap = max(1, int(round(c.chain_gap_ratio * H / step)))
        tol = c.chain_link_px
        chains: list[dict] = []
        active: list[dict] = []
        for ri in range(len(rows) - 1, -1, -1):
            a, b = bounds[ri], bounds[ri + 1]
            active = [ch for ch in active if ch["r"][-1] - ri <= max_gap + 1]
            if a == b:
                continue
            pairs = []
            for j in range(a, b):
                s, e = int(xs[j]), int(xe[j])
                for ci, ch in enumerate(active):
                    gap = ch["r"][-1] - ri
                    shift = ch["slope"] * gap
                    ov = min(e, ch["e"][-1] + shift + tol * gap) - max(s, ch["s"][-1] + shift - tol * gap)
                    if ov > 0:
                        pairs.append((ov - 2.0 * (gap - 1), j, ci))
            pairs.sort(reverse=True)
            used_run, used_chain = set(), set()
            for _, j, ci in pairs:
                if j in used_run or ci in used_chain:
                    continue
                used_run.add(j)
                used_chain.add(ci)
                ch = active[ci]
                gap = ch["r"][-1] - ri
                cx_new, cx_old = (xs[j] + xe[j]) / 2, (ch["s"][-1] + ch["e"][-1]) / 2
                ch["slope"] = 0.5 * ch["slope"] + 0.5 * float(np.clip((cx_new - cx_old) / gap, -12, 12))
                ch["r"].append(ri)
                ch["s"].append(int(xs[j]))
                ch["e"].append(int(xe[j]))
            for j in range(a, b):
                if j not in used_run:
                    ch = {"r": [ri], "s": [int(xs[j])], "e": [int(xe[j])], "slope": 0.0}
                    chains.append(ch)
                    active.append(ch)

        out = []
        min_near_y = y0 + 0.2 * (y1 - y0)
        for i, ch in enumerate(chains):
            if len(ch["r"]) < 2:
                continue
            ys = rows[np.array(ch["r"][::-1])].astype(float)
            s = np.array(ch["s"][::-1], float)
            e = np.array(ch["e"][::-1], float)
            wd = e - s
            area = int(wd.sum() * step)
            h = ys[-1] - ys[0] + step
            if area < c.min_component_area or h < 4:
                continue
            cx = (s + e) / 2
            slope_all = np.polyfit(ys, cx, 1)[0] if np.ptp(ys) >= 3 else 0.0
            if np.median(wd) / np.hypot(1.0, slope_all) > c.max_blob_width_ratio * W:
                continue                           # fat blob (hand, shoe): tape is a thin band
            lo = ys >= ys[0] + 0.4 * (ys[-1] - ys[0])
            if np.ptp(ys[lo]) >= 3:
                a1, b1 = np.polyfit(ys[lo], cx[lo], 1)
                lean, x_bottom = float(-a1), float(a1 * (H - 1) + b1)
            else:
                lean, x_bottom = 0.0, float(cx[-1])
            near = ys >= ys[-1] - max(2, h * 0.15)
            narrow = wd <= c.max_run_width_ratio * W
            if len(ys) >= 6 and c.max_edge_roughness_px > 0:
                # tape has smooth edges; look-alike surfaces (wood grain, skin) are ragged
                rough = max(float(np.median(np.abs(np.diff(s, 2)))), float(np.median(np.abs(np.diff(e, 2)))))
                if rough > c.max_edge_roughness_px:
                    continue
            st = Stripe(i + 1, area, (int(s.min()), int(ys[0]), int(e.max() - s.min()), int(h)),
                        float(cx[near].mean()), float(ys[-1]), lean,
                        ys=ys, xl=s - 0.5, xr=e - 0.5,
                        ok_l=narrow, ok_r=narrow, x_bottom=x_bottom)
            st.substantial = bool(h >= c.acquire_min_span_ratio * H and st.near_y >= min_near_y)
            out.append(st)
        return out

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

        if self._prev_h != H:
            self.reset()
            self._prev_h = H
        self._frame += 1

        stripes = self._chains(mask, y0, y1, W, H)
        det.stripes = stripes
        picked, how = self._assign(stripes, W, H)

        fits = {}
        for side in ("L", "R"):
            trk, group = self._trk[side], picked[side]
            fit = None
            if group:
                pts = np.concatenate([np.stack([g.edge(side)[0][g.edge(side)[1]], g.ys[g.edge(side)[1]]], 1)
                                      for g in group])
                fit = self._fit(pts, H, y0, y1, how[side])
            if fit is not None:
                trk.poly = fit["poly"]
                trk.y_lo, trk.y_hi = float(fit["pts"][:, 1].min()), float(fit["pts"][:, 1].max())
                trk.miss, trk.hits, trk.near_x = 0, trk.hits + 1, group[0].near_x
                fits[side] = fit
                if fit["conf"] >= c.min_confidence:
                    self._last_seen[side] = (self._frame, group[0].near_x)
            elif trk.live:
                trk.miss += 1
                trk.hits = 0
                if trk.miss > c.track_memory_frames:
                    trk.clear()
        left_fit, right_fit = fits.get("L"), fits.get("R")
        chosen = picked["L"][0] if left_fit is not None else None
        rsel = picked["R"][0] if right_fit is not None else None
        for s in stripes:
            s.role = "other"
        anchor = chosen or rsel
        if anchor is not None:
            for s in stripes:
                if s.near_x > anchor.near_x:
                    s.role = "RIGHT"
        for g in picked["L"] if left_fit is not None else ():
            g.role = "LEFT"
        for g in picked["R"] if right_fit is not None else ():
            g.role = "RIGHT_REF"
        det.selected, det.right_selected = chosen, rsel

        left_ok = left_fit is not None and left_fit["conf"] >= c.min_confidence
        right_ok = right_fit is not None and right_fit["conf"] >= c.min_confidence

        # learn lane width (at the lookahead row) whenever both inside edges are solid
        if left_ok and right_ok:
            self._trk["L"].paired = self._trk["R"].paired = True
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
            self._prev_out, self._prev_src, self._handoff = None, "none", 0.0
            det.reason = ("no orange" if not stripes else
                          how["why"] if how["why"] else
                          "only right boundary visible" if right_fit is not None or picked["R"] else
                          "edge too short")
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
            det.sure = self._trk["L" if det.source == "left" else "R"].paired
            det.reason = "ok" if det.source == "left" else "left lost: using right line - lane width"
        else:
            self._prev_out, self._prev_src, self._handoff = None, "none", 0.0
            det.reason = f"low confidence {det.confidence:.2f}"
        return det

    # ------------------------------------------------------------ assignment
    def _expected_width(self, y: float, W: int, H: int) -> float:
        """Lane width in px on image row y (grows linearly below the horizon)."""
        c = self.cfg
        lane = (self._lane_w if self._lane_w is not None else c.lane_width_ratio) * W
        yh, yl = c.horizon_ratio * H, c.lookahead_ratio * H
        return lane * max(0.15, (y - yh) / max(yl - yh, 1.0))

    def _gap(self, a: Stripe, b: Stripe, W: int, H: int) -> float:
        """(b's left edge - a's right edge) / expected lane width, on a row both cover
        (or midway between them). ~1 when a is LEFT and b is RIGHT of one lane."""
        lo, hi = max(a.ys[0], b.ys[0]), min(a.ys[-1], b.ys[-1])
        y = float(np.clip(self.cfg.lookahead_ratio * H, lo, hi)) if lo <= hi else (lo + hi) / 2
        return (b.x_at(y, "R") - a.x_at(y, "L")) / self._expected_width(y, W, H)

    def _is_lane(self, a: Stripe, b: Stripe, W: int, H: int) -> bool:
        c = self.cfg
        return c.lane_min_ratio <= self._gap(a, b, W, H) <= c.lane_max_ratio

    def _vetoed(self, s: Stripe, side: str, W: int) -> bool:
        """A stripe in the far-right zone leaning left is the right boundary, never LEFT
        (and the mirror image for RIGHT), whatever history says."""
        c = self.cfg
        if side == "L":
            return s.near_x > c.reject_right_ratio * W and s.lean < -c.lean_tolerance
        return s.near_x < (1 - c.reject_right_ratio) * W and s.lean > c.lean_tolerance

    def _track_dist(self, s: Stripe, side: str, H: int) -> float:
        trk = self._trk[side]
        xs, ok = s.edge(side)
        if ok.sum() < 2:
            return float("inf")
        ys = s.ys[ok]
        m = 0.08 * H
        sel = (ys >= trk.y_lo - m) & (ys <= trk.y_hi + m)
        if sel.sum() < 2:
            sel = np.ones(len(ys), bool)
        return float(np.median(np.abs(xs[ok][sel] - trk.x(ys[sel]))))

    def _lone_side(self, s: Stripe, W: int) -> str | None:
        """Which boundary a single visible line is, from the side of the car it passes on."""
        c = self.cfg
        m = c.lone_center_margin * W
        if s.x_bottom < W / 2 - m and s.lean > -c.lean_tolerance:
            side = "L"
        elif s.x_bottom > W / 2 + m and s.lean < c.lean_tolerance:
            side = "R"
        elif s.lean > c.lean_tolerance and s.near_x < c.reject_right_ratio * W:
            side = "L"
        elif s.lean < -c.lean_tolerance and s.near_x > (1 - c.reject_right_ratio) * W:
            side = "R"
        else:
            side = None
        # a line we were tracking moments ago keeps its name if it reappears where it was
        best = None
        for sd, (f, x) in self._last_seen.items():
            if self._frame - f <= c.side_memory_frames and abs(s.near_x - x) <= 2 * c.max_jump_ratio * W:
                if best is None or abs(s.near_x - x) < best[1]:
                    best = (sd, abs(s.near_x - x))
        if best is not None:
            side = best[0]
        elif side == "L" and "L" in self._last_seen and self._frame - self._last_seen["L"][0] <= c.side_memory_frames \
                and s.near_x > self._last_seen["L"][1] + 2 * c.max_jump_ratio * W:
            side = None       # the left line can't have jumped far to the right
        if side is not None and self._vetoed(s, side, W):
            return None
        return side

    def _assign(self, stripes: list[Stripe], W: int, H: int):
        """Decide which candidates are LEFT and RIGHT this frame.
        Returns ({"L": [stripes], "R": [stripes]}, {"L": continuity, "R": continuity, "why": reason})."""
        c = self.cfg
        picked = {"L": [], "R": []}
        how = {"L": 0.0, "R": 0.0, "why": ""}
        if not stripes:
            self._pending, self._flip_n = None, 0
            return picked, how
        gate = c.max_jump_ratio * W

        # 1) history: candidates lying on a boundary's previous fit
        cand = []
        for side in ("L", "R"):
            trk = self._trk[side]
            if not trk.live:
                continue
            g = gate * min(2.0, 1.0 + 0.25 * trk.miss)
            for s in stripes:
                d = self._track_dist(s, side, H)
                if d <= g and not self._vetoed(s, side, W):
                    cand.append((d / g, d, side, s))
        cand.sort(key=lambda t: t[0])
        taken = set()
        for rel, d, side, s in cand:
            if id(s) in taken:
                continue
            group = picked[side]
            if group:
                # more fragments of the same line: must be close and cover other rows
                rows_have = np.concatenate([g.ys for g in group])
                if rel > 0.5 or np.isin(s.ys, rows_have).mean() > 0.3:
                    continue
            else:
                how[side] = 1.0 - 0.5 * rel
            group.append(s)
            taken.add(id(s))

        # both tracked but in the wrong order: keep the longer-lived track
        if picked["L"] and picked["R"] and self._gap(picked["L"][0], picked["R"][0], W, H) < 0.3:
            drop = "L" if self._trk["L"].hits < self._trk["R"].hits else "R"
            for s in picked[drop]:
                taken.discard(id(s))
            picked[drop] = []
            self._trk[drop].clear()

        free = [s for s in stripes if id(s) not in taken and s.substantial]
        free.sort(key=lambda s: -s.area)

        # 2) a tracked line may turn out to be the other boundary, or not a boundary at all,
        #    once more lines come into view. The lane is two ADJACENT lines around the car.
        L0 = picked["L"][0] if picked["L"] else None
        R0 = picked["R"][0] if picked["R"] else None
        lane = lambda a, b: c.lane_min_ratio <= self._gap(a, b, W, H) <= c.lane_max_ratio  # noqa: E731
        if L0 is not None and R0 is not None:
            # a line between the two that makes a lane with LEFT is the real RIGHT
            g_lr = self._gap(L0, R0, W, H)
            mid = [(self._gap(L0, s, W, H), s) for s in free]
            mid = [(g, s) for g, s in mid if c.lane_min_ratio <= g < g_lr - 0.2 and not self._vetoed(s, "R", W)]
            if mid:
                picked["R"], how["R"] = [min(mid, key=lambda t: t[0])[1]], 0.8
                R0 = picked["R"][0]
        flip = None
        if L0 is not None:
            # a line left of "LEFT", one lane away, with the car between them: "LEFT" is really RIGHT
            left = [s for s in free if lane(s, L0) and self._brackets_car(s, L0, W)
                    and not self._vetoed(s, "L", W) and not self._vetoed(L0, "R", W)]
            if left and not (R0 is not None and self._brackets_car(L0, R0, W) and W / 2 > L0.x_bottom):
                flip = ("L", max(left, key=lambda s: s.near_x))
        if flip is None and R0 is not None and L0 is None:
            right = [s for s in free if lane(R0, s) and self._brackets_car(R0, s, W)
                     and not self._vetoed(s, "R", W) and not self._vetoed(R0, "L", W) and W / 2 > R0.x_bottom]
            if right:
                flip = ("R", min(right, key=lambda s: s.near_x))
        if flip is not None:
            self._flip_n += 1
            if self._flip_n >= c.flip_confirm_frames:
                side, other = flip
                to = "R" if side == "L" else "L"
                self._trk[to], self._trk[side] = self._trk[side], _Track()
                picked[to], how[to] = picked[side], how[side]
                picked[side], how[side] = [other], 0.8
                self._flip_n = 0
                self._pending = None
                return picked, how
        else:
            self._flip_n = 0

        # 3) one boundary tracked: the other is the nearest line one lane width away
        if (L0 is None) != (R0 is None):
            if L0 is not None:
                ok = [(self._gap(L0, s, W, H), s) for s in free if not self._vetoed(s, "R", W)]
                want = "R"
            else:
                ok = [(self._gap(s, R0, W, H), s) for s in free if not self._vetoed(s, "L", W)]
                want = "L"
            ok = [(g, s) for g, s in ok if c.lane_min_ratio <= g <= c.lane_max_ratio]
            if ok and flip is None:
                picked[want], how[want] = [min(ok, key=lambda t: t[0])[1]], 0.8
            self._pending = None
            return picked, how
        if L0 is not None and R0 is not None:
            self._pending = None
            return picked, how

        # 4) nothing tracked: start from geometry. Two lines one lane apart with the car between them:
        pairs = [(a, b) for a in free for b in free if a is not b and lane(a, b) and self._brackets_car(a, b, W)
                 and not self._vetoed(a, "L", W) and not self._vetoed(b, "R", W)]
        if pairs:
            a, b = min(pairs, key=lambda p: (p[0].near_x, self._gap(p[0], p[1], W, H)))
            picked["L"], picked["R"] = [a], [b]
            how["L"] = how["R"] = 0.8
            self._pending = None
            return picked, how
        # ... or a single line, named by the side of the car it passes on, once it has held still
        for s in free:
            side = self._lone_side(s, W)
            if side is None or s.near_y < c.lookahead_ratio * H:   # must reach the row we steer from
                continue
            p = self._pending
            n = p[2] + 1 if (p is not None and p[0] == side and abs(p[1] - s.near_x) <= gate) else 1
            self._pending = (side, s.near_x, n)
            if n >= c.lone_confirm_frames:
                picked[side], how[side] = [s], 0.75
            else:
                how["why"] = f"confirming new line {n}/{c.lone_confirm_frames}"
            return picked, how
        self._pending = None
        how["why"] = "only right boundary visible" if any(s.x_bottom > W / 2 for s in free) else \
                     ("unclear which line this is" if free else "edge too short")
        return picked, how

    def _brackets_car(self, left: Stripe, right: Stripe, W: int) -> bool:
        """True when the car (bottom centre of the image) sits between the two lines."""
        m = self.cfg.bracket_margin * W
        return left.x_bottom - m < W / 2 < right.x_bottom + m

    # ------------------------------------------------------------------- fit
    def _fit(self, pts, H, y0, y1, continuity):
        """Robust fit of an inside edge, evaluated at the lookahead row."""
        if len(pts) < 4:
            return None
        c = self.cfg
        pts = pts[np.argsort(pts[:, 1])]
        poly, resid = self._robust_fit(pts, H)
        yl = c.lookahead_ratio * H
        if resid > c.local_fit_resid_px:
            # one parabola can't follow a hook-shaped line: fit only the part around the lookahead row
            win = np.abs(pts[:, 1] - np.clip(yl, pts[0, 1], pts[-1, 1])) <= c.local_fit_window_ratio * H
            if win.sum() >= 6 and np.ptp(pts[win, 1]) >= 0.08 * H:
                poly2, resid2 = self._robust_fit(pts[win], H)
                if resid2 < resid:
                    pts, poly, resid = pts[win], poly2, resid2
        ys_pts = pts[:, 1]
        span = ys_pts.max() - ys_pts.min()
        margin = 0.12 * H                       # extrapolate at most a little beyond the data
        yl_eval = float(np.clip(yl, ys_pts.min() - margin, ys_pts.max() + margin))
        extrap_pen = 1.0 if abs(yl_eval - yl) < 1 else 0.7
        conf = (min(1.0, len(pts) / 8.0) * min(1.0, span / (0.12 * H))
                * float(np.exp(-resid / 4.0)) * continuity * extrap_pen)
        cy = np.linspace(max(y0, ys_pts.min() - margin), min(y1 - 1, ys_pts.max() + margin), 24)
        # outside the measured rows, extend the fit as a straight line
        yc = np.clip(yl_eval, ys_pts.min(), ys_pts.max())
        x_look = float(np.polyval(poly, yc) + np.polyval(np.polyder(poly), yc) * (yl_eval - yc))
        if c.lookahead_from_points:
            # follow the line where it really is: median of measured edge points near the
            # lookahead row. A quadratic can swing the wrong way on a bend (seen on the stage:
            # a left curve read as "line on the right" -> steered right).
            band = c.lookahead_band_ratio * H
            near = np.abs(ys_pts - yl_eval) <= band
            if near.sum() >= 3:
                x_look = float(np.median(pts[near, 0]))
        cyc = np.clip(cy, ys_pts.min(), ys_pts.max())
        cx = np.polyval(poly, cyc) + np.polyval(np.polyder(poly), cyc) * (cy - cyc)
        return {"poly": poly, "pts": pts, "x": x_look, "y": yl_eval,
                "conf": float(np.clip(conf, 0, 1)),
                "curve": np.stack([cx, cy], 1)}

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
