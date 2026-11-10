"""The simulation loop: camera timing, control loop, actuators, physics, metrics.

Timing (like the real car):
  camera captures a frame every 1/camera_fps (pose at that instant)
  every control tick (1/control_hz) the controller uses the NEWEST captured frame,
  and its command is sent processing_s later; the actuators add actuator_delay_s.
Physics runs at 4x the control rate.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .camera import Camera, measure_edges, target_offset
from .feasibility import corner_report
from .params import Profile
from .track import Track
from .vehicle import Extensions, Vehicle


@dataclass
class Metrics:
    seconds: float = 0.0
    lap_times: list = field(default_factory=list)
    off_track: int = 0
    off_wide: int = 0                                     # ... of which over the RIGHT (outside) tape
    off_inside: int = 0                                   # ... of which over the LEFT (inside) tape
    inside_ticks: int = 0                                 # ticks with the body over the inside tape's outer edge
    off_track_t: list = field(default_factory=list)
    err_samples: list = field(default_factory=list)      # |offset - ride offset|, m
    min_clear_left: float = float("inf")                 # closest the car centre came to the left tape edge, m
    min_body_clear_left: float = float("inf")
    min_body_clear_right: float = float("inf")
    slip_ticks: int = 0
    sat_ticks: int = 0
    lost_ticks: int = 0
    ticks: int = 0
    speed_sum: float = 0.0

    def summary(self):
        e = np.array(self.err_samples) if self.err_samples else np.zeros(1)
        laps = self.lap_times[1:] or self.lap_times
        return {
            "seconds": round(self.seconds, 1), "laps": len(self.lap_times),
            "lap_s": round(float(np.mean(laps)), 2) if laps else None,
            "best_lap_s": round(min(self.lap_times), 2) if self.lap_times else None,
            "off_track": self.off_track, "off_wide": self.off_wide, "off_inside": self.off_inside,
            "inside_pct": round(100 * self.inside_ticks / max(1, self.ticks), 1),
            "err_mean_cm": round(100 * float(np.mean(e)), 1), "err_p95_cm": round(100 * float(np.percentile(e, 95)), 1),
            "min_clear_left_cm": round(100 * self.min_clear_left, 1) if math.isfinite(self.min_clear_left) else None,
            "min_body_clear_left_cm": round(100 * self.min_body_clear_left, 1) if math.isfinite(self.min_body_clear_left) else None,
            "min_body_clear_right_cm": round(100 * self.min_body_clear_right, 1) if math.isfinite(self.min_body_clear_right) else None,
            "slip_pct": round(100 * self.slip_ticks / max(1, self.ticks), 1),
            "steer_sat_pct": round(100 * self.sat_ticks / max(1, self.ticks), 1),
            "lost_pct": round(100 * self.lost_ticks / max(1, self.ticks), 1),
            "mean_speed": round(self.speed_sum / max(1, self.ticks), 2),
        }


def make_track(prof):
    """The traced track polyline when the profile names one (sim/tracks/*.csv, columns x_m,y_m of the
    LEFT/inner line in driving order), otherwise the stadium from the profile's straight/radius."""
    entry = getattr(prof, "_e", {}).get("track_polyline")
    path = entry.get("value") if isinstance(entry, dict) else None
    if path:
        import csv
        import os
        full = path if os.path.isabs(path) else os.path.join(os.path.dirname(os.path.abspath(__file__)), path)
        with open(full) as f:
            pts = [(float(r["x_m"]), float(r["y_m"])) for r in csv.DictReader(f)]
        return Track.from_points(pts, prof)
    return Track.stadium(prof)


class World:
    def __init__(self, prof: Profile | None = None, ext: Extensions | None = None, vision="geometric",
                 seed=0, target_ratio=None, row_ratio=None, camera_fps=None, processing_s=None,
                 inside_tolerance_m=0.0):
        self.p = prof or Profile.load()
        self.ext = ext or Extensions()
        self.vision = vision
        # How far the body may go over the INSIDE (left) tape's outer edge before it counts as off
        # track. 0 = any crossing (strict). The race only punishes leaving the track, and riding
        # over the inside line is accepted, so the race tools pass a tolerance here.
        self.inside_tolerance_m = float(inside_tolerance_m)
        self.rng = np.random.default_rng(seed)
        self.track = make_track(self.p)
        self.cam = Camera(self.p)
        self.veh = Vehicle(self.p, self.ext)
        self.camera_fps = camera_fps or self.p.camera_fps
        self.processing_s = self.p.processing_s if processing_s is None else processing_s
        self.dt_ctrl = 1.0 / self.p.control_hz
        self.target_ratio = target_ratio
        self.row_ratio = row_ratio
        self._lane_ratio = 0.47
        self.want_edges = True        # measure edge pixels on several rows (corner geometry); costs ~12 row lookups
        self.edges = None
        self.edges_source = "left"    # which line self.edges belongs to
        self._edges_next = None
        self.metrics = Metrics()
        self._detector = None
        self.reset()

    # ------------------------------------------------------------ setup
    def ride_offset(self):
        """Distance from the left tape's inside edge where holding the target puts the car."""
        return target_offset(self.cam, None, self.target_ratio, self.row_ratio, self.p.tape_width_m)

    def reset(self):
        off = self.ride_offset() if self.target_ratio is not None else 0.3
        self._ride = off
        # start 0.3 m along the left line, on the ride line, pointing along the track
        i, _ = self.track.nearest(*(self.track.left[0] + 0.3 * self.track.tangent[0]))
        x, y = self.track.left[i] + (self.p.tape_width_m / 2 + off) * self.track.normal_right[i]
        self.veh.reset(float(x), float(y), float(math.atan2(self.track.tangent[i][1], self.track.tangent[i][0])))
        self.t = 0.0
        self._next_capture = 0.0
        self._frame = None            # (capture_time, measurement or image)
        self._pending = []            # (send_time, steer, thr)
        self._progress = 0.0
        self._last_s = self.track.nearest(self.veh.s.x, self.veh.s.y)[1]
        self._lap_t0 = None
        self._lap_mark = None

    def feasibility(self, speed_mps):
        return corner_report(self.p, self.track, self._ride, speed_mps)

    # ------------------------------------------------------------ sensing
    def _capture(self):
        s = self.veh.s
        pose = (s.x, s.y, s.yaw)
        if self.vision == "rendered":
            self._frame = (self.t, self.cam.render(pose, self.track, self.rng))
        else:
            x, src, both, seen = self.cam.measure_row(pose, self.track, self.row_ratio, self._lane_ratio)
            self._edges_next = measure_edges(self.cam, pose, self.track) if self.want_edges else None
            if seen is not None:
                self._lane_ratio = 0.8 * self._lane_ratio + 0.2 * seen
            if x is not None:
                x += self.rng.normal(0, self.p.meas_noise)
            if self.rng.random() < self.p.meas_dropout:
                x, src, both = None, None, False
            self._frame = (self.t, (x, src, both, self._edges_next))

    def observe(self):
        """What the controller gets this tick: x_ratio, found, both, frame_age, motion score, image."""
        if self._frame is None:
            return None, False, False, float("inf"), 0.9, None
        cap_t, data = self._frame
        age = self.t - cap_t
        motion = 0.9 + 8.0 * self.veh.s.v   # camera-motion score proxy (ASSUMPTION: calibrate vs real motion score)
        if self.vision == "rendered":
            return None, False, False, age, motion, data
        x, src, both, edges = data
        # the edge pixels of the same (delayed) frame, for corner geometry; source says which line
        self.edges = None if edges is None else (edges[0] if src == "left" else edges[1] if src == "right" else None)
        self.edges_source = src if src in ("left", "right") else "left"
        return x, x is not None, both, age, motion, None

    # ------------------------------------------------------------ stepping
    def tick(self, steer_cmd, thr_cmd):
        """Advance one control period with this command (sent after processing_s)."""
        self._pending.append((self.t + self.processing_s, steer_cmd, thr_cmd))
        n = 4
        h = self.dt_ctrl / n
        for _ in range(n):
            while self._pending and self._pending[0][0] <= self.t + 1e-9:
                _, sc, tc = self._pending.pop(0)
                self.veh.command(sc, tc)
            if self.t + 1e-9 >= self._next_capture:
                self._capture()
                self._next_capture += 1.0 / self.camera_fps
            self.veh.step(h)
            self.t += h
        self._account(steer_cmd)

    def _account(self, steer_cmd):
        s, m = self.veh.s, self.metrics
        m.ticks += 1
        m.seconds = self.t
        m.speed_sum += s.v
        m.slip_ticks += int(s.slipping)
        m.sat_ticks += int(abs(steer_cmd) >= 0.95 * 0.8)
        off = self.track.offset_from_left(s.x, s.y)
        m.err_samples.append(abs(off - self._ride))
        m.min_clear_left = min(m.min_clear_left, off)
        body_offsets = self.track.offsets_from_left(self.veh.footprint())
        body_clear_left = float(np.min(body_offsets) + self.p.tape_width_m)
        body_clear_right = float(self.p.lane_width_m + self.p.tape_width_m - np.max(body_offsets))
        m.min_body_clear_left = min(m.min_body_clear_left, body_clear_left)
        m.min_body_clear_right = min(m.min_body_clear_right, body_clear_right)
        # laps
        _, s_now = self.track.nearest(s.x, s.y)
        d = s_now - self._last_s
        L = self.track.length
        d += L if d < -L / 2 else (-L if d > L / 2 else 0)
        self._progress += d
        self._last_s = s_now
        if self._lap_t0 is None and s.v > 0.1:
            self._lap_t0, self._lap_mark = self.t, self._progress + L
        elif self._lap_t0 is not None and self._progress >= self._lap_mark:
            m.lap_times.append(self.t - self._lap_t0)
            self._lap_t0, self._lap_mark = self.t, self._lap_mark + L
        # Off track when any chassis corner crosses either tape's outer edge (the inside one may be
        # crossed by inside_tolerance_m).
        m.inside_ticks += int(body_clear_left < 0.0)
        if body_clear_left < -self.inside_tolerance_m or body_clear_right < 0.0:
            m.off_track += 1
            if body_clear_right < 0.0:
                m.off_wide += 1
            else:
                m.off_inside += 1
            m.off_track_t.append(round(self.t, 2))
            t = self.t
            self.reset()
            self.t = t
            self._next_capture = t
