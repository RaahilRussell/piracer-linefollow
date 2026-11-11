"""Corner geometry: from the tracked edge in the image to "how hard to turn" and "how fast".

The pure-PD controller only steers once the car has drifted off the line, so in a corner it
always runs a little wide and, at speed, reacts too late. A corner is really a geometry
problem:

  steering angle   delta = atan(wheelbase / R)          (kinematic bicycle; independent of speed)
  corner speed     v_max = sqrt(mu * g * R)             (tyre grip)
  braking          v_now^2 = v_corner^2 + 2 * a_brake * (distance left - v_now * delay)

This module turns the detector's inside-edge pixels into ground points (same camera model as
the simulator: equidistant fisheye, mount height/tilt/forward offset), fits the edge's
curvature, shifts it to the path the car actually rides, and answers those three formulas.

Every physical constant here is a MODEL value (sim/profile.json, most still unmeasured: see
sim/CALIBRATION.md). Numpy only, so it runs on the Pi.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

G = 9.81


@dataclass
class CarModel:
    wheelbase_m: float = 0.1741          # Waveshare spec
    steer_max_deg: float = 18.0          # wheel angle at steering command +/-1 (FITTED to recorded runs, not measured;
                                         # the pilot steers with Config.steer_max_deg, which line_config keeps equal)
    camera_fov_deg: float = 160.0        # diagonal, equidistant fisheye (ASSUMED interpretation)
    cam_height_m: float = 0.15           # ESTIMATED from real race-mat frames (lane width, curve radius; sim/profile.json), not measured
    cam_pitch_deg: float = 8.0           # ESTIMATED from real race-mat frames (tape edges parallel), not measured
    cam_fwd_m: float = 0.15              # camera ahead of the rear axle (ASSUMED)
    cam_roll_deg: float = 0.0            # camera rotated about its viewing axis (+ = image turned clockwise)
    lens_k1: float = 0.0                 # fisheye distortion beyond equidistant: theta = rho * (1 + k1 * rho^2)
    cam_cx_px: float = 0.0               # optical centre offset from the image centre, px (+ = right)
    image_w: int = 240
    image_h: int = 192
    grip_mu: float = 0.5                 # ASSUMED tyre-to-mat grip
    brake_decel: float = 1.5             # m/s^2 with throttle released (ASSUMED)
    delay_s: float = 0.08                # camera + processing + actuator, s (rough)
    # speed per throttle command (steady state): PWM = 370 + 37 * cmd, table PWM -> m/s (ASSUMED)
    speed_table: tuple = ((370, 0.0), (387, 0.0), (390, 0.35), (407, 1.8), (460, 3.5))   # 460: ASSUMED
    pwm_stop: float = 370.0
    pwm_full: float = 407.0


def project_pixels(u, v, W, H, fov_deg, pitch_deg, height_m, fwd_m, roll_deg=0.0, k1=0.0, cx_px=0.0):
    """Image pixel(s) -> ground point(s) in the car frame (X forward of the rear axle, Y left +).
    Lens: fisheye, theta = rho * (1 + k1 * rho^2) with rho = r / f; f puts the image corner at fov/2
    (k1 = 0 is equidistant). The camera is tilted down by pitch and rotated by roll about its axis.
    Shared by the car (GroundModel) and the simulator's camera so both use one projection."""
    u, v = np.asarray(u, float), np.asarray(v, float)
    half_diag = math.hypot(W / 2, H / 2)
    th_max = math.radians(fov_deg / 2)
    # f such that the corner pixel maps to fov/2 under this distortion: solve rho(1 + k1 rho^2) = th_max
    rho = th_max
    for _ in range(30):
        g, dg = rho * (1 + k1 * rho * rho) - th_max, 1 + 3 * k1 * rho * rho
        rho -= g / dg if abs(dg) > 1e-9 else 0.0
    f = half_diag / max(rho, 1e-6)
    dx, dy = u - (W / 2 + cx_px), v - H / 2
    if roll_deg:
        cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
        dx, dy = cr * dx + sr * dy, -sr * dx + cr * dy
    r = np.hypot(dx, dy) / f
    theta = r * (1 + k1 * r * r)
    az = np.arctan2(dy, dx)
    rx, ry, rz = np.sin(theta) * np.cos(az), np.sin(theta) * np.sin(az), np.cos(theta)
    phi = math.radians(pitch_deg)
    # camera axes in the car frame: x right -> -Y, y down, z forward (tilted down by pitch)
    gz = -rx * 0.0 + ry * (-math.cos(phi)) + rz * (-math.sin(phi))
    gxr = ry * (-math.sin(phi)) + rz * math.cos(phi)
    gyr = -rx
    down = (gz < -1e-6) & (theta < math.pi / 2 + 0.5)
    t = np.where(down, height_m / np.maximum(-gz, 1e-9), np.nan)
    gx = np.where(down, fwd_m + t * gxr, np.nan)
    gy = np.where(down, t * gyr, np.nan)
    return gx, gy


def ground_to_pixels(gx, gy, W, H, fov_deg, pitch_deg, height_m, fwd_m, roll_deg=0.0, k1=0.0, cx_px=0.0):
    """Ground point(s) in the car frame -> image pixel(s) (u, v): the inverse of project_pixels.
    Only used to draw planned paths on the camera overlay."""
    gx, gy = np.asarray(gx, float), np.asarray(gy, float)
    half_diag = math.hypot(W / 2, H / 2)
    th_max = math.radians(fov_deg / 2)
    rho_max = th_max
    for _ in range(30):
        g, dg = rho_max * (1 + k1 * rho_max * rho_max) - th_max, 1 + 3 * k1 * rho_max * rho_max
        rho_max -= g / dg if abs(dg) > 1e-9 else 0.0
    f = half_diag / max(rho_max, 1e-6)
    phi = math.radians(pitch_deg)
    dX, dY, dZ = gx - fwd_m, gy, -float(height_m)          # ray from the lens to the ground point
    rx = -dY
    ry = -dX * math.sin(phi) - dZ * math.cos(phi)
    rz = dX * math.cos(phi) - dZ * math.sin(phi)
    theta = np.arctan2(np.hypot(rx, ry), rz)
    rho = theta.copy()
    for _ in range(12 if k1 else 0):                       # theta = rho (1 + k1 rho^2)
        rho = rho - (rho * (1 + k1 * rho * rho) - theta) / (1 + 3 * k1 * rho * rho)
    az = np.arctan2(ry, rx)
    dx, dy = f * rho * np.cos(az), f * rho * np.sin(az)
    if roll_deg:
        cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
        dx, dy = cr * dx - sr * dy, sr * dx + cr * dy
    return dx + W / 2 + cx_px, dy + H / 2


class GroundModel:
    """Pixel -> ground point (car frame: X forward of the rear axle, Y left +), metres."""

    def __init__(self, m: CarModel):
        self.m = m
        u, v = np.meshgrid(np.arange(m.image_w) + 0.5, np.arange(m.image_h) + 0.5)
        self.gx, self.gy = project_pixels(u, v, m.image_w, m.image_h, m.camera_fov_deg, m.cam_pitch_deg,
                                          m.cam_height_m, m.cam_fwd_m, m.cam_roll_deg, m.lens_k1, m.cam_cx_px)

    def project(self, pts_px: np.ndarray, img_w: int, img_h: int) -> np.ndarray:
        """Image points (x, y) in an img_w x img_h image -> ground (X, Y); NaN rows dropped."""
        if pts_px is None or len(pts_px) == 0:
            return np.zeros((0, 2))
        p = np.asarray(pts_px, float)
        u = np.clip((p[:, 0] * self.m.image_w / img_w).astype(int), 0, self.m.image_w - 1)
        v = np.clip((p[:, 1] * self.m.image_h / img_h).astype(int), 0, self.m.image_h - 1)
        g = np.stack([self.gx[v, u], self.gy[v, u]], 1)
        return g[np.isfinite(g).all(1)]

    def expected_x_ratio(self, row_ratio: float, target_ratio: float, path_curvature: float) -> float | None:
        """Where the tracked edge appears on this image row when the car rides the curve perfectly.

        On a straight that's target_ratio. In a corner the edge bends across the image even when the
        car is exactly on its line, so a PD loop aimed at target_ratio would add a second helping of
        steering on top of the feedforward. Aim it here instead and PD only corrects real deviations.
        Model: the car's path is a circle tangent to the car (curvature k, + left); the edge runs
        parallel to it at the lateral offset it has when riding a straight at the target."""
        m = self.m
        r = min(m.image_h - 1, int(row_ratio * m.image_h))
        u0 = int(np.clip(round(target_ratio * m.image_w - 0.5), 0, m.image_w - 1))
        y0 = self.gy[r, u0]                      # edge's lateral offset when riding straight at the target
        if not np.isfinite(y0) or abs(path_curvature) < 1e-3:
            return target_ratio
        cy = 1.0 / path_curvature                # circle centre (0, cy) relative to the rear axle
        R_edge = abs(cy - y0)
        gx, gy = self.gx[r], self.gy[r]
        d = np.abs(np.hypot(gx, gy - cy) - R_edge)
        ok = np.isfinite(d)
        if not ok.any():
            return None
        # the crossing nearest the straight-ahead target position (the circle can cross the row twice)
        cand = np.nonzero(ok & (d < 0.02))[0]
        if not len(cand):
            return None
        u = cand[np.argmin(np.abs(cand - u0))]
        return float((u + 0.5) / m.image_w)

    def lateral_at(self, row_ratio: float, x_ratio: float) -> float | None:
        """Lateral ground offset (Y, left +) of the point seen at this image position: where a line
        that appears there runs beside the car when the car drives parallel to it."""
        m = self.m
        r = min(m.image_h - 1, int(row_ratio * m.image_h))
        u = int(np.clip(round(x_ratio * m.image_w - 0.5), 0, m.image_w - 1))
        y = self.gy[r, u]
        return float(y) if np.isfinite(y) else None

    def x_ratio_for_lateral(self, row_ratio: float, lateral_m: float) -> float | None:
        """Inverse of lateral_at: where on this image row a line lying `lateral_m` beside the car appears."""
        m = self.m
        r = min(m.image_h - 1, int(row_ratio * m.image_h))
        gy = self.gy[r]
        ok = np.isfinite(gy)
        if not ok.any():
            return None
        d = np.where(ok, np.abs(gy - lateral_m), np.inf)
        return float((int(np.argmin(d)) + 0.5) / m.image_w)

    def to_pixels(self, ground: np.ndarray, img_w: int, img_h: int) -> np.ndarray:
        """Ground points (X, Y) -> image points (x, y) in an img_w x img_h image (for the overlay)."""
        m = self.m
        g = np.asarray(ground, float).reshape(-1, 2)
        u, v = ground_to_pixels(g[:, 0], g[:, 1], m.image_w, m.image_h, m.camera_fov_deg, m.cam_pitch_deg,
                                m.cam_height_m, m.cam_fwd_m, m.cam_roll_deg, m.lens_k1, m.cam_cx_px)
        return np.stack([u * img_w / m.image_w, v * img_h / m.image_h], 1)

    def row_distance(self, row_ratio: float) -> float:
        r = min(self.m.image_h - 1, int(row_ratio * self.m.image_h))
        return float(self.gx[r, self.m.image_w // 2])


@dataclass
class Bend:
    curvature: float          # 1/m of the PATH the car rides; + = bends left, - = bends right
    edge_curvature: float     # 1/m of the tracked tape edge itself
    lateral_m: float          # tape edge's lateral offset near the car (Y, left +)
    near_m: float             # closest ground distance the fit covers
    far_m: float              # farthest
    n: int                    # points used
    resid_m: float            # fit RMS residual

    @property
    def radius_m(self) -> float:
        return math.inf if abs(self.curvature) < 1e-6 else 1.0 / abs(self.curvature)


def fit_bend(ground: np.ndarray, max_dist=1.4, min_pts=5, min_span=0.2, straight_tol_m=0.006) -> Bend | None:
    """Fit a circle to the edge's ground points (algebraic least squares: exact for arcs, and a
    huge radius on straights), then move it to the parallel path the car rides.

    Sign: + = the edge bends left (circle centre to the left of the direction of travel)."""
    if ground is None or len(ground) < min_pts:
        return None
    g = ground[(ground[:, 0] > 0.05) & (ground[:, 0] < max_dist)]
    if len(g) < min_pts or np.hypot(np.ptp(g[:, 0]), np.ptp(g[:, 1])) < min_span:
        return None
    X, Y = g[:, 0], g[:, 1]
    i0 = int(np.argmin(X))
    x0, lat = float(X[i0]), float(Y[i0])              # nearest point: the edge's lateral position near the car
    # straight? the points sit within a few mm (RMS) of the chord -> curvature 0. A circle
    # through near-collinear points is ill-conditioned, so don't ask it.
    far = int(np.argmax(X))
    cdx, cdy = X[far] - X[i0], Y[far] - Y[i0]
    chord = math.hypot(cdx, cdy)
    if chord < 1e-6:
        return None
    dev = np.abs((X - X[i0]) * cdy - (Y - Y[i0]) * cdx) / chord
    line_resid = float(np.sqrt(np.mean(dev ** 2)))
    if line_resid < straight_tol_m:
        return Bend(0.0, 0.0, lat, x0, float(X.max()), int(len(g)), line_resid)
    # circle  x^2 + y^2 + D x + E y + F = 0 (centred on the data for conditioning)
    mx, my = X.mean(), Y.mean()
    x, y = X - mx, Y - my
    A = np.stack([x, y, np.ones_like(x)], 1)
    sol, *_ = np.linalg.lstsq(A, -(x ** 2 + y ** 2), rcond=None)
    D, E, F = sol
    cx, cy = -D / 2, -E / 2
    r2 = cx ** 2 + cy ** 2 - F
    if r2 <= 1e-9:
        return None
    R = math.sqrt(r2)
    resid = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - R) ** 2)))
    if resid > 0.6 * line_resid:          # a circle doesn't explain the points much better than a line: no real bend
        return Bend(0.0, 0.0, lat, x0, float(X.max()), int(len(g)), line_resid)
    # direction of travel along the edge at its near end (towards the far points)
    tx, ty = cdx, cdy
    side = np.sign(tx * (cy + my - Y[i0]) - ty * (cx + mx - X[i0]))   # cross product: centre left (+) or right (-)
    k_edge = 0.0 if R > 50 else float(side / R)
    # parallel path at signed distance (car at Y=0, edge at Y=lat): k' = k / (1 + k*lat)
    den = 1.0 + k_edge * lat
    k_path = k_edge / den if den > 0.2 else k_edge
    return Bend(float(k_path), k_edge, lat, x0, float(X.max()), int(len(g)), resid)


def steer_for_curvature(m: CarModel, curvature: float, steer_max_deg: float | None = None) -> float:
    """Steering command (-1 left .. +1 right) whose wheel angle follows this path curvature.
    steer_max_deg: full-lock wheel angle to use (the pilot passes Config.steer_max_deg)."""
    delta = math.degrees(math.atan(m.wheelbase_m * curvature))    # + = left
    return float(np.clip(-delta / (steer_max_deg or m.steer_max_deg), -1.0, 1.0))


def ride_line(ground: np.ndarray, lateral: float, min_pts: int = 5, max_dist: float = 1.6):
    """The line the car should ride: the tracked edge moved sideways by `lateral` (the edge's Y when
    the car is on target: left line > 0, right line < 0). Points near -> far, or None if too short."""
    if ground is None or len(ground) < min_pts:
        return None
    g = np.asarray(ground, float)
    g = g[(g[:, 0] > 0.05) & (np.hypot(g[:, 0], g[:, 1]) < max_dist)]
    if len(g) < min_pts:
        return None
    if len(g) >= 7:                                   # 3-point moving average: pixel steps, not shape
        g = np.stack([np.convolve(g[:, i], np.ones(3) / 3, mode="valid") for i in (0, 1)], 1)
    t = np.gradient(g, axis=0)
    n = np.hypot(t[:, 0], t[:, 1])
    ok = n > 1e-6
    if ok.sum() < 3:
        return None
    g, t = g[ok], t[ok] / n[ok, None]
    ride = g - lateral * np.stack([-t[:, 1], t[:, 0]], 1)          # shift along the left normal
    ride = ride[ride[:, 0] > 0.0]
    return ride if len(ride) >= 2 else None


def pursuit_goal(ground: np.ndarray, lateral: float, lookahead: float, min_pts: int = 5,
                 max_dist: float = 1.6, min_reach: float = 0.25):
    """Pure-pursuit goal point: where the RIDE line is `lookahead` metres from the rear axle.

    ground: the tracked edge's ground points (X forward, Y left +), ordered near -> far.
    lateral: the edge's Y when the car rides at its target (left line > 0, right line < 0); the
    ride line is the edge moved sideways by that much, so it passes through the car when on target.
    Returns (x, y) or None when the edge is too short to aim at. If the edge ends before the
    lookahead distance (it leaves the image in a tight curve) the farthest ride point is used."""
    ride = ride_line(ground, lateral, min_pts, max_dist)
    if ride is None:
        return None
    d = np.hypot(ride[:, 0], ride[:, 1])
    beyond = np.nonzero(d >= lookahead)[0]
    if not len(beyond):
        i = int(np.argmax(d))
        return (float(ride[i, 0]), float(ride[i, 1])) if d[i] >= min_reach else None
    i = int(beyond[0])
    if i == 0:
        return float(ride[0, 0]), float(ride[0, 1])
    f = (lookahead - d[i - 1]) / max(d[i] - d[i - 1], 1e-9)
    p = ride[i - 1] + f * (ride[i] - ride[i - 1])
    return float(p[0]), float(p[1])


def predict_goal(goal, speed_mps: float, wheel_deg: float, wheelbase_m: float, tau_s: float):
    """The goal point as it will be seen from where the car is `tau_s` seconds from now.

    The camera frame shows the world where the car WAS; by the time the steering command reaches the
    wheels the car has moved on. Assume it keeps its current speed and wheel angle (+ = left) for tau:
    it drives an arc of curvature tan(wheel)/L. Move the goal into the car frame at the arc's end
    (X forward from the rear axle, Y left +). With tau 0, zero speed, or a goal that would end up
    behind the car, the goal is returned unchanged."""
    x, y = goal
    s = speed_mps * tau_s
    if s <= 0.0:
        return float(x), float(y)
    k = math.tan(math.radians(wheel_deg)) / wheelbase_m
    yaw = k * s
    if abs(k) < 1e-6:
        dx, dy = s, 0.0
    else:
        dx, dy = math.sin(yaw) / k, (1.0 - math.cos(yaw)) / k
    px, py = x - dx, y - dy
    c, sn = math.cos(yaw), math.sin(yaw)
    nx, ny = c * px + sn * py, -sn * px + c * py
    if nx <= 0.05:
        return float(x), float(y)
    return float(nx), float(ny)


def pursuit_steer(goal, wheelbase_m: float, steer_max_deg: float) -> float:
    """Pure pursuit: wheel angle = atan(2 L sin(alpha) / Ld) for the goal point (x, y) seen from the
    rear axle, as a steering command (-1 left .. +1 right)."""
    x, y = goal
    d2 = x * x + y * y
    delta = math.degrees(math.atan2(2.0 * wheelbase_m * y, d2))    # sin(alpha) / Ld = y / Ld^2; + = left
    return float(np.clip(-delta / steer_max_deg, -1.0, 1.0))


def min_radius(m: CarModel) -> float:
    return m.wheelbase_m / math.tan(math.radians(m.steer_max_deg))


def corner_speed(m: CarModel, curvature: float, margin: float = 0.8) -> float:
    """Fastest speed (m/s) for this curvature: grip limit x margin."""
    if abs(curvature) < 1e-4:
        return math.inf
    return margin * math.sqrt(m.grip_mu * G / abs(curvature))


def speed_of_cmd(m: CarModel, cmd: float) -> float:
    pwm = m.pwm_stop + max(0.0, cmd) * (m.pwm_full - m.pwm_stop)
    xs, ys = zip(*m.speed_table)
    return float(np.interp(pwm, xs, ys))


def cmd_for_speed(m: CarModel, v: float) -> float:
    """Smallest throttle command whose steady speed reaches v (inverse of the speed table)."""
    xs, ys = zip(*m.speed_table)
    if v >= ys[-1]:
        return 1.0
    i = next(k for k in range(1, len(ys)) if ys[k] >= v and ys[k] > ys[k - 1])
    pwm = xs[i - 1] + (v - ys[i - 1]) / (ys[i] - ys[i - 1]) * (xs[i] - xs[i - 1])
    return float(np.clip((pwm - m.pwm_stop) / (m.pwm_full - m.pwm_stop), 0.0, 1.0))


def allowed_speed(m: CarModel, bend: Bend | None, v_now: float, margin: float = 0.8) -> float:
    """Speed allowed NOW so the car can still brake down to the corner speed of the bend it sees
    ahead (the curvature is measured from bend.near_m onwards)."""
    if bend is None:
        return math.inf
    v_c = corner_speed(m, bend.curvature, margin)
    if not math.isfinite(v_c):
        return math.inf
    room = max(0.0, bend.near_m - v_now * m.delay_s)
    return math.sqrt(v_c ** 2 + 2 * m.brake_decel * room)
