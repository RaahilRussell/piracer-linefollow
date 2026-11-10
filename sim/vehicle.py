"""Vehicle model: kinematic bicycle (reference point = rear axle centre) + optional extensions.

Extensions (each can be switched off to see what it contributes):
  slip       tyre grip limit. If v^2 * curvature would exceed mu*g the car can't follow
             the steering: the turn widens to mu*g / v^2 (understeer) and `slipping` is set.
  servo      the wheels move toward the commanded angle at most servo_rate_dps,
             with an extra first-order lag servo_tau_s.
  accel      speed changes at most accel_max (up) / brake_coast or brake_active (down)
             instead of jumping to the steady speed for the pulse.
  delay      commands reach the servo/ESC actuator_delay_s after they're sent.

Sign convention: steering + = right; world yaw is counter-clockwise positive, so a
right turn decreases yaw.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from .actuators import SteeringMap, ThrottleMap

G = 9.81


@dataclass
class Extensions:
    slip: bool = True
    servo: bool = True
    accel: bool = True
    delay: bool = True


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    delta: float = 0.0            # actual front-wheel angle, rad (+ right)
    # diagnostics of the last step
    steer_cmd: float = 0.0
    thr_cmd: float = 0.0
    steer_pwm: int = 0
    thr_pwm: int = 0
    delta_target: float = 0.0
    kappa_cmd: float = 0.0        # curvature the wheels ask for (1/m, + right)
    kappa: float = 0.0            # curvature actually driven
    yaw_rate: float = 0.0
    a_lat: float = 0.0
    a_lat_max: float = 0.0
    slipping: bool = False
    braking: bool = False
    history: list = field(default_factory=list)


class Vehicle:
    def __init__(self, prof, ext: Extensions | None = None):
        self.p = prof
        self.ext = ext or Extensions()
        self.steer = SteeringMap(prof)
        self.thr = ThrottleMap(prof)
        self.L = prof.wheelbase_m
        self.s = VehicleState()
        self._q = deque()          # (apply_time, steer_cmd, thr_cmd)
        self.t = 0.0
        self._applied = (0.0, 0.0)

    # ------------------------------------------------------------ geometry facts
    def min_radius(self, side="left"):
        """Tightest turn the steering linkage allows (geometry, any speed)."""
        a_l, a_r = self.steer.max_angle_rad()
        a = a_l if side == "left" else a_r
        return self.L / math.tan(a) if a > 1e-6 else float("inf")

    def max_corner_speed(self, radius):
        """Fastest speed at which a turn of this radius can be held without sliding."""
        return math.sqrt(self.p.mu_lat * G * radius)

    def reset(self, x, y, yaw, v=0.0):
        self.s = VehicleState(x=x, y=y, yaw=yaw, v=v)
        self._q.clear()
        self._applied = (0.0, 0.0)

    # ------------------------------------------------------------ step
    def command(self, steer_cmd, thr_cmd):
        delay = self.p.actuator_delay_s if self.ext.delay else 0.0
        self._q.append((self.t + delay, float(steer_cmd), float(thr_cmd)))

    def step(self, dt):
        s, p = self.s, self.p
        while self._q and self._q[0][0] <= self.t + 1e-9:
            _, sc, tc = self._q.popleft()
            self._applied = (sc, tc)
        steer_cmd, thr_cmd = self._applied
        s.steer_cmd, s.thr_cmd = steer_cmd, thr_cmd
        s.steer_pwm, s.thr_pwm = self.steer.pwm(steer_cmd), self.thr.pwm(thr_cmd)

        # --- steering servo
        target = math.radians(self.steer.angle_deg(s.steer_pwm))
        s.delta_target = target
        if self.ext.servo:
            want = target
            if p.servo_tau_s > 0:
                want = s.delta + (target - s.delta) * min(1.0, dt / p.servo_tau_s)
            step = math.radians(p.servo_rate_dps) * dt
            s.delta += max(-step, min(step, want - s.delta))
        else:
            s.delta = target

        # --- speed (ESC + motor)
        moving = s.v > 0.02
        s.braking = self.thr.braking(s.thr_pwm) and moving
        v_ss = self.thr.steady_speed(s.thr_pwm, moving)
        if self.ext.accel:
            if s.braking:
                s.v = max(0.0, s.v - p.brake_active * dt)
            elif v_ss > s.v:
                s.v = min(v_ss, s.v + p.accel_max * dt)
            else:
                s.v = max(v_ss, s.v - p.brake_coast * dt)
        else:
            s.v = 0.0 if s.braking else v_ss

        # --- turning, limited by grip
        s.kappa_cmd = math.tan(s.delta) / self.L
        s.a_lat_max = p.mu_lat * G
        kappa = s.kappa_cmd
        s.slipping = False
        if self.ext.slip and s.v > 0.05 and s.v ** 2 * abs(kappa) > s.a_lat_max:
            kappa = math.copysign(s.a_lat_max / s.v ** 2, kappa)
            s.slipping = True
        s.kappa = kappa
        s.yaw_rate = -s.v * kappa
        s.a_lat = s.v ** 2 * abs(kappa)
        # midpoint integration
        yaw_mid = s.yaw + 0.5 * s.yaw_rate * dt
        s.x += s.v * math.cos(yaw_mid) * dt
        s.y += s.v * math.sin(yaw_mid) * dt
        s.yaw += s.yaw_rate * dt
        self.t += dt
        return s

    def projected_path(self, seconds=1.0, n=25):
        """Where the car goes in the next `seconds` if nothing changes (curvature actually achievable)."""
        s = self.s
        x, y, yaw = s.x, s.y, s.yaw
        dt = seconds / n
        pts = [(x, y)]
        for _ in range(n):
            yaw_mid = yaw - 0.5 * s.v * s.kappa * dt
            x += max(s.v, 0.05) * math.cos(yaw_mid) * dt
            y += max(s.v, 0.05) * math.sin(yaw_mid) * dt
            yaw -= max(s.v, 0.05) * s.kappa * dt
            pts.append((x, y))
        return pts

    def footprint(self):
        """Body rectangle corners in the world (length/width from the verified dimensions).
        Assumes the rear axle sits (length - wheelbase)/2 from the rear bumper."""
        s, p = self.s, self.p
        rear = -(p.body_length_m - self.L) / 2
        front = rear + p.body_length_m
        hw = p.body_width_m / 2
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        return [(s.x + px * c - py * sn, s.y + px * sn + py * c)
                for px, py in ((front, hw), (front, -hw), (rear, -hw), (rear, hw))]
