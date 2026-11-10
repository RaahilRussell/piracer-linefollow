"""Command -> PWM -> physical quantity, exactly as the car does it.

Donkeycar (donkeycar/parts/actuator.py) turns the normalized commands into
PCA9685 pulse lengths LINEARLY:
    PWMSteering: angle    -1 .. +1  ->  STEERING_LEFT_PWM .. STEERING_RIGHT_PWM
    PWMThrottle: throttle  0 .. +1  ->  THROTTLE_STOPPED_PWM .. THROTTLE_FORWARD_PWM
                 throttle -1 ..  0  ->  THROTTLE_REVERSE_PWM .. THROTTLE_STOPPED_PWM
A normalized steering command is therefore NOT an angle: the angle is whatever
the servo + linkage do at that pulse, which must be measured (steer_angle_table).
Likewise speed comes from the ESC + motor + gearing at that pulse (speed_table).
"""
from __future__ import annotations

import numpy as np


def _clip(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _interp(table, x):
    xs = [p[0] for p in table]
    ys = [p[1] for p in table]
    order = np.argsort(xs)
    return float(np.interp(x, np.array(xs)[order], np.array(ys)[order]))


class SteeringMap:
    def __init__(self, prof):
        self.left_pwm = prof.steer_pwm_left
        self.right_pwm = prof.steer_pwm_right
        self.table = prof.steer_angle_table          # [[pwm, deg]], + = right

    def pwm(self, cmd: float) -> int:
        """Donkeycar PWMSteering: linear -1..1 -> left..right pulse (int ticks)."""
        c = _clip(float(cmd), -1.0, 1.0)
        return int(self.left_pwm + (c + 1.0) / 2.0 * (self.right_pwm - self.left_pwm))

    def angle_deg(self, pwm: float) -> float:
        """Front-wheel (bicycle-equivalent) angle at this pulse, from the calibration table."""
        return _interp(self.table, pwm)

    def cmd_to_angle_rad(self, cmd: float) -> float:
        return np.radians(self.angle_deg(self.pwm(cmd)))

    def max_angle_rad(self):
        """(max left, max right) angle the linkage reaches within the configured PWM range."""
        a_l = self.angle_deg(self.left_pwm)
        a_r = self.angle_deg(self.right_pwm)
        return np.radians(abs(a_l)), np.radians(abs(a_r))


class ThrottleMap:
    def __init__(self, prof):
        self.fwd = prof.thr_pwm_forward
        self.stop = prof.thr_pwm_stopped
        self.rev = prof.thr_pwm_reverse
        self.table = prof.speed_table                # [[pwm, steady m/s]]
        self.start_pwm = prof.start_pwm

    def pwm(self, cmd: float) -> int:
        """Donkeycar PWMThrottle mapping (int ticks)."""
        c = _clip(float(cmd), -1.0, 1.0)
        if c > 0:
            return int(self.stop + c * (self.fwd - self.stop))
        return int(self.rev + (c + 1.0) * (self.stop - self.rev))

    def steady_speed(self, pwm: float, moving: bool) -> float:
        """Speed the car settles at for this pulse (forward). 0 inside the deadband,
        and 0 from rest unless the pulse beats static friction (start_pwm)."""
        if pwm <= self.stop:
            return 0.0
        if not moving and pwm < self.start_pwm:
            return 0.0
        return max(0.0, _interp(self.table, pwm))

    def braking(self, pwm: float) -> bool:
        """Reverse pulse while rolling forward = ESC brake."""
        return pwm < self.stop
