"""Gain scheduling for the line follower, the way vehicle-dynamics teams tune lateral control.

For each speed profile:
  1. LINEAR SCREEN  Linearize the loop at that speed: kinematic bicycle + camera preview
     (the lookahead row is D metres ahead) + servo lag + every delay (processing, actuator,
     frame age), closed by the EXACT digital controller that runs on the car (20 Hz,
     PD with the 0.6/0.4 derivative filter, output smoothing). For every KP/KD candidate:
       - stability (eigenvalues of the one-period map of the sampled-data loop)
       - gain margin (how much KP/KD could grow before instability)
       - delay margin (extra latency tolerated before instability)
       - phase margin + crossover (frequency response)
       - damping ratio of the dominant closed-loop poles
     Keep candidates with PM >= 45 deg, GM >= 6 dB, delay margin >= 40 ms, damping >= 0.4.
  2. NONLINEAR SCORE  Drive the survivors in the full simulator (grip limit, servo slew,
     camera, speed plan): offset-step manoeuvre + laps. Cost = tracking error + overshoot +
     slip + steering pinned + off-track.
  3. ROBUSTNESS  Re-test the best few with every ASSUMED parameter drawn across its range
     (Monte Carlo). Pick the best that stays stable and on track across the draws.
Output: sim/gain_schedule.json (+ printed table) - one baseline KP/KD per speed profile.

The margins are as good as the model: until the assumed parameters are measured, treat the
schedule as a starting point, and check the dashboard's tuning-health panel on the car.
"""
from __future__ import annotations

import json
import math
import os
import time

import numpy as np
from scipy.linalg import expm

from .actuators import SteeringMap, ThrottleMap
from .camera import Camera
from .params import Profile

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
REQ = {"pm_deg": 45.0, "gm_db": 6.0, "delay_margin_ms": 40.0, "zeta": 0.4}


# ------------------------------------------------------------------ the linear loop
class LinearLoop:
    """Sampled-data model of the steering loop at one speed (small-signal, no clipping)."""

    def __init__(self, prof, v, cfg, n_sub=8, extra_delay_s=0.0):
        self.p, self.v, self.cfg = prof, max(v, 0.05), cfg
        self.T = 1.0 / prof.control_hz
        self.n = n_sub
        self.h = self.T / n_sub
        cam = Camera(prof)
        r = min(cam.H - 1, int(cfg.lookahead_ratio * cam.H))
        ui = int(np.clip(round(cfg.target_x_ratio * cam.W - 0.5), 1, cam.W - 2))
        dgy_du = (cam.gy[r, ui + 1] - cam.gy[r, ui - 1]) / 2.0          # m per pixel (negative)
        self.k_img = -1.0 / (cam.W * dgy_du)                              # image-width fraction per metre
        self.D = float(cam.gx[r, ui])                                     # preview distance ahead of rear axle
        sm = SteeringMap(prof)
        self.ks = (sm.cmd_to_angle_rad(0.1) - sm.cmd_to_angle_rad(-0.1)) / 0.2   # rad per unit command (+ right)
        frame_age = max(0.0, 0.5 / prof.camera_fps - 0.5 / prof.control_hz)
        self.delay_s = prof.processing_s + prof.actuator_delay_s + frame_age + extra_delay_s
        self.tau_s = prof.servo_tau_s
        L = prof.wheelbase_m
        # continuous plant  x = [y (left +), psi (left +), delta (actual, right +)]
        A = np.zeros((3, 3))
        A[0, 1] = self.v
        A[1, 2] = -self.v / L
        B = np.zeros((3, 1))
        if self.tau_s > 1e-4:
            A[2, 2] = -1.0 / self.tau_s
            B[2, 0] = 1.0 / self.tau_s
        self.A, self.B = A, B
        M = expm(np.block([[A, B], [np.zeros((1, 4))]]) * self.h)
        self.Ad, self.Bd = M[:3, :3], M[:3, 3:4]
        self.instant_servo = self.tau_s <= 1e-4

    # one control period as a linear map on the augmented state
    def _period_map(self, kp, kd, gain=1.0, extra_steps=0):
        c = self.cfg
        a = min(max(c.steering_smoothing, 0.0), 0.95)
        d_steps = int(round(self.delay_s / self.h)) + extra_steps
        n_x = 3
        # z = [x(3), buf(d_steps), e_prev, d_filt, ang]
        N = n_x + d_steps + 3
        i_buf, i_ep, i_df, i_ang = n_x, n_x + d_steps, n_x + d_steps + 1, n_x + d_steps + 2

        def step(z):
            z = z.copy()
            y, psi = z[0], z[1]
            e = self.k_img * (y + self.D * psi)
            d = (e - z[i_ep]) / self.T
            df = 0.6 * z[i_df] + 0.4 * d
            raw = gain * (kp * e + kd * df)
            ang = a * z[i_ang] + (1 - a) * raw
            z[i_ep], z[i_df], z[i_ang] = e, df, ang
            for _ in range(self.n):
                u_in = z[i_ang] * self.ks                     # command held for the period (ZOH)
                if d_steps:
                    u_out = z[i_buf + d_steps - 1]
                    z[i_buf + 1:i_buf + d_steps] = z[i_buf:i_buf + d_steps - 1]
                    z[i_buf] = u_in
                else:
                    u_out = u_in
                x = z[:3]
                if self.instant_servo:
                    x = x.copy()
                    x[2] = u_out
                    x = self.Ad @ x + self.Bd[:, 0] * 0.0
                    x[2] = u_out
                else:
                    x = self.Ad @ x + self.Bd[:, 0] * u_out
                z[:3] = x
            return z
        Mtx = np.column_stack([step(np.eye(N)[i]) for i in range(N)])
        return Mtx

    def poles(self, kp, kd, gain=1.0, extra_steps=0):
        return np.linalg.eigvals(self._period_map(kp, kd, gain, extra_steps))

    def stable(self, kp, kd, gain=1.0, extra_steps=0):
        return float(np.max(np.abs(self.poles(kp, kd, gain, extra_steps)))) < 1.0 - 1e-9

    def analyze(self, kp, kd):
        lam = self.poles(kp, kd)
        rho = float(np.max(np.abs(lam)))
        out = {"stable": rho < 1.0, "spectral_radius": round(rho, 4)}
        if not out["stable"]:
            return out
        # damping of the dominant oscillatory closed-loop poles (z -> s = ln z / T)
        s = np.log(lam[np.abs(lam) > 1e-6].astype(complex)) / self.T
        osc = s[np.abs(s.imag) > 1e-3]
        dom = osc[np.argsort(-osc.real)][:2] if len(osc) else []
        out["zeta"] = round(float(min(-x.real / abs(x) for x in dom)), 3) if len(dom) else 1.0
        # gain margin: grow KP and KD together until unstable
        g = 1.0
        while g < 20 and self.stable(kp, kd, gain=g * 1.05):
            g *= 1.05
        out["gm_db"] = round(20 * math.log10(g * 1.05), 1) if g < 20 else 26.0
        # delay margin: extra latency (substeps) until unstable
        k = 0
        while k < int(0.3 / self.h) and self.stable(kp, kd, extra_steps=k + 1):
            k += 1
        out["delay_margin_ms"] = round(1000 * (k + 1) * self.h, 0) if k < int(0.3 / self.h) else 300.0
        # phase margin from the loop frequency response (continuous plant x ZOH x delay x digital PD)
        pm, wc = self.phase_margin(kp, kd)
        out["pm_deg"], out["crossover_hz"] = (round(pm, 1) if pm is not None else None), (round(wc / (2 * math.pi), 2) if wc else None)
        return out

    def loop_tf(self, kp, kd, w):
        T, c = self.T, self.cfg
        a = min(max(c.steering_smoothing, 0.0), 0.95)
        z = np.exp(1j * w * T)
        S = (1 - a) / (1 - a / z)
        Dz = 0.4 * (1 - 1 / z) / T / (1 - 0.6 / z)
        C = S * (kp + kd * Dz)
        s = 1j * w
        servo = 1.0 if self.instant_servo else 1.0 / (self.tau_s * s + 1)
        zoh = (1 - np.exp(-s * T)) / (s * T)
        P = self.k_img * self.ks * (self.v / self.p.wheelbase_m) * (self.D * s + self.v) / s ** 2
        return C * zoh * servo * P * np.exp(-s * self.delay_s)

    def phase_margin(self, kp, kd):
        w = np.logspace(-1.5, math.log10(math.pi / self.T * 0.999), 3000)
        Lw = self.loop_tf(kp, kd, w)
        mag = np.abs(Lw)
        ph = np.unwrap(np.angle(Lw))
        ph -= 2 * math.pi * np.round((ph[0] + math.pi) / (2 * math.pi))   # low-frequency phase ~ -180 deg
        idx = np.nonzero((mag[:-1] >= 1) & (mag[1:] < 1))[0]
        if not len(idx):
            return None, None
        i = idx[-1]
        return math.degrees(ph[i]) + 180.0, float(w[i])


# ------------------------------------------------------------------ nonlinear scoring
def nonlinear_score(prof, speed, kp, kd, seeds=(0, 1), seconds=30.0):
    from .__main__ import run_one
    tot, parts = 0.0, []
    for sd in seeds:
        w, _ = run_one(prof, seconds=seconds, speed=speed, overrides={"kp": kp, "kd": kd}, seed=sd)
        m = w.metrics.summary()
        steer = np.array([e for e in w.metrics.err_samples]) if False else None
        clearance = min(m.get("min_body_clear_left_cm") or 0.0, m.get("min_body_clear_right_cm") or 0.0)
        cost = (m["err_mean_cm"] + 0.5 * m["err_p95_cm"] + 40.0 * m["off_track"] + 0.4 * m["slip_pct"]
                + 0.3 * m["steer_sat_pct"] + 0.2 * m["lost_pct"] + 2.0 * max(0.0, 2.0 - clearance))
        tot += cost
        parts.append(m)
    step = step_response(prof, speed, kp, kd)
    tot = tot / len(seeds) + 0.15 * step["overshoot_pct"] + 2.0 * step["settle_s"]
    return tot, parts, step


def step_response(prof, speed, kp, kd, offset_m=0.06, seconds=6.0):
    """Start 6 cm off the ride line on a long straight; measure overshoot and settling."""
    from .__main__ import _Clock
    from .controllers import PilotController
    from .world import World
    q = prof.with_values(track_straight_m=max(prof.track_straight_m, 12.0))
    clk = _Clock()
    ctrl = PilotController("step", {"kp": kp, "kd": kd}, speed=speed, clock=clk)
    w = World(q, seed=5, target_ratio=ctrl.cfg.target_x_ratio, row_ratio=ctrl.cfg.lookahead_ratio)
    w.want_edges = ctrl.needs_edges
    s = w.veh.s
    s.y += offset_m                                   # shifted toward the left line
    w._ride_ref = w._ride
    errs, ts = [], []
    while w.t < seconds:
        clk.t = w.t
        x, found, both, age, motion, img = w.observe()
        a, t_, _ = ctrl.step(x, found, both, age, motion, img, edges=w.edges)
        w.tick(a, t_)
        off = w.track.offset_from_left(s.x, s.y) - w._ride
        errs.append(off)
        ts.append(w.t)
        if w.metrics.off_track:
            return {"overshoot_pct": 200.0, "settle_s": seconds}
    e = np.array(errs)
    first = e[0] if abs(e[0]) > 1e-3 else -offset_m
    over = max(0.0, float(np.max(-np.sign(first) * e)) / abs(first) * 100)
    band = 0.15 * abs(first)
    outside = np.nonzero(np.abs(e) > band)[0]
    settle = ts[outside[-1]] if len(outside) else 0.0
    return {"overshoot_pct": round(over, 1), "settle_s": round(settle, 2)}


# ------------------------------------------------------------------ robustness
def monte_carlo_profiles(prof, n=10, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        vals = {}
        for k in prof.assumed_in_use():
            e = prof.entry(k)
            if k == "speed_table":
                continue                      # no defensible uncertainty range is recorded for this table
            if k == "steer_angle_table":
                f = rng.uniform(e["range"][0], e["range"][1]) / 25.0
                vals[k] = [[pw, ang * f] for pw, ang in e["value"]]
            elif "range" in e and isinstance(e["value"], (int, float)):
                lo, hi = e["range"]
                vals[k] = type(e["value"])(rng.uniform(lo, hi))
        out.append(prof.with_values(**vals))
    return out


# ------------------------------------------------------------------ the schedule
PROFILES = [("Crawl", 0.55), ("Slow", 0.62), ("Medium", 0.70), ("Fast", 0.80), ("Faster", 0.90), ("Max", 1.00)]
KP_GRID = np.round(np.arange(1.0, 10.01, 0.5), 2)
KD_GRID = np.round(np.arange(0.0, 1.21, 0.1), 2)


def cmd_for_speed(tm: ThrottleMap, v_target):
    """Smallest throttle command whose steady speed reaches v_target (inverse of the speed table)."""
    for c in np.arange(0.45, 1.0001, 0.01):
        if tm.steady_speed(tm.pwm(c), True) >= v_target:
            return round(float(c), 2)
    return 1.0


def schedule(prof=None, profiles=PROFILES, mc=10, verbose=True):
    import line_config
    from donkey_part import load_line_config
    prof = prof or Profile.load()
    cfg, _ = load_line_config(line_config)
    tm = ThrottleMap(prof)
    from .camera import target_offset
    from .feasibility import corner_report
    from .track import Track
    track = Track.stadium(prof)
    ride = target_offset(Camera(prof), None, cfg.target_x_ratio, cfg.lookahead_ratio, prof.tape_width_m)
    results = []
    for name, cmd in profiles:
        t0 = time.time()
        v = tm.steady_speed(tm.pwm(cmd), True)
        feas, _ = corner_report(prof, track, ride, v)
        v_corner_max = min(f["v_max_mps"] for f in feas) if feas else v
        # speed profile: if the corners can't take this speed with a 20% margin, slow for the corners
        corner_cmd = cmd if v <= 0.8 * v_corner_max else cmd_for_speed(tm, 0.8 * v_corner_max)
        speed = (cmd, corner_cmd) if corner_cmd < cmd else cmd
        v_lin = min(v, 0.8 * v_corner_max) if corner_cmd < cmd else v
        lin = LinearLoop(prof, max(v, 0.1), cfg)
        # 1. linear screen at the straight speed AND the corner speed (gains must work at both)
        lin_c = LinearLoop(prof, max(v_lin, 0.1), cfg)
        cands = []
        for kp in KP_GRID:
            for kd in KD_GRID:
                a1 = lin.analyze(kp, kd)
                if not a1["stable"]:
                    continue
                a2 = lin_c.analyze(kp, kd) if lin_c.v != lin.v else a1
                if not a2["stable"]:
                    continue
                worst = {k: min(a1.get(k) or 0, a2.get(k) or 0) for k in ("pm_deg", "gm_db", "delay_margin_ms", "zeta")}
                ok = all(worst[k] >= REQ[k] for k in REQ)
                cands.append((kp, kd, ok, worst, a1.get("crossover_hz") or 0))
        passing = [c for c in cands if c[2]]
        pool = passing or sorted(cands, key=lambda c: -c[3]["pm_deg"])[:10]
        # prefer the higher-bandwidth candidates (tighter tracking), then score them
        pool = sorted(pool, key=lambda c: -c[4])[:14]
        scored = []
        for kp, kd, ok, worst, wc in pool:
            cost, parts, step = nonlinear_score(prof, speed, kp, kd)
            scored.append((cost, kp, kd, ok, worst, wc, parts, step))
        scored.sort(key=lambda s: s[0])
        # 3. robustness on the best few
        draws = monte_carlo_profiles(prof, n=mc)
        chosen = None
        for cost, kp, kd, ok, worst, wc, parts, step in scored[:4]:
            fails, worst_pm, off, clearance_failures, min_clearance = 0, 999.0, 0, 0, float("inf")
            for q in draws:
                a = LinearLoop(q, max(v_lin, 0.1), cfg).analyze(kp, kd)
                if not a["stable"]:
                    fails += 1
                    continue
                worst_pm = min(worst_pm, a.get("pm_deg") or 0)
            for q in draws[:4]:
                from .__main__ import run_one
                w, _ = run_one(q, seconds=20, speed=speed, overrides={"kp": kp, "kd": kd}, seed=3)
                off += w.metrics.off_track
                m = w.metrics.summary()
                clear = min(m.get("min_body_clear_left_cm") or 0.0, m.get("min_body_clear_right_cm") or 0.0)
                min_clearance = min(min_clearance, clear)
                clearance_failures += int(clear < 2.0)
            rob = {"draws": len(draws), "unstable": fails, "worst_pm_deg": round(worst_pm, 1),
                   "off_track_in_4_runs": off, "clearance_failures_in_4_runs": clearance_failures,
                   "min_body_clearance_cm": round(min_clearance, 1) if math.isfinite(min_clearance) else None}
            entry = (cost, kp, kd, ok, worst, wc, parts, step, rob)
            if fails == 0 and off == 0 and clearance_failures == 0:
                chosen = entry
                break
            if chosen is None or (fails, off, clearance_failures) < (chosen[8]["unstable"], chosen[8]["off_track_in_4_runs"], chosen[8]["clearance_failures_in_4_runs"]):
                chosen = entry
        cost, kp, kd, ok, worst, wc, parts, step, rob = chosen
        lap = [p["lap_s"] for p in parts if p["lap_s"]]
        row = {
            "profile": name, "straight_cmd": cmd, "corner_cmd": corner_cmd,
            "straight_pwm": tm.pwm(cmd), "corner_pwm": tm.pwm(corner_cmd),
            "straight_mps": round(v, 2), "corner_mps_limit": round(v_corner_max, 2),
            "kp": float(kp), "kd": float(kd), "meets_margins": bool(ok),
            "pm_deg": worst["pm_deg"], "gm_db": worst["gm_db"], "delay_margin_ms": worst["delay_margin_ms"],
            "zeta": worst["zeta"], "crossover_hz": wc,
            "err_mean_cm": round(np.mean([p["err_mean_cm"] for p in parts]), 1),
            "err_p95_cm": round(np.mean([p["err_p95_cm"] for p in parts]), 1),
            "min_body_clearance_cm": round(min(min(p["min_body_clear_left_cm"] for p in parts),
                                                min(p["min_body_clear_right_cm"] for p in parts)), 1),
            "lap_s": round(float(np.mean(lap)), 2) if lap else None,
            "slip_pct": round(np.mean([p["slip_pct"] for p in parts]), 1),
            "off_track": int(sum(p["off_track"] for p in parts)),
            "step_overshoot_pct": step["overshoot_pct"], "step_settle_s": step["settle_s"],
            "robustness": rob, "candidates_passing_margins": len(passing), "seconds": round(time.time() - t0, 1),
            "robust_physical": bool(fails == 0 and off == 0 and clearance_failures == 0),
        }
        results.append(row)
        if verbose:
            print(f"{name:<7} thr {cmd:.2f}/{corner_cmd:.2f} ({row['straight_mps']} m/s): KP {kp:.1f} KD {kd:.1f} | "
                  f"PM {row['pm_deg']} GM {row['gm_db']} dB DM {row['delay_margin_ms']} ms zeta {row['zeta']} | "
                  f"err {row['err_mean_cm']}/{row['err_p95_cm']} cm body margin {row['min_body_clearance_cm']} cm "
                  f"lap {row['lap_s']} slip {row['slip_pct']}% "
                  f"off {row['off_track']} | step {step['overshoot_pct']}% {step['settle_s']} s | robust {rob}", flush=True)
    out = {"generated": time.strftime("%Y-%m-%d %H:%M"), "requirements": REQ,
           "assumed_parameters": prof.assumed_in_use(),
           "note": "Model-based starting KP/KD per speed profile. Off-track uses the full chassis footprint; uncertainty sweeps include track geometry and require 2 cm body clearance. Physical values remain assumptions until measured; this is not a real-car calibration.",
           "profiles": results}
    with open(os.path.join(HERE, "gain_schedule.json"), "w") as f:
        json.dump(out, f, indent=2)
    return out


if __name__ == "__main__":
    import sys
    sys.path[:0] = [os.path.join(os.path.dirname(HERE), "pi"), os.path.dirname(HERE)]
    schedule()
