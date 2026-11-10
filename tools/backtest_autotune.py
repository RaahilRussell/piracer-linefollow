#!/usr/bin/env python3
"""Backtest the auto-tuner on the PiRacer simulator (sim/).

For each scenario (the profile, and the profile with ASSUMED values pushed to the ends of
their ranges): run the tuner from its slow start until it finishes (off track = crash; the
simulator puts the car back), then drive 40 s with what it picked and compare to a crawl.
"""
from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "pi"), ROOT]

from autotune import AutoTuner  # noqa: E402
from sim.__main__ import _Clock, run_one  # noqa: E402
from sim.controllers import PilotController  # noqa: E402
from sim.params import Profile  # noqa: E402
from sim.world import World  # noqa: E402


def load_schedule():
    import json
    with open(os.path.join(ROOT, "sim", "gain_schedule.json")) as f:
        return [(r["straight_cmd"], r["kp"], r["kd"]) for r in json.load(f)["profiles"]]


def tune(prof, kp=None, kd=None, seed=0, max_s=900, ceiling=1.0, start=0.55, window_s=10.0, verbose=False,
         strategy="ladder", hold_speed=None):
    clk = _Clock()
    ov = {k: v for k, v in (("kp", kp), ("kd", kd)) if v is not None}
    ctrl = PilotController("autotune", ov, speed=start, clock=clk)
    ctrl.cap.mode_caps["local"] = max(ctrl.cap.mode_caps.get("local", 1.0), ceiling)
    ctrl.cap.top = max(ctrl.cap.top, ceiling)
    logs = []
    tuner = AutoTuner(ctrl.pilot, start=start, step=0.03, ceiling=ceiling, window_s=window_s, clock=clk,
                      log=lambda lvl, txt: logs.append((round(clk.t, 1), lvl, txt)), schedule=load_schedule())
    w = World(prof, seed=seed, target_ratio=ctrl.cfg.target_x_ratio, row_ratio=ctrl.cfg.lookahead_ratio)
    tuner.begin(strategy, speed=hold_speed)
    seen_off = 0
    while tuner.state == "running" and w.t < max_s:
        clk.t = w.t
        x, found, both, age, motion, img = w.observe()
        angle, thr, cmd = ctrl.step(x, found, both, age, motion, img, edges=w.edges)
        tuner.update("local", cmd, cmd.status == "TRACKING")
        w.tick(angle, thr)
        if w.metrics.off_track > seen_off:
            seen_off = w.metrics.off_track
            logs.append((round(w.t, 1), "bad", "CRASH (off track): car put back at the start"))
            ctrl.pilot.ctrl.reset()
            if tuner.state == "running":
                tuner._judge(force_fail=True)
    if verbose:
        for t, lvl, txt in logs:
            print(f"   {t:7.1f}s {lvl:4} {txt}")
    return tuner, w.metrics.off_track, w.t


def drive(prof, kp, kd, straight, corner, seed=7, seconds=40):
    w, _ = run_one(prof, seconds=seconds, speed=(straight, corner), overrides={"kp": kp, "kd": kd}, seed=seed)
    return w.metrics.summary()


def scenarios():
    p = Profile.load()
    out = [("PiRacer profile", p, {})]
    tbl = p.steer_angle_table
    out += [
        ("  steering 0.7x (less lock)", p.with_values(steer_angle_table=[[a, b * 0.7] for a, b in tbl]), {}),
        ("  steering 1.3x (more lock)", p.with_values(steer_angle_table=[[a, b * 1.3] for a, b in tbl]), {}),
        ("  slippery (mu 0.3)", p.with_values(mu_lat=0.3), {}),
        ("  grippy (mu 0.9)", p.with_values(mu_lat=0.9), {}),
        ("  slow servo (150 deg/s)", p.with_values(servo_rate_dps=150.0, servo_tau_s=0.08), {}),
        ("  laggy (processing 51 ms + 60 ms actuator)", p.with_values(processing_s=0.051, actuator_delay_s=0.06), {}),
        ("  noisy line (0.02)", p.with_values(meas_noise=0.02, meas_dropout=0.1), {}),
        ("  start KP too high (8)", p, {"kp": 8.0}),
        ("  start KP too low (1.5)", p, {"kp": 1.5}),
    ]
    return out


def compare_strategies(verbose=False):
    p = Profile.load()
    cases = [("profile", p), ("slippery mu 0.3", p.with_values(mu_lat=0.3)),
             ("laggy 51+60 ms", p.with_values(processing_s=0.051, actuator_delay_s=0.06))]
    runs = [("ladder", {}), ("scheduled", {}), ("gains @0.70", {"strategy": "gains", "hold_speed": 0.70}),
            ("max_speed (KP3.5 KD0.4)", {"strategy": "max_speed"}), ("zn @0.62", {"strategy": "zn", "hold_speed": 0.62})]
    print(f"{'strategy':<26}{'case':<18}{'tune s':>7}{'crashes':>8}  {'picked':<36}{'lap s':>7}{'err p95 cm':>11}{'off':>5}")
    for label, kw in runs:
        kw = dict(kw)
        strat = kw.pop("strategy", label.split()[0])
        for cname, prof in cases:
            tuner, crashes, t = tune(prof, seed=3, verbose=verbose, strategy=strat, **kw)
            b = tuner.best
            if b is None:
                print(f"{label:<26}{cname:<18}{t:>7.0f}{crashes:>8}  {'(no result: ' + tuner.message[:40] + ')':<36}")
                continue
            m = drive(prof, b.kp, b.kd, b.straight, b.corner)
            pick = f"{b.straight:.2f}/{b.corner:.2f} KP {b.kp:.2f} KD {b.kd:.2f}"
            print(f"{label:<26}{cname:<18}{t:>7.0f}{crashes:>8}  {pick:<36}{str(m['lap_s']):>7}{m['err_p95_cm']:>11}{m['off_track']:>5}")


if __name__ == "__main__":
    verbose = "-v" in sys.argv
    if "--strategies" in sys.argv:
        compare_strategies(verbose)
        sys.exit(0)
    print("ASSUMED (unmeasured) values in the profile:", ", ".join(Profile.load().assumed_in_use()), "\n")
    print(f"{'scenario':<44}{'tune s':>7}{'crashes':>8}  {'picked':<44}{'lap s':>7}{'crawl lap s':>12}{'off':>5}")
    for name, prof, start in scenarios():
        tuner, crashes, t = tune(prof, seed=3, verbose=verbose, **start)
        crawl = drive(prof, start.get("kp", 3.5), start.get("kd", 0.4), 0.55, 0.55)
        b = tuner.best
        if b is None:
            print(f"{name:<44}{t:>7.0f}{crashes:>8}  {'no passing speed':<44}{'-':>7}{str(crawl['lap_s']):>12}")
            continue
        m = drive(prof, b.kp, b.kd, b.straight, b.corner)
        pick = f"{b.straight:.2f}/{b.corner:.2f} KP {b.kp:.2f} KD {b.kd:.2f}"
        print(f"{name:<44}{t:>7.0f}{crashes:>8}  {pick:<44}{str(m['lap_s']):>7}{str(crawl['lap_s']):>12}{m['off_track']:>5}")
