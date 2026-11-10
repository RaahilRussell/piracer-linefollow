"""PiRacer simulator command line.

  python3 -m sim params                       every parameter, grouped verified / measured / assumed
  python3 -m sim feasibility --speed 1.2      which corners are possible at this speed, and why not
  python3 -m sim run --throttle 0.55          one run with the current controller
  python3 -m sim speeds                       speed sweep (throttle commands -> m/s -> laps / slip / off-track)
  python3 -m sim compare --throttle 0.55      controller variants side by side
  python3 -m sim gains --throttle 0.55        KP x KD grid
  python3 -m sim uncertainty --throttle 0.55  same controller with each ASSUMED value at its low/high end
  python3 -m sim view                         interactive 2D viewer (http://localhost:8892)

Add --rendered to run the real detector on synthetic camera images (slower).
"""
from __future__ import annotations

import argparse
import itertools
import sys

from .controllers import VARIANTS, PilotController
from .params import Profile
from .vehicle import Extensions
from .world import World


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def run_one(prof=None, seconds=40.0, speed=None, variant="current", overrides=None, seed=0, vision="geometric",
            ext=None, target=None, row=None, camera_fps=None, processing_s=None):
    prof = prof or Profile.load()
    clk = _Clock()
    v = dict(VARIANTS.get(variant, {}))
    ov = dict(v.get("overrides", {}))
    ov.update(overrides or {})
    ctrl = PilotController(variant, ov, speed=speed, clock=clk)
    w = World(prof, ext or Extensions(), vision=vision, seed=seed,
              target_ratio=target if target is not None else ctrl.cfg.target_x_ratio,
              row_ratio=row if row is not None else ctrl.cfg.lookahead_ratio,
              camera_fps=camera_fps, processing_s=processing_s)
    w.want_edges = ctrl.needs_edges
    while w.t < seconds:
        clk.t = w.t
        x, found, both, age, motion, img = w.observe()
        angle, thr, _ = ctrl.step(x, found, both, age, motion, img, edges=w.edges, source=w.edges_source)
        w.tick(angle, thr)
    return w, ctrl


def _table(rows, cols):
    widths = [max(len(str(c)), *(len(str(r.get(c, ""))) for r in rows)) for c in cols]
    print("  ".join(str(c).rjust(w) for c, w in zip(cols, widths)))
    for r in rows:
        print("  ".join(str(r.get(c, "")).rjust(w) for c, w in zip(cols, widths)))


def _assumed_banner(prof):
    a = prof.assumed_in_use()
    print(f"NOTE: {len(a)} values are ASSUMED placeholders (not measured): {', '.join(a)}")
    print("      Results show how the controllers behave on this MODEL; confirm on the car.\n")


COLS = ["laps", "lap_s", "off_track", "err_mean_cm", "err_p95_cm", "min_clear_left_cm",
        "min_body_clear_left_cm", "min_body_clear_right_cm", "slip_pct",
        "steer_sat_pct", "lost_pct", "mean_speed"]


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python3 -m sim")
    ap.add_argument("cmd", choices=["params", "feasibility", "run", "speeds", "compare", "gains", "uncertainty", "view"])
    ap.add_argument("--speed", type=float, default=1.0, help="m/s (feasibility)")
    ap.add_argument("--throttle", type=float, default=0.55, help="fixed throttle command for run/compare/gains")
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--rendered", action="store_true", help="run the real detector on synthetic images")
    ap.add_argument("--fps", type=float)
    ap.add_argument("--latency", type=float, help="processing delay, s")
    ap.add_argument("--no-slip", action="store_true")
    ap.add_argument("--port", type=int, default=8892)
    a = ap.parse_args(argv)
    prof = Profile.load()
    vision = "rendered" if a.rendered else "geometric"
    ext = Extensions(slip=not a.no_slip)
    kw = dict(seconds=a.seconds, vision=vision, ext=ext, camera_fps=a.fps, processing_s=a.latency)

    if a.cmd == "params":
        print(prof.report())
        return
    if a.cmd == "view":
        from .server import serve
        return serve(a.port)
    _assumed_banner(prof)
    if a.cmd == "feasibility":
        w, _ = run_one(prof, seconds=0.1)
        rows, assumed = w.feasibility(a.speed)
        print(f"Ride offset from the left tape edge: {100 * w._ride:.1f} cm; min turning radius "
              f"{w.veh.min_radius('left'):.2f} m (left) / {w.veh.min_radius('right'):.2f} m (right)")
        _table(rows, ["s0", "direction", "radius_path_m", "min_radius_m", "steer_needed_deg", "steer_available_deg", "v_max_mps", "verdict"])
        print("depends on ASSUMED:", ", ".join(assumed))
    elif a.cmd == "run":
        w, _ = run_one(prof, speed=a.throttle, **kw)
        print(w.metrics.summary())
    elif a.cmd == "speeds":
        rows = []
        from .actuators import ThrottleMap
        tm = ThrottleMap(prof)
        for thr in (0.50, 0.55, 0.60, 0.65, 0.70, 0.80, 0.90, 1.00):
            w, _ = run_one(prof, speed=thr, **kw)
            r = w.metrics.summary()
            pwm = tm.pwm(thr)
            r.update(throttle=thr, pwm=pwm, steady_mps=round(tm.steady_speed(pwm, True), 2))
            feas, _ = w.feasibility(r["steady_mps"])
            r["corners"] = "; ".join(sorted({f["verdict"].split(":")[0] for f in feas}))
            rows.append(r)
        _table(rows, ["throttle", "pwm", "steady_mps"] + COLS + ["corners"])
    elif a.cmd == "compare":
        rows = []
        for name in VARIANTS:
            w, _ = run_one(prof, speed=a.throttle, variant=name, **kw)
            r = w.metrics.summary()
            r["controller"] = name
            rows.append(r)
        _table(rows, ["controller"] + COLS)
    elif a.cmd == "gains":
        rows = []
        for kp, kd in itertools.product((1.5, 2.5, 3.5, 5.0, 7.0), (0.0, 0.2, 0.4, 0.8)):
            w, _ = run_one(prof, speed=a.throttle, overrides={"kp": kp, "kd": kd}, **kw)
            r = w.metrics.summary()
            r.update(kp=kp, kd=kd)
            rows.append(r)
        _table(rows, ["kp", "kd"] + COLS)
    elif a.cmd == "uncertainty":
        rows = []
        w, _ = run_one(prof, speed=a.throttle, **kw)
        r = w.metrics.summary()
        r.update(param="(as profile)", value="")
        rows.append(r)
        for name in prof.assumed_in_use():
            e = prof.entry(name)
            if "range" not in e or not isinstance(e["value"], (int, float)):
                continue
            for val in e["range"]:
                try:
                    q = prof.with_values(**{name: type(e["value"])(val)})
                    w, _ = run_one(q, speed=a.throttle, **kw)
                except Exception as ex:  # a value that breaks the geometry (e.g. camera sees no ground)
                    rows.append({"param": name, "value": val, "laps": f"error: {ex}"})
                    continue
                r = w.metrics.summary()
                r.update(param=name, value=val)
                rows.append(r)
        _table(rows, ["param", "value"] + COLS)


if __name__ == "__main__":
    sys.exit(main())
