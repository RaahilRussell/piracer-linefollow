#!/usr/bin/env python3
"""Check a whole setup for race day: many seeds, long runs, and every assumed physical value moved
inside its plausible range (steering lock 16-20 deg, grip, delays, speed table, servo, camera mount).

A setup passes when it never runs WIDE (over the outside tape) and never loses the inside line by
more than tools/simdrive.INSIDE_TOLERANCE_M, in any variation. Riding over the inside line is
accepted and reported as "% over the inside line".

Everything here is SIMULATED (real detector + pilot, simulated car and camera).

    python3 tools/race_eval.py                       # the race values in pi/line_config.py (RACE_SPEED_*)
    python3 tools/race_eval.py --speeds 1.0/0.92,1.0/0.95 --seeds 6 --seconds 60 --pairs
    python3 tools/race_eval.py --set pure_pursuit=0 --set target_x_ratio=0.40   # PD + feedforward instead
"""
from __future__ import annotations

import argparse
import os
import sys
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simdrive as sd  # noqa: E402

# Two unknowns wrong at once, in the direction that hurts (runs wide).
PAIRS = {
    "fast car + low grip": ("speed +15%", "grip 0.40"),
    "fast car + delay": ("speed +15%", "delay +40 ms"),
    "fast car + 16 deg lock": ("speed +15%", "lock 16 deg"),
    "low grip + slow servo": ("grip 0.40", "slow servo"),
    "16 deg lock + delay": ("lock 16 deg", "delay +40 ms"),
    "slow servo + delay": ("slow servo", "delay +40 ms"),
}


def parse_value(text):
    if text.lower() in ("true", "false"):
        return text.lower() == "true"
    return float(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speeds", default="", help="straight/curve pairs, e.g. 1.0/0.92,0.95/0.90 (default: line_config)")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--set", action="append", default=[], metavar="name=value", help="override a setting")
    ap.add_argument("--pairs", action="store_true", help="also test two unknowns wrong at once")
    ap.add_argument("--laps", type=int, default=10)
    a = ap.parse_args()
    setup = {}
    for item in a.set:
        k, v = item.split("=", 1)
        cfg0 = sd.build()[0]
        cur = getattr(cfg0, k)
        setup[k] = bool(parse_value(v)) if isinstance(cur, bool) else type(cur)(parse_value(v))
    speeds = [tuple(float(x) for x in p.split("/")) for p in a.speeds.split(",") if p]
    if not speeds:  # default: the race values (line_config RACE_SPEED_*), not the slower start-up speeds
        _, ex = sd.load_line_config(sd.line_config)
        speeds = [(ex["RACE_SPEED_STRAIGHT"], ex["RACE_SPEED_CORNER"])]
    if a.pairs:
        for name, (x, y) in PAIRS.items():
            sd.VARIATIONS[name] = {**sd.VARIATIONS[x], **sd.VARIATIONS[y]}
    names = list(sd.VARIATIONS)
    cfg, pilot, _ = sd.build(setup)
    print(f"SIMULATED. Steering: {'pure pursuit' if cfg.pure_pursuit else 'PD + feedforward'}, target {cfg.target_x_ratio}, "
          f"full lock {cfg.steer_max_deg} deg. {a.seeds} seeds x {a.seconds:.0f} s x {len(names)} physics variations.")
    with Pool(max(1, min(10, os.cpu_count() or 2))) as pool:
        for st, co in speeds:
            res = sd.sweep(setup, st, co, a.seconds, a.seeds, names, pool)
            t = sd.total(res)
            label = (f"straight {st:.2f} / curve {co:.2f}" if st is not None
                     else f"line_config: straight {pilot.speed_plan.straight:.2f} / curve {pilot.speed_plan.corner:.2f}")
            print(f"\n== {label}")
            print(f"{'variation':24s} {'wide':>4s} {'deep in':>7s} {'lost %':>6s} {'lap s':>6s} {'right gap cm':>12s} {'over inside %':>13s}")
            for n in names:
                r = res[n]
                print(f"{n:24s} {r['wide']:4d} {r['inside']:7d} {r['lost']:6.1f} {r['lap']:6.2f} {r['clear_right_cm']:12.1f} {r['inside_pct']:13.0f}")
            ok = t["off"] == 0
            print(f"-> {'PASS' if ok else 'FAIL'}: {t['wide']} wide, {t['inside']} deep inside, worst line loss {t['lost']:.1f}%. "
                  f"Lap {t['lap']:.2f} s nominal ({t['worst_lap']:.2f} s slowest variation); "
                  f"{a.laps} laps = {a.laps * t['lap']:.1f} s" + ("" if ok else f". Failed in: {', '.join(t['failed'])}"))


if __name__ == "__main__":
    main()
