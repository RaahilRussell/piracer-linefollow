"""Diagnoser: "what we think is wrong right now", each with the evidence and a fix.

Pure function over what the dashboard already has (latest telemetry, the last ~10 s of history,
the tuning-health verdict, the current knob values and the simulator's speed profiles), so it
runs on the Pi with no simulator. Each issue:

  {"id", "level": bad|warn|info|ok, "title", "evidence", "cause", "fix": text,
   "apply": {knob: value} or None, "apply_label": str}

Worst first. "apply" values are clipped to the knob ranges by the hub when applied.
"""
from __future__ import annotations

import math

PILOT = ("local_angle", "local")
ORDER = {"bad": 0, "warn": 1, "info": 2, "ok": 3}


def _r(x, step):
    return round(round(x / step) * step, 4)


def schedule_for(profiles, throttle):
    """Simulator starting gains for this throttle: interpolate KP/KD between the nearest profiles."""
    pts = sorted(((p["straight_cmd"], p["kp"], p["kd"], p["profile"]) for p in (profiles or {}).get("profiles", [])),
                 key=lambda p: p[0])
    if not pts or throttle is None:
        return None
    if throttle <= pts[0][0]:
        c, kp, kd, name = pts[0]
        return {"kp": kp, "kd": kd, "profile": name, "cmd": c}
    if throttle >= pts[-1][0]:
        c, kp, kd, name = pts[-1]
        return {"kp": kp, "kd": kd, "profile": name, "cmd": c}
    for (c0, kp0, kd0, n0), (c1, kp1, kd1, n1) in zip(pts, pts[1:]):
        if c0 <= throttle <= c1:
            f = (throttle - c0) / (c1 - c0) if c1 > c0 else 0
            near = n0 if f < 0.5 else n1
            return {"kp": round(kp0 + f * (kp1 - kp0), 1), "kd": round(kd0 + f * (kd1 - kd0), 2),
                    "profile": f"between {n0} and {n1}" if 0.15 < f < 0.85 else near, "cmd": round(throttle, 2)}
    return None


def diagnose(tele, hist, health, knobs, profiles=None, video_ok=True) -> list:
    """tele: latest telemetry dict (or None). hist: telemetry dicts from the last ~10 s.
    health: hub.health dict (or None). knobs: {id: value}."""
    out = []

    def add(id_, level, title, evidence, cause, fix, apply=None, apply_label=None):
        out.append({"id": id_, "level": level, "title": title, "evidence": evidence, "cause": cause,
                    "fix": fix, "apply": apply, "apply_label": apply_label})

    if not video_ok or tele is None:
        add("no_data", "bad", "No data from the car", "The dashboard isn't receiving telemetry.",
            "drive_line.py isn't running, or the Pi dropped off Wi-Fi.",
            "Check the Pi is powered and on the network, then start drive_line.py.")
        return out

    mode = tele.get("mode")
    pilot = mode in PILOT
    hz = tele.get("hz") or 0
    k = knobs or {}
    recent = [h for h in (hist or []) if h.get("mode") in PILOT]
    n = len(recent)

    # ---- camera and loop
    if tele.get("status") == "STALE":
        add("stale", "bad", "Camera frozen", f"No new frame for {tele.get('frame_age_ms', '?')} ms." if tele.get('frame_age_ms') else "No new frames.",
            "The camera stopped delivering frames (cable, or the Pi is overloaded). Throttle is forced to 0.",
            "Check the camera ribbon cable; restart drive_line.py if it doesn't come back.")
    if hz and hz < 14:
        add("slow_loop", "warn", "Control loop running slow", f"{hz:.0f} Hz (target 20). Detection takes {tele.get('proc_ms', '?')} ms.",
            "The Pi is busy: another program, recording to a slow SD card, or a hot CPU throttling.",
            "Close other programs on the Pi; check `vcgencmd get_throttled`. Slow loops make every gain less stable.")

    # ---- safety stops
    if tele.get("single_stop"):
        add("single_stop", "bad", "Stopped: only one line visible",
            f"One line for longer than {k.get('single_line_stop_s', '?')} s while rolling.",
            "The inside line left the view in a turn, or the car is following the wrong line.",
            "If it was a normal turn, allow longer before stopping.",
            {"single_line_stop_s": min(10.0, (k.get("single_line_stop_s") or 2.5) + 1.0)}, "Allow 1 s longer")

    if mode == "local" and (tele.get("throttle") or 0) >= 0.4 and (tele.get("motion") or 9) < 1.5:
        mx = k.get("max_auto")
        add("stall", "warn", "Throttle on but the car isn't moving",
            f"Throttle {tele.get('throttle', 0):.2f} (PWM {tele.get('pwm', '?')}), camera motion {tele.get('motion', 0):.1f} (still).",
            "Below the motor's start point, a flat battery, or the ESC switch is off.",
            "Check the battery and ESC switch; otherwise raise Max: Full auto a little.",
            {"max_auto": round(min(0.8, (mx or 0.55) + 0.03), 2)} if mx is not None else None, "Raise Max: Full auto +0.03")

    # ---- line tracking over the last 10 s of pilot driving
    if n >= 40:
        lost = sum(h.get("status") != "TRACKING" for h in recent) / n
        backup = sum(h.get("source") == "right" for h in recent if h.get("status") == "TRACKING") / n
        conf = [h.get("conf") for h in recent if h.get("conf") is not None]
        mean_conf = sum(conf) / len(conf) if conf else None
        if lost > 0.15:
            if mean_conf is not None and mean_conf < 0.5:
                s_lo = k.get("s_lo")
                add("lost_detect", "bad", "Losing the line: detection is weak",
                    f"Lost {100 * lost:.0f}% of the last 10 s; average confidence {mean_conf:.2f}.",
                    "The orange filter is too strict for this lighting (tape looks less saturated).",
                    "Loosen the orange filter a little and watch the camera overlay.",
                    {"s_lo": max(60, int(s_lo) - 10)} if s_lo is not None else None, "Lower Min saturation by 10")
            else:
                add("lost_speed", "bad", "Losing the line in turns",
                    f"Lost {100 * lost:.0f}% of the last 10 s while the detector itself is confident.",
                    "The car runs wide and the line leaves the camera's view: too fast, or KP too low to turn in time.",
                    "Slow down for now (and check Tuning for KP).",
                    {"max_auto": round(max(0.5, (k.get("max_auto") or 0.6) - 0.03), 2)} if mode == "local" else None,
                    "Lower Max: Full auto by 0.03")
        if backup > 0.35:
            la = k.get("lookahead_ratio")
            add("backup", "info", "Often following the right-line backup",
                f"Right line used {100 * backup:.0f}% of the time.",
                "The inside (left) line leaves the view in tight left turns. Normal in moderation; it costs accuracy.",
                "Measure a little closer to the car so the left line stays in view.",
                {"lookahead_ratio": round(min(0.95, la + 0.03), 2)} if la is not None else None, "Lookahead +0.03 (closer)")

    # ---- PID health
    if health:
        sug = health.get("suggest") or {}
        for f in health.get("findings", []):
            if f["level"] == "info":
                continue
            apply, label = None, None
            code = f.get("code")
            if code in ("weave", "lagging") and "kp" in sug and k.get("kp") is not None:
                new = _r(k["kp"] * sug["kp"], 0.1)
                apply, label = {"kp": new}, f"Set KP {k['kp']:.1f} → {new:.1f}"
            elif code == "jitter" and "kd" in sug and k.get("kd") is not None:
                new = _r(k["kd"] * sug["kd"], 0.05)
                apply, label = {"kd": new}, f"Set KD {k['kd']:.2f} → {new:.2f}"
            elif code in ("too_fast", "cant_turn", "lost") and mode == "local" and k.get("max_auto") is not None:
                new = round(max(0.5, k["max_auto"] - 0.03), 2)
                apply, label = {"max_auto": new}, f"Max: Full auto {k['max_auto']:.2f} → {new:.2f}"
            cause = {"weave": "Too much proportional gain for this speed (or too little damping).",
                     "lagging": "Not enough proportional gain: steering reacts too little to the error.",
                     "jitter": "The derivative term amplifies frame-to-frame camera noise.",
                     "too_fast": "The corners need more steering than the car has at this speed.",
                     "cant_turn": "The corners need more steering than the car has at this speed.",
                     "lost": "The line leaves the view: too fast or not turning in time."}.get(code, "")
            add("health_" + (code or "x"), f["level"], f["text"].split(":")[0],
                f"Last {health.get('seconds', 10):.0f} s: {health.get('swing_rate', 0)} swings/s, error 90% under {health.get('err_p90', 0)}, "
                f"steering at limit {round(100 * (health.get('saturated') or 0))}%.", cause,
                f["text"].split(": ", 1)[-1].capitalize() + ".", apply, label)

    # ---- gains far from the simulator's starting point for the speed being driven
    thr = None
    if mode == "local":
        thrs = [h.get("throttle") for h in recent if (h.get("throttle") or 0) > 0.3]
        thr = sum(thrs) / len(thrs) if thrs else None
    sched = schedule_for(profiles, thr)
    if sched and k.get("kp") is not None:
        dkp = abs(k["kp"] - sched["kp"]) / max(sched["kp"], 0.1)
        if dkp > 0.3:
            add("vs_schedule", "info", "Gains far from the simulator's starting point",
                f"Driving at ~{thr:.2f}: KP {k['kp']:.1f} / KD {k.get('kd', 0):.2f} vs simulator {sched['kp']:.1f} / {sched['kd']:.2f} ({sched['profile']}).",
                "The simulator's values are a modelled starting point; car-verified values beat them once you have some.",
                "Start from the simulator's values, then let the health check correct them.",
                {"kp": sched["kp"], "kd": sched["kd"]}, f"Use KP {sched['kp']:.1f} · KD {sched['kd']:.2f}")

    # ---- context
    if tele.get("web_driving"):
        add("web", "info", "Web joystick is driving", "The dashboard joystick has control.",
            "Someone is holding the web joystick or W/A/S/D.", "Let go to hand control back to the gamepad.")
    if not pilot and not out:
        add("idle", "info", "Pilot not driving",
            "Steering diagnosis needs Pilot Steers or Full Auto for about 10 s.",
            "In Manual the pilot only watches.", "Switch to Pilot Steers (gamepad A) and drive a lap.")
    if pilot and n >= 40 and not any(o["level"] in ("bad", "warn") for o in out):
        add("ok", "ok", "Nothing wrong spotted",
            f"Last 10 s: tracked {100 * (1 - sum(h.get('status') != 'TRACKING' for h in recent) / n):.0f}%"
            + (f", {health.get('swing_rate')} swings/s" if health else "") + ".",
            "", "Try a little more speed, or run Auto-Tune.")
    out.sort(key=lambda o: ORDER.get(o["level"], 9))
    return out
