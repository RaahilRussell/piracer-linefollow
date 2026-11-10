"""Auto-tuner with selectable strategies. Runs while the pilot drives in Full auto.

Every `window_s` of driving it judges the window with tuning.analyze() and moves on.
Strategies:

  ladder      start slow; fix KP/KD until healthy; step the speed up; when a speed is too
              fast, keep the corners at the best speed and raise only the straights.
  scheduled   the same ladder, but at every speed it STARTS from the simulator's gain
              schedule (KP/KD interpolated for that speed), then verifies/corrects it live.
              The passes form a car-verified schedule.
  gains       hold the current speed; start from the simulator's KP/KD for that speed (if a
              schedule is loaded) and fix whatever the health check finds. It does NOT climb KP:
              the car's own error signal (at the lookahead row) keeps improving with more KP even
              when the car's real position gets worse (it cuts corners) - verified in the simulator.
  max_speed   keep KP/KD fixed; only climb speed until it isn't healthy.
  zn          Ziegler-Nichols ultimate-gain test at a fixed slow speed: KD = 0, raise KP
              until a steady weave appears (Ku, period Tu), then KP = 0.8 Ku, KD = KP * Tu / 8,
              and verify one window.

Safety (all strategies): speed changes by at most one `step`, never above `ceiling`, nothing
changes mid-window, line lost > 1.5 s ends that level, and abort() puts back the settings
it started with. It never starts on its own.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import tuning

STRATEGIES = {
    "ladder": "Speed ladder: start slow, fix gains, step speed up, then raise only the straights",
    "scheduled": "Schedule-guided ladder: start each speed from the simulator's KP/KD, verify and correct live",
    "gains": "Gains only: hold this speed, start from the simulator's KP/KD for it, fix weave/jitter/lag",
    "max_speed": "Max speed: keep KP/KD, climb speed until it isn't healthy",
    "zn": "Ziegler-Nichols: KD 0, raise KP to a steady weave (Ku, Tu), then KP 0.8Ku, KD KP*Tu/8",
    "section": "Section tune: tune only the selected straight or corner gain pair after you place the car",
    "sections": "Guided section tune: learn straight and corner gains from their detected sections",
}


@dataclass
class Level:
    phase: str
    straight: float
    corner: float
    kp: float
    kd: float
    health: tuning.Health
    verdict: str            # "pass" | "retune" | "fail" | "probe"


def interp_schedule(schedule, throttle):
    """KP/KD for a throttle from the simulator schedule [(throttle, kp, kd), ...] (linear, clamped)."""
    pts = sorted(schedule)
    if throttle <= pts[0][0]:
        return pts[0][1], pts[0][2]
    if throttle >= pts[-1][0]:
        return pts[-1][1], pts[-1][2]
    for (t0, kp0, kd0), (t1, kp1, kd1) in zip(pts, pts[1:]):
        if t0 <= throttle <= t1:
            f = (throttle - t0) / (t1 - t0)
            return kp0 + f * (kp1 - kp0), kd0 + f * (kd1 - kd0)
    return pts[-1][1], pts[-1][2]


@dataclass
class AutoTuner:
    pilot: object                       # LineFollowerPilot (its .cfg is what the car uses)
    start: float = 0.70                 # classic strategies only; the lap tuner starts from the speed plan
    step: float = 0.04                  # straight step (line_config AUTOTUNE_STEP); curves use curve_step_ratio of it
    ceiling: float = 1.00               # never above this
    window_s: float = 10.0
    settle_s: float = 2.0               # ignore this long after any change
    max_retunes: int = 3                # per speed level
    kp_range: tuple = (0.5, 12.0)
    kd_range: tuple = (0.0, 1.5)
    two_phase: bool = True
    clock: object = time.monotonic
    log: object = None                  # callable(level, text) for the dashboard log
    schedule: list = field(default_factory=list)   # [(throttle, kp, kd)] from the simulator

    # Lap tuner rules (all adjustable from the dashboard)
    curve_step_ratio: float = 0.5       # curve step = straight step x this
    lost_grace_s: float = 0.3           # line must be gone this long to count as a line failure
    min_tracked: float = 0.85           # section fails below this tracked fraction
    straight_sat: float = 0.20          # straights: share of time at full lock that means too fast
    curve_sat: float = 0.95             # curves: full lock is normal; only near-constant lock fails
    weave_fast: float = 0.9             # swings/s that count as a fast weave
    evidence: int = 40                  # samples per section before it can pass
    kp_max: float = 3.5                 # the lap tuner never raises KP above this (higher KP crashed in curves)
    confirm_laps: int = 2               # clean laps in a row needed at a speed before going faster
    margin_steps: int = 1               # a section's race speed = its last clean speed minus this many steps
    race_first: bool = True             # Start drives the race values first, then climbs (or steps down)
    min_throttle: float = 0.45          # stepping down never goes below this (the motor stalls lower)
    lap_source: object = None           # callable -> LapTimer.status() dict, for measured lap times in messages
    strategy: str = "ladder"
    section: str | None = None       # section mode: straight or corner
    stop_on_transition: bool = False
    state: str = "idle"                 # idle | running | done | aborted
    phase: str = "uniform"
    history: list = field(default_factory=list)
    best: Level | None = None
    best_uniform: Level | None = None
    verified: list = field(default_factory=list)   # scheduled strategy: passes per speed
    message: str = ""
    result: str = "not_run"

    # ------------------------------------------------------------ control
    def begin(self, strategy=None, speed=None):
        from donkey_part import SpeedPlan
        if self.state == "running":
            raise RuntimeError("Stop the current auto-tune first")
        if strategy in ("section", "sections"):
            raise ValueError("Use the guided section controls")
        self.result = "running"
        self.pilot.resume_motion()
        if strategy:
            if strategy not in STRATEGIES:
                raise ValueError(f"unknown strategy '{strategy}'")
            self.strategy = strategy
        if self.strategy == "scheduled" and not self.schedule:
            raise RuntimeError("No simulator gain schedule loaded (gain_schedule.json)")
        c = self.pilot.cfg
        self._saved = (c.kp, c.kd, c.straight_kp, c.straight_kd, c.corner_kp, c.corner_kd,
                       self.pilot.speed_plan, c.max_throttle)
        s0 = speed if speed is not None else self.start
        if self.strategy in ("gains", "zn") and speed is None and self.pilot.speed_plan is not None:
            s0 = self.pilot.speed_plan.straight              # hold the speed you're already using
        s0 = min(s0, self.ceiling)
        self.pilot.speed_plan = SpeedPlan(s0, s0)
        c.max_throttle = max(c.max_throttle, self.ceiling)
        self.state, self.history, self.best, self.best_uniform, self.verified = "running", [], None, None, []
        self.section = None
        self.stop_on_transition = False
        self.phase = {"ladder": "uniform", "scheduled": "uniform", "gains": "fix", "max_speed": "climb", "zn": "ramp"}[self.strategy]
        self._retunes = 0
        self._last_pass = None
        if self.strategy == "scheduled" or (self.strategy == "gains" and self.schedule):
            c.kp, c.kd = interp_schedule(self.schedule, s0)
        if self.strategy == "zn":
            self._zn_window = self.window_s
            self.window_s = min(self.window_s, 8.0)
            c.kd = 0.0
            c.kp = max(self.kp_range[0], min(c.kp, 2.0))
        self._reset_window()
        self._say("info", f"Auto-tune ({STRATEGIES[self.strategy].split(':')[0]}) started at speed {s0:.2f}, "
                          f"KP {c.kp:.2f} KD {c.kd:.2f}, ceiling {self.ceiling:.2f}")

    def begin_section(self, section: str, speed=None):
        """Tune only one section's gains after the user has positioned the car."""
        from donkey_part import SpeedPlan
        if section not in ("straight", "corner"):
            raise ValueError("section must be straight or corner")
        if self.state == "running":
            raise RuntimeError("Stop the current auto-tune first")
        c = self.pilot.cfg
        self._saved = (c.kp, c.kd, c.straight_kp, c.straight_kd, c.corner_kp, c.corner_kd,
                       self.pilot.speed_plan, c.max_throttle)
        current = speed
        if current in (None, ""):
            plan = self.pilot.speed_plan
            current = getattr(plan, section, self.start)
        current = min(float(current), self.ceiling)
        self.strategy, self.section = "section", section
        self.stop_on_transition = True
        self.state, self.history, self.best, self.best_uniform, self.verified = "running", [], None, None, []
        self.phase = section
        self._retunes = 0
        self.pilot.speed_plan = SpeedPlan(current, current)
        c.max_throttle = max(c.max_throttle, self.ceiling)
        self.pilot.ctrl.reset()
        self.pilot.curvature = None
        self._reset_window()
        self._guided_begin()
        self._say("info", f"Section tune ready: {section} at {current:.2f}; place the car, then watch the next {self.window_s:.0f} s")

    def _set_steps(self, step=None):
        """Straight and curve step from the rules. Curve step 0 (share 0) = curves never speed up."""
        self._sections_step = min(0.12, max(0.01, float(self.step if step is None else step)))
        self._corner_step = max(0.0, round(self._sections_step * self.curve_step_ratio + 1e-9, 2))

    def begin_sections(self, speed=None, step=None, race=None):
        """Tune both sections and raise their throttle after each clean pair of passes.
        race: {"straight", "corner", "lap_s", "total_s", "laps"} when the speed plan now holds the race
        values (webui applies them first): the tuner checks those before it climbs, and reports."""
        from donkey_part import SpeedPlan
        if self.state == "running":
            raise RuntimeError("Stop the current auto-tune first")
        c = self.pilot.cfg
        self._saved = (c.kp, c.kd, c.straight_kp, c.straight_kd, c.corner_kp, c.corner_kd,
                       self.pilot.speed_plan, c.max_throttle)
        plan = self.pilot.speed_plan
        current = getattr(plan, "straight", self.start)
        current = min(float(speed if speed not in (None, "") else current), self.ceiling)
        self.strategy, self.section = "sections", "both"
        self.stop_on_transition = False
        self.state, self.history, self.best, self.best_uniform, self.verified = "running", [], None, None, []
        self.phase = "collecting"
        self._retunes = 0
        self._set_steps(step)             # curves climb slower: they're where the car loses the line
        self._race = dict(race, state="driving") if race else None
        self._sections_safe = None
        self._frozen = {"straight": False, "corner": False}
        self._limits = {}
        self._laps = []
        self._clean_streak = 0
        self._first_retries = 0
        self._lap_health = {}
        self._lap_started = self.clock()
        self._last_segment = None
        import copy
        plan = self._saved[6]
        self.pilot.speed_plan = copy.deepcopy(plan) if plan is not None else SpeedPlan(current, current)
        self.pilot.speed_plan.straight = min(self.pilot.speed_plan.straight, self.ceiling)
        self.pilot.speed_plan.corner = min(self.pilot.speed_plan.corner, self.pilot.speed_plan.straight,
                                           self.ceiling)
        self.pilot.speed_plan.transition = min(self.pilot.speed_plan.straight,
                                               max(self.pilot.speed_plan.corner,
                                                   min(self.pilot.speed_plan.transition, self.ceiling)))
        c.max_throttle = max(c.max_throttle, self.ceiling)
        self._reset_window()
        self._guided_begin()
        p = self.pilot.speed_plan
        if self._race:
            est = (f" Estimated lap {self._race['lap_s']:.2f} s, {self._race['laps']} laps {self._race['total_s']:.1f} s "
                   f"(an estimate from the assumed speed table, no penalties)") if self._race.get("lap_s") else ""
            self._say("info", f"Driving the race values first: straight {p.straight:.2f} / curve {p.corner:.2f}.{est} "
                              f"After {max(1, int(self.confirm_laps))} clean lap(s) it climbs; if a section fails it steps down")
        else:
            self._say("info", f"Lap tuner ready: straight {p.straight:.2f}, curve {p.corner:.2f}; after both pass, "
                              f"straight +{self._sections_step:.2f} and curve +{self._corner_step:.2f}, up to "
                              f"{self.ceiling:.2f}. Each section stops climbing at its own limit while the other keeps going")

    def _guided_begin(self):
        self.pilot.resume_motion()
        self.result = "running"
        self._guided_started = self.clock()
        self._segment = None
        self._section_results = {"straight": "pending", "corner": "pending"}
        self._evidence = {"straight": 0, "corner": 0}
        self._attempts = {"straight": 0, "corner": 0}
        self._transitions = 0
        self._lost_at = None            # a stale loss time from an earlier run must not skip the grace period
        self._reset_window()

    def _race_report(self):
        """The race values just passed their clean laps: say what they are worth."""
        r = self._race
        r["state"] = "passed"
        st = self.lap_source() if self.lap_source else None
        if st and st.get("mean"):
            n, lap = r["laps"], st["mean"]
            pen = st.get("race_penalty_s", 0.0)
            r.update(measured_lap_s=lap, measured_total_s=round(n * lap + pen, 1), penalty_s=pen)
            text = (f"measured lap {lap:.2f} s, {n} laps about {n * lap + pen:.1f} s"
                    f" including {pen:.0f} s of penalties" + ("" if pen else " (none so far)"))
        elif r.get("lap_s"):
            text = f"estimated lap {r['lap_s']:.2f} s, {r['laps']} laps {r['total_s']:.1f} s (estimate, no lap timed yet)"
        else:
            text = "no lap time yet"
        self._say("info", f"Race values clean at straight {r['straight']:.2f} / curve {r['corner']:.2f}: {text}. Now climbing")

    def _guided_end(self, result, message):
        if result != "passed":
            if self.strategy == "sections" and getattr(self, "_sections_safe", None) is not None:
                attempted = self.pilot.speed_plan
                attempted_s, attempted_c = attempted.straight, attempted.corner
                self._restore_sections_safe()
                safe = self.pilot.speed_plan
                result = "passed"
                message = (f"{message}; speed step failed at straight {attempted_s:.2f} / curve "
                           f"{attempted_c:.2f}. Kept last passing throttle: straight "
                           f"{safe.straight:.2f} / curve {safe.corner:.2f}")
            else:
                self._restore()
                self.best = None
        self.result, self.state = result, "done"
        self.pilot.stop_motion()
        self._say("info" if result == "passed" else "warn", message + "; driving stopped")

    def _lap(self, result, change, plan=None):
        """One row of the lap log: the speeds and gains this lap ran with, how it went, what changed."""
        if self.strategy != "sections" or not hasattr(self, "_laps"):
            return
        c, p = self.pilot.cfg, plan or self.pilot.speed_plan
        now = self.clock()
        self._laps.append({
            "lap": len(self._laps) + 1, "straight": round(p.straight, 3), "corner": round(p.corner, 3),
            "straight_kp": round(c.straight_kp or c.kp, 2), "straight_kd": round(c.straight_kd or c.kd, 2),
            "corner_kp": round(c.corner_kp or c.kp, 2), "corner_kd": round(c.corner_kd or c.kd, 2),
            "result": result, "change": change, "seconds": round(now - getattr(self, "_lap_started", now), 1),
            "health": dict(getattr(self, "_lap_health", {}))})
        self._laps = self._laps[-60:]
        self._lap_started = now
        self._lap_health = {}

    def _save_sections_safe(self):
        import copy
        c = self.pilot.cfg
        self._sections_safe = (c.kp, c.kd, c.straight_kp, c.straight_kd, c.corner_kp, c.corner_kd,
                               copy.deepcopy(self.pilot.speed_plan), c.max_throttle, self.best)
        self._section_results = {"straight": "passed", "corner": "passed"}

    def _restore_sections_safe(self):
        c = self.pilot.cfg
        (c.kp, c.kd, c.straight_kp, c.straight_kd, c.corner_kp, c.corner_kd,
         self.pilot.speed_plan, c.max_throttle, self.best) = self._sections_safe
        self._section_results = {"straight": "passed", "corner": "passed"}

    def _advance_sections_speed(self):
        from donkey_part import SpeedPlan
        plan = self.pilot.speed_plan
        frozen = getattr(self, "_frozen", {"straight": False, "corner": False})
        if getattr(self, "_skip_lap_log", False):
            self._save_sections_safe()     # called from a section limit: the restored plan (with margin) is the safe one
        else:
            self._lap("passed", "", plan)
            self._clean_streak = getattr(self, "_clean_streak", 0) + 1
            need = max(1, int(self.confirm_laps))
            if self._clean_streak < need:
                # Not proven yet: "last good" stays the previous fully confirmed speed, so a failure
                # on the next lap rolls back to that and not to this half-checked one.
                self._laps[-1]["change"] = f"Clean {self._clean_streak} of {need} at this speed; same speed again"
                self._section_results = {"straight": "pending", "corner": "pending"}
                self._evidence = {"straight": 0, "corner": 0}
                self._attempts = {"straight": 0, "corner": 0}
                self._transitions = 0
                self._say("info", f"Clean lap {self._clean_streak}/{need} at straight {plan.straight:.2f} / "
                                  f"curve {plan.corner:.2f}; repeating before going faster")
                self._reset_window()
                return
            self._clean_streak = 0
            self._save_sections_safe()     # this speed passed all its confirm laps: it is the new last good
            if getattr(self, "_race", None) and self._race["state"] == "driving":
                self._race_report()
        s_step = 0.0 if frozen["straight"] else self._sections_step
        c_step = 0.0 if frozen["corner"] else getattr(self, "_corner_step", self._sections_step)
        straight = min(self.ceiling, round(plan.straight + s_step, 2))
        corner = min(straight, self.ceiling, round(plan.corner + c_step, 2))
        transition = min(straight, max(corner, round(plan.transition + max(s_step, c_step) / 2, 2)))
        if straight <= plan.straight + 1e-9 and corner <= plan.corner + 1e-9:
            why = ("both sections reached their limits" if all(frozen.values())
                   else "the rest reached the speed ceiling" if any(frozen.values())
                   else "both sections passed at the speed ceiling")
            return self._guided_end("passed", f"Done: {why}. Fastest passing throttle: "
                                    f"straight {plan.straight:.2f} / curve {plan.corner:.2f}")
        self.pilot.speed_plan = SpeedPlan(straight, corner, transition=transition,
                                          full_at=plan.full_at, ramp_down=plan.ramp_down,
                                          ramp_up=plan.ramp_up)
        self.phase = f"{straight:.2f}/{corner:.2f}"
        self._section_results = {"straight": "pending", "corner": "pending"}
        self._evidence = {"straight": 0, "corner": 0}
        self._attempts = {"straight": 0, "corner": 0}
        self._transitions = 0
        self.pilot.ctrl.reset()
        self.pilot.speed_plan.reset()
        held = [n for n, f in (("straight", frozen["straight"]), ("curve", frozen["corner"])) if f]
        if getattr(self, "_laps", None) and self._laps[-1]["result"] == "passed":
            ups = [f"straight +{straight - plan.straight:.2f}" if straight > plan.straight + 1e-9 else "",
                   f"curve +{corner - plan.corner:.2f}" if corner > plan.corner + 1e-9 else ""]
            self._laps[-1]["change"] = "Faster next lap: " + ", ".join(u for u in ups if u)
        self._say("info", f"Both sections passed; increasing throttle to straight {straight:.2f} / "
                          f"curve {corner:.2f}" + (f" ({' and '.join(held)} held at its limit)" if held else ""))
        self._reset_window()

    def fail_sections(self, section=None):
        """Driver flags a step. With a section ('straight'/'corner'), hold that section at its last
        passing throttle and keep climbing the other; without one, stop at the last passed throttle."""
        if self.state not in ("running", "paused") or self.strategy != "sections":
            raise RuntimeError("Start Tune both automatically before marking a speed step failed")
        if section in ("straight", "corner"):
            label = "Curve" if section == "corner" else "Straight"
            return self._section_limit(section, f"Driver marked {label.lower()} throttle too fast")
        return self._guided_end("failed", "Driver marked this throttle step failed")

    def adopt_user_gains(self):
        """The driver changed KP/KD mid-ramp: make those the gains any later rollback returns to,
        so a section limit never silently undoes a dashboard change."""
        safe = getattr(self, "_sections_safe", None)
        if self.state in ("running", "paused") and safe is not None:
            c = self.pilot.cfg
            self._sections_safe = (c.kp, c.kd, c.straight_kp, c.straight_kd, c.corner_kp, c.corner_kd) + tuple(safe[6:])
        if self.state in ("running", "paused") and getattr(self, "_saved", None) is not None:
            c = self.pilot.cfg
            self._saved = (c.kp, c.kd, c.straight_kp, c.straight_kd, c.corner_kp, c.corner_kd) + tuple(self._saved[6:])

    def report_off_track(self):
        """The car left the track (simulator, or a future on-car detector). In Tune both this is a
        limit for the section it was in (a curve if it just left one); other modes treat it as a fail."""
        if self.state != "running":
            return
        if self.strategy == "sections":
            name = self._segment or getattr(self, "_last_segment", None) or "corner"
            if name == "straight" and getattr(self, "_last_segment", None) == "corner":
                name = "corner"
            return self._section_limit(name, "went off track")
        if self.strategy == "section":
            return self._guided_end("failed", "Went off track")
        return self._judge(force_fail=True) if self._t else None

    def _section_limit(self, name, why, stop=False):
        """A section failed at the attempted throttle: put back the last passing settings, hold that
        section there, and keep raising the other one. stop=True (line lost) pauses until the
        driver puts the car back and re-selects Full Auto."""
        label = "curve" if name == "corner" else "straight"
        if getattr(self, "_sections_safe", None) is None:
            # Nothing has passed yet: the starting speed was too fast for this section.
            # Step it down and retry instead of giving up on the first lap.
            self._first_retries = getattr(self, "_first_retries", 0) + 1
            plan = self.pilot.speed_plan
            step = self._sections_step if name == "straight" else getattr(self, "_corner_step", self._sections_step)
            step = max(step, 0.01)          # stepping DOWN still works when the curve share is 0
            tried = getattr(plan, name)
            lower = round(max(self.min_throttle, tried - step), 2)
            self._clean_streak = 0
            if getattr(self, "_race", None) and self._race["state"] == "driving":
                self._race["state"] = "stepped down"
            if self._first_retries > 3 or lower >= tried - 1e-9:
                return self._guided_end("failed", f"{why} ({label}) before any speed passed, even at {tried:.2f}")
            self._lap(f"{label} limit", f"{why} before any pass: {label} {tried:.2f} → {lower:.2f}, retrying")
            setattr(plan, name, lower)
            plan.corner = min(plan.corner, plan.straight)
            plan.transition = min(plan.straight, max(plan.corner, plan.transition))
            plan.reset()
            self.pilot.ctrl.reset()
            self._section_results = {"straight": "pending", "corner": "pending"}
            self._evidence = {"straight": 0, "corner": 0}
            self._attempts = {"straight": 0, "corner": 0}
            self._transitions = 0
            self._reset_window()
            if stop:
                self.state = "paused"
                self.pilot.stop_motion()
                self._say("warn", f"{label.title()} {why} at {tried:.2f} on the first lap; retrying at {lower:.2f}. "
                                  f"Put the car back on the line and re-select Full Auto")
            else:
                self._say("warn", f"{label.title()} {why} at {tried:.2f}; retrying at {lower:.2f}")
            return
        tried = getattr(self.pilot.speed_plan, name)
        self._lap(f"{label} limit", f"{why}: {label} back to "
                  f"{getattr(self._sections_safe[6], name):.2f} and held")
        self._restore_sections_safe()
        self._clean_streak = 0
        step = self._sections_step if name == "straight" else getattr(self, "_corner_step", self._sections_step)
        last_clean = getattr(self.pilot.speed_plan, name)
        kept = round(max(self.min_throttle, last_clean - max(0, int(self.margin_steps)) * step), 2)
        if kept < last_clean - 1e-9:
            plan = self.pilot.speed_plan
            setattr(plan, name, kept)
            plan.corner = min(plan.corner, plan.straight)
            plan.transition = min(plan.straight, max(plan.corner, plan.transition))
            # the margin is part of the race settings, not a fresh unproven step
            self._sections_safe = self._sections_safe[:6] + (__import__("copy").deepcopy(plan),) + self._sections_safe[7:]
        self._frozen[name] = True
        self._limits[name] = {"failed_at": round(tried, 3), "last_clean": round(last_clean, 3),
                              "kept": round(kept, 3), "why": why}
        self._say("bad", f"{label.title()} limit: {why} at {tried:.2f}; holding {label} at {kept:.2f}")
        self._skip_lap_log = True
        try:
            self._advance_sections_speed()
        finally:
            self._skip_lap_log = False
        if self.state != "running":
            return
        if stop:
            self.state = "paused"
            self.pilot.stop_motion()
            self._say("warn", f"{label.title()} held at {kept:.2f}; next step is staged. Put the car back on the "
                              f"line and re-select Full Auto to continue")

    def finish_section(self):
        if self.state != "running" or self.strategy not in ("section", "sections"):
            raise RuntimeError("Start a section trial first")
        self._guided_judge(boundary=True)
        if self.state == "running":
            self._guided_end("insufficient", "Trial ended before both sections passed")

    def _guided_judge(self, boundary=False):
        name = self._segment
        if name not in ("straight", "corner"):
            if self.strategy == "section":
                self._guided_end("insufficient", "No confirmed section samples")
            return
        h = tuning.analyze(self._t, self._e, self._s, self._p, self._d, self._k,
                           max_steering=self.pilot.cfg.max_steering, min_samples=12,
                           min_tracked=self.min_tracked, weave_fast=self.weave_fast,
                           sat_limit=self.curve_sat if name == "corner" else self.straight_sat)
        if not h.sufficient or h.seconds < 0.5:
            if self.strategy == "section":
                self._guided_end("insufficient", "Section too short for a reliable result")
            return
        old = self.section
        self.section = name
        self.phase = name
        if hasattr(self, "_lap_health"):
            self._lap_health[name] = tuning.summary(h)
        fix = {k: v for k, v in h.suggest.items() if k in ("kp", "kd")}
        if not self._gains_matter():
            fix = {}       # pure pursuit with no PD share: KP/KD do nothing, so only the speed can be judged
        if self.strategy == "sections" and fix.get("kp", 1.0) > 1.0:
            # "Lagging" in a curve is the steady offset of turning, not a weak KP; a short straight window
            # usually clips a curve. Neither is a reason to steer harder (which caused off-tracks).
            cur_kp = self._get_active_gains_for(name)[0]
            if name == "corner" or h.seconds < 2.0 or cur_kp >= self.kp_max - 1e-9:
                fix.pop("kp")
        verdict = "fail" if h.suggest.get("speed", 0) < 0 else "retune" if (fix and not h.healthy) else "pass"
        lv, _ = self._record(verdict, h)
        self.section = old
        if verdict == "fail":
            self._section_results[name] = "failed"
            if self.strategy == "sections":
                return self._section_limit(name, tuning.summary(h))
            return self._guided_end("failed", f"{name.title()}: {tuning.summary(h)}")
        if verdict == "retune":
            self._attempts[name] += 1
            if not fix or self._attempts[name] > self.max_retunes:
                if self.strategy == "sections":
                    return self._section_limit(name, "gains did not settle within the attempt limit")
                return self._guided_end("failed", f"{name.title()} did not settle within the attempt limit")
            self.section = name
            before = self._get_active_gains()
            self._apply_fix(fix)
            after = self._get_active_gains()
            self.section = old
            label = "curve" if name == "corner" else name
            self._lap("retune", f"{label} KP {before[0]:.2f}→{after[0]:.2f}, KD {before[1]:.2f}→{after[1]:.2f} "
                                f"({tuning.summary(h).split(' -> ')[0].split(' · ')[-1]})")
            self._evidence = {"straight": 0, "corner": 0}
            self._section_results = {"straight": "pending", "corner": "pending"}
            self._transitions = 0
            self.pilot.ctrl.reset()
            if self.strategy == "section":
                # Stop before repositioning; keep a candidate, but do not claim verification.
                self.pilot.speed_plan = self._saved[6]
                self.result, self.state = "retune", "done"
                self.pilot.stop_motion()
                self._say("warn", f"Adjusted {name} gains. Reset the car and run again; driving stopped")
            else:
                self._say("warn", f"Adjusted {name}; verifying both sections again")
            return
        self._evidence[name] += h.n
        # Several independently scored short segments can supply evidence without joining raw traces.
        if self.strategy == "section" or self._evidence[name] >= self.evidence:
            self._section_results[name] = "passed"
            self.best = lv
        needed = ("straight", "corner") if self.strategy == "sections" else (self.section,)
        if all(self._section_results[s] == "passed" for s in needed) and (self.strategy != "sections" or self._transitions >= 2):
            if self.strategy == "sections":
                self._advance_sections_speed()
            else:
                self.pilot.speed_plan = self._saved[6]
                self._guided_end("passed", "Selected section passed tracking checks")

    def _guided_update(self, cmd, tracking, section, now):
        if now - self._guided_started > 600:
            return self._guided_end("insufficient", "Trial timed out")
        if section in ("straight", "corner"):
            self._last_segment = section
        if not tracking and self.strategy == "sections" and self.lost_grace_s > 0:
            if getattr(self, "_lost_at", None) is None:
                self._lost_at = now
            if now - self._lost_at < self.lost_grace_s:
                return            # a dropped frame or two is not a line failure
        if tracking:
            self._lost_at = None
        if not tracking:
            self._lost_at = None
            if self.strategy == "sections":
                blame = self._segment or getattr(self, "_last_segment", None) or "corner"
                return self._section_limit(blame, "line lost", stop=True)
            return self._guided_end("failed", "Tracking lost during trial")
        if self.strategy == "section" and self._segment is None and section != self.section:
            return
        if section == "unknown":
            if self.strategy == "section" and self._segment is not None:
                return self._guided_end("insufficient", "Geometry lost before section boundary could be confirmed")
            self._reset_window()
            self._segment = None
            return
        if self._t and now - self._t[-1] > 0.25:
            self._reset_window()  # A camera/timing gap must not join unrelated samples.
        if section != self._segment:
            if self._segment is not None:
                self._transitions += 1
                self._guided_judge(boundary=True)
                if self.state != "running":
                    return
            self._reset_window()
            self._segment = section
        if section not in ("straight", "corner"):
            return  # Unknown geometry never becomes evidence for a straight.
        if self.strategy == "section" and section != self.section:
            return
        if now - self._t_change < 0.15:
            return
        self._t.append(now); self._e.append(cmd.error); self._s.append(cmd.angle)
        self._p.append(cmd.p); self._d.append(cmd.d); self._k.append(True)
        if self.strategy != "section" and self._t[-1] - self._t[0] >= self.window_s:
            self._guided_judge()
            self._reset_window()

    def abort(self, why="stopped"):
        if self.state not in ("running", "paused"):
            return
        if self.strategy == "sections" and getattr(self, "_sections_safe", None) is not None:
            self._restore_sections_safe()
        else:
            self._restore()
        self.state = "aborted"
        self.result = "stopped"
        self.pilot.stop_motion()
        self._say("warn", f"Auto-tune {why}: original settings restored")

    def reset_section(self, section=None):
        """Forget the current sample window without changing gains or speed."""
        if self.state != "running":
            raise RuntimeError("Start a section tune first")
        if self.strategy in ("section", "sections"):
            self._guided_begin()
            self.pilot.ctrl.reset()
            self._say("info", "Trial samples reset; collecting fresh evidence")
            return
        raise RuntimeError("Reset sample is only available during a guided trial")

    def apply_best(self):
        """Keep the best result found (called when the user accepts it)."""
        from donkey_part import SpeedPlan
        if self.best is None:
            return None
        if self.strategy in ("section", "sections"):
            return self.best  # Each section's accepted gains are already installed.
        c = self.pilot.cfg
        self._set_active_gains(self.best.kp, self.best.kd)
        self.pilot.speed_plan = SpeedPlan(self.best.straight, self.best.corner)
        return self.best

    # ------------------------------------------------------------ per frame
    def update(self, mode, cmd, tracking: bool, section: str | None = None):
        """Call every loop with the pilot's latest command."""
        if self.state == "paused":
            # Waiting for the driver to put the car back; re-selecting Full Auto re-arms motion.
            if mode == "local" and not self.pilot.motion_stopped and tracking:
                self.state = "running"
                self._guided_begin()
                self.pilot.ctrl.reset()
                p = self.pilot.speed_plan
                self._say("info", f"Continuing at straight {p.straight:.2f} / curve {p.corner:.2f}")
            return
        if self.state != "running":
            return
        if mode != "local":
            return self.abort("stopped (left Full auto)")
        now = self.clock()
        if self.strategy in ("section", "sections"):
            return self._guided_update(cmd, tracking, section, now)
        if now - self._t_change < self.settle_s:
            return
        self._t.append(now)
        self._e.append(cmd.error if tracking else float("nan"))
        self._s.append(cmd.angle)
        self._p.append(cmd.p)
        self._d.append(cmd.d)
        self._k.append(bool(tracking))
        if not tracking:
            self._lost_since = self._lost_since or now
            if now - self._lost_since > 1.5:
                return self._judge(force_fail=True)
        else:
            self._lost_since = None
        if now - self._t[0] >= self.window_s:
            self._judge()

    # ------------------------------------------------------------ internals
    def _restore(self):
        kp, kd, skp, skd, ckp, ckd, plan, mx = self._saved
        c = self.pilot.cfg
        c.kp, c.kd, c.straight_kp, c.straight_kd, c.corner_kp, c.corner_kd, self.pilot.speed_plan, c.max_throttle = (
            kp, kd, skp, skd, ckp, ckd, plan, mx)
        if hasattr(self, "_zn_window"):
            self.window_s = self._zn_window

    def _reset_window(self):
        self._t, self._e, self._s, self._p, self._d, self._k = [], [], [], [], [], []
        self._lost_since = None
        self._t_change = self.clock()

    def _record(self, verdict, h):
        c, plan = self.pilot.cfg, self.pilot.speed_plan
        kp, kd = self._get_active_gains()
        lv = Level(self.phase, plan.straight, plan.corner, kp, kd, h, verdict)
        self.history.append(lv)
        return lv, f"{self.phase} {plan.straight:.2f}/{plan.corner:.2f}, KP {kp:.2f} KD {kd:.2f}"

    def _gains_matter(self):
        """KP/KD are tuned only when PD steers. Under pure pursuit they are at most a small correction, and
        the "weave" the health check sees is the image-row error that pursuit does not try to zero (it aims
        at a point 0.45 m+ ahead), so lowering KP fixes nothing: in the simulator it drove KP 3.5 -> 0.5
        and gave up below the starting speed. With pursuit, laps are judged on speed only (line kept,
        steering not pinned, no off-track)."""
        return not getattr(self.pilot.cfg, "pure_pursuit", False)

    def _get_active_gains_for(self, name):
        old, self.section = self.section, name
        try:
            return self._get_active_gains()
        finally:
            self.section = old

    def _get_active_gains(self):
        c = self.pilot.cfg
        if self.section == "straight":
            return c.straight_kp or c.kp, c.straight_kd or c.kd
        if self.section == "corner":
            return c.corner_kp or c.kp, c.corner_kd or c.kd
        return c.kp, c.kd

    def _set_active_gains(self, kp, kd):
        c = self.pilot.cfg
        if self.section == "straight":
            c.straight_kp, c.straight_kd = kp, kd
        elif self.section == "corner":
            c.corner_kp, c.corner_kd = kp, kd
        else:
            c.kp, c.kd = kp, kd

    def _clamp_gains(self):
        kp, kd = self._get_active_gains()
        self._set_active_gains(min(max(kp, self.kp_range[0]), self.kp_range[1]),
                               min(max(kd, self.kd_range[0]), self.kd_range[1]))

    def _apply_fix(self, gain_fix):
        kp, kd = self._get_active_gains()
        if "kp" in gain_fix:
            kp *= gain_fix["kp"]
            if self.strategy == "sections" and gain_fix["kp"] > 1:
                kp = min(kp, self.kp_max)
        if "kd" in gain_fix:
            kd = kd * gain_fix["kd"] if kd > 0 else (0.1 if gain_fix["kd"] > 1 else 0.0)
        self._set_active_gains(kp, kd)
        self._clamp_gains()

    def _judge(self, force_fail=False):
        c = self.pilot.cfg
        h = tuning.analyze(self._t, self._e, self._s, self._p, self._d, self._k, max_steering=c.max_steering)
        if not h.sufficient:
            return self._finish("insufficient tracking data")
        gain_fix = {k: v for k, v in h.suggest.items() if k in ("kp", "kd")}
        weaving = any(code == "weave" for _, code, _ in h.findings)
        too_fast = force_fail or (h.suggest.get("speed", 0) < 0 and not (weaving and "kp" in gain_fix))
        verdict = "fail" if too_fast else ("retune" if gain_fix and not h.healthy else "pass")
        handler_name = "gains" if self.strategy == "section" else (self.strategy if self.strategy != "scheduled" else "ladder")
        handler = getattr(self, f"_judge_{handler_name}")
        return handler(verdict, h, gain_fix, weaving, too_fast)

    # --- ladder / scheduled ------------------------------------------------------------
    def _judge_ladder(self, verdict, h, gain_fix, weaving, too_fast):
        lv, where = self._record(verdict, h)
        if verdict == "pass":
            self._say("info", f"PASS at {where}: {tuning.summary(h)}")
            self.best = lv
            if self.phase == "uniform":
                self.best_uniform = lv
            if self.strategy == "scheduled":
                self.verified.append({"throttle": lv.straight, "kp": round(lv.kp, 3), "kd": round(lv.kd, 3),
                                      "corner": lv.corner, "summary": tuning.summary(h)})
            self._retunes = 0
            return self._step_up()
        if verdict == "retune" and self._retunes < self.max_retunes:
            self._retunes += 1
            self._apply_fix(gain_fix)
            self._say("warn", f"Re-tune at {where}: {tuning.summary(h)} -> KP {self.pilot.cfg.kp:.2f} KD {self.pilot.cfg.kd:.2f}")
            return self._reset_window()
        why = "too fast" if verdict == "fail" else "gains couldn't be fixed"
        self._say("bad", f"Limit at {where} ({why}): {tuning.summary(h)}")
        return self._limit_reached()

    def _step_up(self):
        from donkey_part import SpeedPlan
        plan = self.pilot.speed_plan
        nxt = round(plan.straight + self.step, 3)
        if nxt > self.ceiling + 1e-9:
            return self._limit_reached(ceiling=True) if self.phase == "uniform" else self._finish("reached the speed ceiling")
        corner = nxt if self.phase in ("uniform", "climb") else plan.corner
        self.pilot.speed_plan = SpeedPlan(nxt, corner)
        if self.strategy == "scheduled":
            c = self.pilot.cfg
            c.kp, c.kd = interp_schedule(self.schedule, nxt if self.phase == "uniform" else corner)
        self._reset_window()

    def _limit_reached(self, ceiling=False):
        from donkey_part import SpeedPlan
        if (self.strategy in ("ladder", "scheduled") and self.phase == "uniform" and self.two_phase
                and self.best_uniform is not None and not ceiling):
            b = self.best_uniform
            self.phase = "straights"
            self.pilot.cfg.kp, self.pilot.cfg.kd = b.kp, b.kd
            self.pilot.speed_plan = SpeedPlan(round(b.straight + self.step, 3), b.corner)
            self._retunes = 0
            self._say("info", f"Corners max out at {b.corner:.2f}; now raising straight speed only")
            return self._reset_window()
        return self._finish("reached the speed ceiling" if ceiling else "found the limit")

    # --- max speed -----------------------------------------------------------------------
    def _judge_max_speed(self, verdict, h, gain_fix, weaving, too_fast):
        lv, where = self._record("pass" if verdict == "pass" else "fail", h)
        if verdict == "pass":
            self._say("info", f"PASS at {where}: {tuning.summary(h)}")
            self.best = lv
            return self._step_up()
        self._say("bad", f"Limit at {where} with these gains: {tuning.summary(h)}")
        return self._finish("found the fastest healthy speed for these gains")

    # --- gains only ----------------------------------------------------------------------
    def _judge_gains(self, verdict, h, gain_fix, weaving, too_fast):
        lv, where = self._record(verdict, h)
        if too_fast:
            self._say("bad", f"Too fast to tune at this speed: {tuning.summary(h)}")
            return self._finish("speed too high for tuning")
        if verdict == "pass":
            self.best = lv
            self._say("info", f"Healthy at {where}: {tuning.summary(h)}")
            return self._finish("gains are healthy at this speed")
        if self._retunes < self.max_retunes:
            self._retunes += 1
            self._apply_fix(gain_fix)
            kp, kd = self._get_active_gains()
            self._say("warn", f"Fixing at {where}: {tuning.summary(h)} -> KP {kp:.2f} KD {kd:.2f}")
            return self._reset_window()
        return self._finish("couldn't make it healthy at this speed")

    # --- Ziegler-Nichols -----------------------------------------------------------------
    def _judge_zn(self, verdict, h, gain_fix, weaving, too_fast):
        c = self.pilot.cfg
        if too_fast:
            self._record("fail", h)
            self._say("bad", f"Lost control during the ZN test: {tuning.summary(h)}")
            self._restore()
            self.state = "aborted"
            return self._say("warn", "Ziegler-Nichols aborted: original settings restored (try a slower speed)")
        if self.phase == "ramp":
            sustained = (h.swing_rate >= 0.5 and h.swing_period_s and h.swings >= 4
                         and (h.growth is None or h.growth >= 0.85))
            self._record("probe", h)
            if sustained or weaving:
                ku, tu = c.kp, h.swing_period_s or 1.0
                c.kp = 0.8 * ku
                c.kd = c.kp * tu / 8.0
                self._clamp_gains()
                self.phase = "verify"
                self._zn = (ku, tu)
                self._say("info", f"Steady weave at KP {ku:.2f} (period {tu:.2f} s) -> Ziegler-Nichols PD: "
                                  f"KP {c.kp:.2f}, KD {c.kd:.2f}; verifying")
                return self._reset_window()
            if c.kp >= self.kp_range[1] - 1e-9:
                return self._finish("no steady weave even at the highest KP allowed")
            c.kp *= 1.25
            self._clamp_gains()
            self._say("info", f"No steady weave yet ({tuning.summary(h)}); KP up to {c.kp:.2f}")
            return self._reset_window()
        lv, where = self._record(verdict, h)
        if verdict == "pass":
            self.best = lv
            return self._finish(f"Ziegler-Nichols gains verified (Ku {self._zn[0]:.2f}, Tu {self._zn[1]:.2f} s)")
        if verdict == "retune" and self._retunes < self.max_retunes:
            self._retunes += 1
            self._apply_fix(gain_fix)
            self._say("warn", f"ZN gains need softening: {tuning.summary(h)} -> KP {c.kp:.2f} KD {c.kd:.2f}")
            return self._reset_window()
        return self._finish("Ziegler-Nichols gains didn't settle; keep your previous ones")

    # --- end -----------------------------------------------------------------------------
    def _finish(self, why):
        from donkey_part import SpeedPlan
        self.state = "done"
        self.result = "passed" if self.best is not None else "failed"
        self.pilot.stop_motion()
        if hasattr(self, "_zn_window"):
            self.window_s = self._zn_window
        if self.best is not None:
            b = self.best
            self._set_active_gains(b.kp, b.kd)
            self.pilot.speed_plan = SpeedPlan(b.straight, b.corner)
            label = f"{self.section} " if self.section else ""
            self.message = (f"Done ({why}). Best {label}gains: KP {b.kp:.2f}, KD {b.kd:.2f} "
                            f"at speed {b.straight:.2f}")
        else:
            self._restore()
            self.message = f"Done ({why}) without a passing result: original settings kept"
        self._say("info", self.message)

    def _say(self, level, text):
        self.message = text
        if self.log:
            self.log(level, text)

    def status(self) -> dict:
        plan = self.pilot.speed_plan
        skp, skd = self.pilot.cfg.straight_kp or self.pilot.cfg.kp, self.pilot.cfg.straight_kd or self.pilot.cfg.kd
        ckp, ckd = self.pilot.cfg.corner_kp or self.pilot.cfg.kp, self.pilot.cfg.corner_kd or self.pilot.cfg.kd
        return {"state": self.state, "strategy": self.strategy, "section": self.section, "strategies": STRATEGIES, "phase": self.phase,
                "result": self.result, "section_results": getattr(self, "_section_results", {}),
                "transitions_observed": getattr(self, "_transitions", 0),
                "speed_step": getattr(self, "_sections_step", self.step),
                "corner_step": getattr(self, "_corner_step", None),
                "frozen": getattr(self, "_frozen", {}),
                "safe_straight": None if getattr(self, "_sections_safe", None) is None else round(self._sections_safe[6].straight, 3),
                "safe_corner": None if getattr(self, "_sections_safe", None) is None else round(self._sections_safe[6].corner, 3),
                "limits": getattr(self, "_limits", {}),
                "laps": getattr(self, "_laps", []),
                "race": getattr(self, "_race", None), "race_first": bool(self.race_first),
                "message": self.message, "has_schedule": bool(self.schedule),
                "straight": None if plan is None else round(plan.straight, 3),
                "corner": None if plan is None else round(plan.corner, 3),
                "transition": None if plan is None else round(plan.transition, 3),
                "kp": round(self.pilot.cfg.kp, 3), "kd": round(self.pilot.cfg.kd, 3),
                "straight_kp": round(skp, 3), "straight_kd": round(skd, 3),
                "corner_kp": round(ckp, 3), "corner_kd": round(ckd, 3),
                "levels": [{"phase": lv.phase, "straight": lv.straight, "corner": lv.corner, "kp": round(lv.kp, 3),
                            "kd": round(lv.kd, 3), "verdict": lv.verdict, "summary": tuning.summary(lv.health)}
                           for lv in self.history],
                "verified": self.verified,
                "best": None if self.best is None else {"straight": self.best.straight, "corner": self.best.corner,
                                                        "kp": round(self.best.kp, 3), "kd": round(self.best.kd, 3)}}
