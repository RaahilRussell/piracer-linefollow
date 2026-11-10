# Review + race-day handoff (paste this into the new session)

You're reviewing uncommitted work in `~/piracer-linefollow` (PiRacer line follower, ESE 1110). The race is **Nov 11, 2026**: 10 autonomous laps on the orange Waveshare 3 × 2 m mat. It runs counterclockwise, following the inner/left orange line. Off-track costs +5 s; a manual reposition costs +10 s. A good human lap is about 3.4 s. Read `HANDOFF.md` first, including the "2026-11-09 night update" section and its Rules: no Pi access, deploys, commits or pushes without asking me, and never type passwords.

## Goals, in order

### 1. Audit every changed file
Go through `git status` and `git diff`, plus the untracked files: `pi/laptuner.html`, `sim/tracks/`, `tools/calibrate_sim.py`, `tools/curve_audit.py`, `tools/sim_limits.py`, `tools/sim_shootout.py`. For each change, confirm it is intentional, correct and safe on the real car. List anything accidental, dead, duplicated or risky, then fix it. Some files may have been edited by hand after the last session. Treat the files on disk as the current version, and flag anything that looks wrong rather than silently reverting it.

Pay particular attention to:
- **`pi/autotune.py`, the `sections` strategy (the lap tuner).** Check:
  - rollback, holding a section at its limit, the safety margin, confirm laps, first-lap retries, and pause/resume when the line is lost
  - the 0.3 s line-loss grace, the KP cap, and that curve "lagging" never raises KP
  - the lap log
- **`pi/webui.py`.** Check:
  - the `tune_*`, `lap_sections` and `race_laps` knobs
  - `attach_autotuner` and the saved-settings load order
  - `adopt_user_gains`
  - the `/` (new page) and `/classic` (old page) routes
  - the `/api/laps` endpoints
  - that the lap timer pauses while motion is stopped
- **`pi/donkey_part.py`.** The curve exit hold (`corner_exit_steer`, default off).
- **`pi/tuning.py`.** The new `analyze()` thresholds (`min_tracked`, `sat_limit`, `weave_fast`).
- **`pi/lap_tuning.py`.** The `LapTimer`: a lap is N straight/curve changes, default 4. Lap times jumped around in the simulator, so make lap detection robust: debounce brief flickers between straight and curve, and reject impossible lap times.
- **`pi/line_config.py` and `linetrack/config.py` defaults.** Target 0.40, KP 3.5, KD 0.4, max steering 1.0 (a hard standard), curve detection thresholds 0.6/1.2, coast 0.5 s, speeds 0.70/0.65.
- **`sim/`.** The traced track (`sim/tracks/waveshare_inner_line.csv`), the fitted 18° full-lock steering angle, `make_track`, and the generalised start position in `reset`. Also the new `"estimated"` provenance status, and the tests pinned to the old setup through `legacy_profile` / `tight_track`.
- **`pi/laptuner.html`.** The page is meant to be usable by anyone straight away, and I edit it by hand. Every tunable value must stay a slider that saves to the Pi; nothing tunable may be hard-coded. Check every preset and slider against the knob IDs in `webui.py`, and confirm that `deploy.sh` ships the page.
- **Tests.** Run `python3 -B -m unittest discover -s tests -q`. 16 tests were failing before this work; now 11 fail, all of which were already failing. Fix or justify each one. No new failures.

### 1b. Pure pursuit and lookahead: review heavily
This is the steering core. What exists today is **not textbook pure pursuit**; it's PD on a single image row plus curvature feedforward:
- **Lookahead:** the error is measured on one image row, `lookahead_ratio` 0.62 (`linetrack/config.py:32`). `lookahead_from_points` uses the measured edge near that row (`linetrack/detector.py` around lines 407–415, 587 and 615–634).
- **Bend ahead:**
  - `LineDetectorPart._bend` (`pi/donkey_part.py:120`) projects the tracked edge onto the ground with `GroundModel` and fits curvature with `fit_bend` (`linetrack/geometry.py:151`).
  - The pilot then smooths it in `_curvature`.
  - That curvature drives the straight/curve label (`_confirmed_section`), gain blending (`_section_gains`) and the speed plan.
- **"Pure pursuit" / feedforward** (`pi/donkey_part.py` around lines 399–407):
  - `ff = steer_for_curvature(model, k)` = atan(wheelbase × k) ÷ `steer_max_deg` (`linetrack/geometry.py:199`).
  - The PD target is shifted toward `GroundModel.expected_x_ratio` (`geometry.py:103`) by `ff_gain`.
  - It's added as `ff_gain * ff` in `SteeringController.update` (`linetrack/controller.py` around lines 51–83). The slider is `ff_gain` ("Pure pursuit (curve feedforward)").
- **Known bug: the feedforward uses the wrong steering angle.** `STEER_MAX_DEG = 25.0` in `pi/line_config.py:50` (the car side), but the simulator is now fitted to **18°**. Also, `tools/sim_limits.py`, `tools/sim_shootout.py` and `tools/calibrate_sim.py` build a `LineFollowerPilot` without setting `pilot.model`, so they get `CarModel()`'s default 25°. So the feedforward under-steers by about 28%. That probably explains why "pure pursuit didn't help". Fix it with one source of truth for the full-lock angle, shared by car and simulator; make it a slider as well. Then re-test.
- **Then evaluate real pure pursuit.** Take a lookahead point on the ride line (the inner-line edge offset by the target), projected to the ground with the existing `GroundModel`. Steer with δ = atan(2·L·sin α / Ld). Make Ld adjustable and optionally speed-scaled (Ld = Ld0 + k·v). Map δ to a command with the fitted full-lock angle, and keep PD as a small correction.
  - Compare against PD + feedforward on the calibrated simulator: many seeds, the physics sweep, lap time and off-tracks.
  - Also compare on recorded frames with `tools/curve_audit.py` / `tools/backtest.py`: line identity, steering jumps, curve-exit behaviour.
  - Ship whichever is better as the default. Keep the other selectable, with every parameter as a slider. Explain the choice with numbers.
- **Also check:**
  - camera model assumptions in `GroundModel` (height 0.17 m, pitch 8°, 160° fisheye): curvature correctness at 0.46–0.76 m radius
  - `curvature_smoothing` lag at curve entry and exit
  - whether the 0.62 row is the best lookahead at race speed
  - whether `corner_exit_steer` (off) should be on

### 2. Plug-and-play race values
Produce one set of values that should work tomorrow without tuning and give a fast time. Base it on:
- **Known good on the real car (Nov 9 morning):** about 0.70–0.75 throttle, KP 3.5, target 0.40.
- **Calibrated simulator** (`python3 tools/sim_limits.py`): curves clean to 0.95, straights to 1.00, about a 3.4 s simulated lap. Pure pursuit (`ff_gain`) didn't help on this track.
- **What broke on the real car:** target 0.46 and KP 4.0 or more crashed in curves. At max steering 0.8 the line was lost in tight curves.
- **Track:** 71% curve, so curve speed dominates lap time.

Keep the safety margin in mind: "barely ever leave the line" matters more than the last tenth of a second. Verify the chosen set over many seeds and longer runs using `tools/sim_shootout.py` / `tools/sim_limits.py`. Also check robustness: vary the assumed physical values (steering angle 16–20°, grip, delay, speed table) and require 0 off-tracks across them. Put the result in `pi/line_config.py` as the defaults and as a clearly named **Race** preset on the page.

### 3. Lap tuner tests the race values first
Change the lap tuner so that pressing Start does this:
1. It drives the race values first.
2. It reports the estimated lap time, and the estimated 10-lap total including any penalties.
3. Only then does it climb from there, or step down if a section fails.

Show the estimated lap time on the page for the current settings before driving. Estimate it from the traced track length and curve share plus the throttle-to-speed table. Label it clearly as an estimate, and replace it with real lap-timer laps once they exist. Add tests for this.

### 4. Report back
Give me:
- a file-by-file table: what changed, intentional (yes/no), what you fixed
- the final race values with simulated lap time and 10-lap total
- what is still unverified on the real car
- a short race-morning checklist: deploy (needs my OK), lifted-wheels check, press Race, save on car, a first slow lap, then the lap tuner

Don't commit, push or touch the Pi until I say so.
