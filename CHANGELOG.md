# Changelog

Newest first. **Agents: read this file before changing anything**, then `HANDOFF.md` for the working rules. Every number marked *sim* comes from the simulator (`tools/race_eval.py`, 21 physics variations incl. two-unknowns-wrong pairs); nothing here has been driven on the real car unless it says so.

## Where things stand (Nov 10, ~02:30)

| | Nov 9 ~8 pm (`99c1647`) | Nov 9 9:31 pm (`88375a3`, last push before tonight) | **Now (`main`)** |
|---|---|---|---|
| Steering law | PD on one image row | PD, section gains | **Pure pursuit + delay compensation (0.15 s) + 10% PD** |
| Target (edge position in image) | 0.40 | 0.46 (crashed in curves on the real car) | **0.40** |
| KP straight / curve | 3.5 / 3.5 | 4.0 / 4.5 | 3.5 / 3.5 (pursuit does the steering; PD is a 10% correction) |
| Max steering | 0.8 (lost the line in tight curves) | 1.0 | 1.0 |
| Steering rate limit | 3 /s | 3 /s | **6 /s** |
| Camera model (height / tilt) | 0.17 m / 8° (assumed) | 0.17 m / 8° | **0.12 m / 15°** (estimated, see below) |
| Full-lock angle in the model | 25° (placeholder) | 25° | **18°** (fitted to recorded runs) |
| Speeds (straight / curve) | auto-crawl, driven by hand ~0.75 | 1.00 / 0.60 | **Race 1.00 / 0.90 · Safe 0.92 / 0.86** |
| *sim* lap · 10 laps | 7.35 s · 73.5 s | 8.40 s · 84.0 s | **3.68 s · 36.8 s** |
| *sim* variations failed (of 21) | 5 (16° lock, delay, slow servo pairs) | 3 (delay pairs) | **0** |
| Tests | 157 | 187 | **231**, all passing |
| Deploy safety | copy files | copy files | bundle built + imported on the Mac first; stale `tuning.json` set aside |

All three columns were scored in the **same** simulator (today's calibrated one, at the speeds each version actually ran), so they compare like with like. In short: about **2× faster per lap in simulation and robust to every tested modelling error**, against setups that failed several. Real-car times are still unknown, and the throttle-to-speed table is assumed.

---

## 2026-11-10: deploy hardening, README, this changelog
- **`./car.sh check`**: a preflight that needs no Pi. It checks:
  - git state, and whether GitHub `main` is ahead
  - the SSH key and the Pi's host key
  - the **exact deploy bundle**, built and imported on the Mac
  - the test suite (`--quick` skips the tests)
- **`./car.sh pull`**: `git pull --ff-only origin main`, then the quick check.
- **`./deploy.sh --check`**: one file list (`PI_FILES`, `LT_FILES`). Every deploy unpacks the bundle into an empty folder on the Mac and builds the pilot, detector and dashboard from it, like `drive_line.py` does. A missing file fails before anything reaches the Pi (tested by dropping `lap_tuning.py` from the list).
- **Stale saved settings fixed.** `tuning.json` overrides `line_config.py` at start-up. Until now, a file saved under older defaults (e.g. target 0.46, KP 4/4.5) silently beat every new default after a pull and deploy.
  - **Save** now stamps a fingerprint of the defaults it was made under (`webui.defaults_fingerprint`).
  - **Load** moves a mismatched or unstamped file to `tuning.json.stale-<time>` and logs it.
  - The old `car.sh` special case (only for files from before pure pursuit) is gone.
- **New `README.md`** (race day in four commands), **`AGENTS.md` / `CLAUDE.md`** (point agents here).
- Tests: 228 → 231 (stale-settings guard).

## 2026-11-10 `ea0f2ec`: delay-compensated pure pursuit, race pace 1.00 / 0.90
- **Delay compensation (`PURSUIT_PREDICT_S = 0.15`).**
  - **What it does:** the camera frame shows where the car *was*. Pure pursuit now moves its goal point into the frame the car will be in 0.15 s later, assuming it holds its current speed and wheel angle on that arc (`linetrack/geometry.py: predict_goal`).
  - **Why 0.15 s:** it matches the ~150 ms command-to-visible-motion lag fitted from the Nov 8 logs.
  - **Result** (*sim*, 1.00 / 0.92, pairs): wide runs 123 → 60, failing pairs 4 → 1. The benefit flattens from 0.15–0.20 s, and 0.30 is worse.
- **Race preset 1.00 / 0.95 / 0.90**, *sim* 3.68 s, passes all 21 variations (5 seeds × 40 s). 0.91 also passes; 0.92 fails "fast car + low grip", so 0.90 keeps one step of margin.
- **`CONF_SLOW_BELOW`** (off): blends throttle to curve speed when detection confidence drops. It has no effect in the simulator (clean rendered frames), so judge it on the car. Real tracking confidence is 0.7–0.85; try 0.6.
- **Checked and left off:** corner speed planning (`CORNER_PLAN`). No effect, because it relies on the assumed grip and speed table. Revisit once those are measured.
- Dashboard sliders: Delay compensation, Slow when detection below.

## 2026-11-10 `10d40ac`: camera re-fit, 3.5 s pace, lap tuner and race tools
- **Camera re-estimated to 0.12 m / 15°** (was an assumed 0.17 m / 8°). Three independent sources agree:
  - **Waveshare side photos:** scaled by the 174.1 mm wheelbase, they put the lens at 0.118 m.
  - **The to-scale Waveshare mat (3.46 mm/px):** its 0.27 m lane, together with the camera fit's lane width, implies 0.124 m.
  - **Real frames:** they show the two lines 0.47 of the image width apart at the steering row, which only ~0.12 m / 15° reproduces (0.17 m / 8° gives 0.25). The fisheye fit to the real frames gives 15.2°.

  Under the old model the then-race-config ran wide at race speed in *sim*. The stock-mount photo shows 22–28°, but this car's frames show a flatter mount. **Still not measured.**
- **Full-lock angle re-fit under the new camera:** 18° remains the only angle that holds the line at 0.70 (16° goes off, 20° saturates less).
- **Pure pursuit re-tuned:** target 0.44 → 0.40, Ld0 0.45 → 0.55 m, PD share 0.15 → 0.10, steering rate 3 → 6 /s.

  Robust ranges with the corrected camera:

  | Setting | Works | Fails |
  |---|---|---|
  | Target | 0.38–0.40 | 0.42+ runs wide with delay |
  | Ld0 | 0.45–0.55 m | 0.35 fails with delay |
  | Ld gain | 0.3 s | 0.2 fails with delay, 0.4 at 18° tilt |
  | PD share | 0.10–0.15 | 0.05 and 0.25 fail |
- **Presets:**
  - Race 1.00 / 0.92: 3.5 s, passes the one-wrong-value variations; superseded by `ea0f2ec`.
  - **New Safe race** 0.92 / 0.86: 4.1 s, passes the pairs too.
  - PD fallback 0.90 / 0.85 → **0.75 / 0.70**: PD fails above that under the new camera.
- **Camera height / tilt sliders:** they change the projection live, rebuilding the ground model atomically, so the car loop never sees a half-built model. Presets don't overwrite them.
- **PD gain schedule regenerated** (`sim/gain_schedule.json` → `pi/gain_schedule.json`; the Pi had been shipping the Nov 8 one).
- **`race_eval` camera variations re-centred** on the estimate: 11 / 14 cm, 12 / 18°.
- **Added:** `pi/laptuner.html` (Lap Tuner page), `tools/race_eval.py`, `simdrive.py`, `sim_limits.py`, `sim_shootout.py`, `calibrate_sim.py`, `curve_audit.py`, the traced track `sim/tracks/`, and `car.sh`.
- **Test fixes:**
  - The weave test only raised `kp`, which the section gains shadow, so it never tested a high KP.
  - The schedule tests now read the shipped file instead of hard-coded numbers.
  - The feedforward test now pins the new finding: curvature feedforward makes PD worse under the new camera (0.75: 14 → 25 off-tracks), so `FF_GAIN` stays 0.
  - Suite: 11 failing → 0.

## Before tonight
- **Nov 9, 8:21–9:31 pm** (`356315e` … `88375a3`):
  - lap tuner with section throttle targets, full-scale throttle, live camera
  - section steering (KP 4.0 / 4.5, target 0.46)
  - slowing on visible bends
  - curve trials judged at the exit
- **Nov 9, up to ~8 pm** (`847d98a` … `99c1647`):
  - OpenCV detector and PD controller
  - the simulator and gain-schedule tuner
  - speed profiles
  - the classic dashboard
  - replay and backtest tools

  Full history: `git log`, and `docs/HANDOFF_history.md`.

---

## Still unknown (do these to make the simulator trustworthy)
1. **Camera height and tilt:** ruler and phone level. Enter them on the dashboard sliders and in `sim/profile.json`.
2. **Throttle → speed table:** time the car over a known distance at throttle 0.7, 0.9 and 1.0. Every simulated lap time rests on this.
3. **Full-lock angle:** measure the full-lock turning radius R; angle = atan(0.1741 / R).
4. **Grip** (`mu_lat`) and the **real delay**. `sim/CALIBRATION.md` has the procedures.
5. **On the car:** A/B delay compensation (0 vs 0.15) and `CONF_SLOW_BELOW` (0 vs 0.6). Same battery, 10 laps each, compared by penalty-adjusted time.
