# Changelog

**Agents: read this file before changing anything**, then `HANDOFF.md` for the working rules (no Pi writes, deploys, commits or pushes without asking). When you change behaviour or a default, add an entry at the top: what changed in the code, why it is better for the car, and the evidence.

Conventions:
- **sim** marks results from `tools/race_eval.py`. It drives the real detector and pilot code against a simulated car and camera, across 21 physics variations (each assumed value moved across its plausible range, plus 6 pairs with two values wrong at once).
- **real** marks anything measured on the actual car. Nothing in the Nov 10 entries is *real* yet: none of it has been driven.

---

## Nov 10 evening (2): motor PWM 420 + faster speed-up: Race mean 3.77 → 3.59 s (**real**)

**Where the lap time goes (real telemetry, Race at PWM 407).** The car spent:
- 76% of the time at the curve throttle
- 23% in the speed plan's "recovery correction" slowdown
- only 1–6% at full straight throttle

The straights are ~0.7 m each. At `SPEED_RAMP_UP = 0.4`/s the throttle needs ~0.35 s to climb from curve to straight, which is most of a straight. Curve speed is capped by steering lock (section above), not by the PWM.

**Change.**
- **Forward PWM is now ours to set.** `THROTTLE_FORWARD_PWM` in `line_config.py` (default 407, the same as `~/mycar`) is the PWM at throttle 1.0.
  - `drive_line.py` sets it on donkeycar's `PWMThrottle` after `add_drivetrain`. `~/mycar` is never written, and donkeycar's ESC start-up pulse still uses `~/mycar`'s value.
  - A dashboard slider **Motor PWM at full throttle** changes it live, with a hard range of 390–460. Stopped and reverse PWMs are never touched.
  - The car model's `pwm_full` follows the slider, so the lookahead and lap estimate stay in step.
- **Presets carry their PWM.**
  - Race: `RACE_FORWARD_PWM = 420`.
  - Safe race (`SAFE_FORWARD_PWM`), First slow lap and PD fallback: 407, the PWM their speeds were driven at.
- **Race speeds rescaled so curves keep the PWM that ran clean.** 1.00 / 0.69 / 0.64 at PWM 420. Curve 0.64 at 420 = PWM 402 = curve 0.86 at 407. Only the straights get faster.
- **`SPEED_RAMP_UP` 0.4 → 1.0**/s (the slider maximum).
- **Fixed a silent bug.** `load_line_config` only reads names listed in `_EXTRAS`, so `THROTTLE_FORWARD_PWM` / `RACE_` / `SAFE_FORWARD_PWM` are now registered there.
- **Speed table.** Added an *assumed* point, 460 → 3.5 m/s (car and simulator). Check: real laps average ~1.4 m/s at mean PWM ~402, and the table gives 1.37.
- **Simulator.**
  - `simdrive` accepts `forward_pwm`, applied to the pilot model and the simulated ESC.
  - `race_eval` uses `RACE_FORWARD_PWM` by default; `--set forward_pwm=N` overrides it.

**Real results** (same battery session, 30–40 s runs, lap timer):

| PWM | Speeds (straight / transition / curve) | Speed-up /s | Laps | Notes |
|---|---|---|---|---|
| 407 | 1.00 / 0.93 / 0.86 | 0.4 | mean 3.77 | baseline |
| 407 | same | 1.0 | mean 3.75 | no gain: straight and curve PWM are only 5 apart |
| 420 | 1.00 / 0.69 / 0.64 | 0.4 | mean 3.73 | ramp too slow to use the range |
| **420** | **1.00 / 0.69 / 0.64** | **1.0** | **mean 3.56–3.59, best 3.41** | 1 brief line loss in 21 laps |
| 435 | 1.00 / 0.53 / 0.49 | 1.0 | mean 3.59 | more backup / line losses: reaches the curves too fast (the ESC coasts, it doesn't brake) |

Deployed Race preset, final check: **10 laps 3.41–3.71 s, mean 3.59 s (35.9 s), 0 line losses.**

*sim* `race_eval --pairs` at PWM 420, 1.00 / 0.64: passes all 21 variations, 3.54 s nominal (4.18 s slowest).

**Notes.**
- At PWM 420 the manual stick is faster too: Max User 0.60 = PWM 400 instead of 392. Presets other than Race put 407 back.
- Next levers, untested:
  - the "recovery correction" slowdown (23% of the time)
  - a slightly wider ride line in curves (lower target), which could allow more curve speed before full lock

---

## Nov 10 evening: camera model corrected to 0.15 m / 8°; Race 1.00 / 0.93 / 0.86 (**real**, first real pure-pursuit laps)

**Symptom (real, run `20261110-142638`, 14:26).** At 0.75 / 0.70 with camera 0.12 m / 15°:
- laps of 18.1 s and 9.8 s, with 9–11 line losses per lap
- steering at full lock 43% of moving frames
- right-line backup ~30% of frames

Replaying the frames showed the same thing every curve:
1. The car turned in early and crossed the inside tape diagonally.
2. The tape, now under the car, was relabelled "right", and pursuit steered hard right (+0.6 to +1.0) inside a left curve.
3. The car ran wide and spent ~1.5 s at full left lock recovering. The overshoot carried into the next curve.

**Cause: the camera model was wrong.** The 15.2° pitch came from a *joint* fit (with roll −3.2°, k1 −0.076, centre −14.6 px) on stage frames. Only the pitch was carried to the car; `car_model()` doesn't read the other three at all. Checked on 738 real race-mat frames with both lines (`tools/calibrate_camera.py` cost, train / held-out):

| | edge-parallel CV (held out) | lane width | inner curve edge | line spacing at the 0.62 row (real: **0.283**) |
|---|---|---|---|---|
| 0.12 m / 15° (was) | 0.125 (0.118) | 0.19–0.20 m | 2.4 /m at 0.17 m | 0.43–0.47 |
| **0.15 m / 8° (now)** | **0.031 (0.033)** | **0.29 m** (mat 0.27–0.30) | **1.85 /m** (traced 1.7–1.8) | **0.283–0.31** |

In the simulator, rendering with the true camera (0.15 / 8) but piloting with the old model reproduces the real failure: 14–21 inside off-tracks per 30 s and 27–36% full lock. With the matching model there are 0.

**Real results after the change** (dashboard sliders, same battery session, 40 s each, lap timer):

| Speeds (straight / transition / curve) | Laps (s) | Line losses | Right-line backup | Full lock |
|---|---|---|---|---|
| 0.75 / 0.75 / 0.70 | 6.94–7.32 (5 laps) | 0 | 0% | 14% |
| 0.92 / 0.89 / 0.86 (Safe race) | 3.67–3.81, mean 3.75 | 0 | 0% | 36% |
| **1.00 / 0.93 / 0.86 (new Race)** | **3.54–3.77, mean 3.68** | **0** | 2% | 40% |
| 1.00 / 0.94 / 0.88 | 3.50–3.72, mean 3.59 | 2 brief | 11% | 40% |
| 1.00 / 0.95 / 0.90 (old Race) | 3.40–3.67 + one 5.16 | 4 laps | 21% | 47% |

At curve 0.90 the car sits at full lock with the inside line sliding out of the left of the image. It physically can't turn tighter at that speed, so it runs wide, and that's where the +5 s penalties come from. 0.86 is the clean limit. 0.88 would save ~0.9 s over 10 laps but risks a 5 s penalty.

**Changes:**
- `CAM_HEIGHT_M` 0.12 → **0.15**, `CAM_PITCH_DEG` 15 → **8** (`pi/line_config.py`, `sim/profile.json`, `CarModel` defaults). The race_eval camera variations are re-centred to 13 / 17 cm and 6 / 10°.
- `LANE_WIDTH_RATIO` 0.47 → **0.29** (real straights: 0.283). This is the right-line backup's lane width before it's learned.
- `RACE_SPEED_*` 1.00 / 0.95 / 0.90 → **1.00 / 0.93 / 0.86**. Safe race is unchanged (0.92 / 0.86), now with real numbers.
- Dashboard help text for the camera sliders and the Race / Safe presets now states the real results.
- Tests: the race lap bound is 4.2 s (*sim* 4.0 s; the simulator's speed table is ~9% slow against the real car). The lap-estimate test expects 4.05 s. Safe must be no faster than Race in curves and slower on straights. The feedforward test now pins `FF_GAIN = 0` with the new *sim* numbers (PD 30 vs PD + FF 26 off-tracks: marginal, PD-only).
- *sim* `race_eval --pairs`: 1.00 / 0.86 and 0.92 / 0.86 both pass all 21 variations (4.01 s and 4.17 s nominal).

**On the car now:** these values are set live and saved in `~/linefollow/tuning.json`. The code itself is **not deployed**. Once it is, the new defaults equal the saved values.

**Still open:** a ruler / phone-level check of height and tilt. PD share, lookahead and delay compensation were not re-tuned on the real car. Full lock at 40% in curves suggests curve speed, not steering gain, is the limit.

---

## At a glance: Nov 9 ~8 pm vs 9:31 pm vs now

| | Nov 9 ~8 pm (`99c1647`) | Nov 9 9:31 pm (`88375a3`, last push before Nov 10) | **Now (`main`)** |
|---|---|---|---|
| Steering law | PD on one image row | PD on one image row, separate straight/curve gains | **Pure pursuit on the ground-projected line + delay compensation + 10% PD correction** |
| Line target (edge position in image) | 0.40 | 0.46 (*real*: put the car on the left tape) | **0.40** |
| KP straight / curve | 3.5 / 3.5 | 4.0 / 4.5 (*real*: 4.0+ crashed in curves) | 3.5 / 3.5, applied at 10% (pursuit does the steering) |
| Max steering | 0.8 (*real*: line lost 58% of frames in tight curves) | 1.0 | 1.0 |
| Steering rate limit / smoothing | 3 /s / 0.35 | 3 /s / 0.35 | **6 /s / 0.20** |
| Camera model (height / tilt) | 0.17 m / 8° (assumed) | same | **0.12 m / 15°** (estimated from 3 sources) |
| Full-lock wheel angle in the model | 25° (placeholder) | 25° | **18°** (fitted to recorded runs), one value shared by car and simulator |
| Speeds (straight / curve) | auto-crawl, hand-driven ~0.75 | 1.00 / 0.60 | **Race 1.00 / 0.90 · Safe 0.92 / 0.86** |
| *sim* lap · 10-lap total | 7.35 s · 73.5 s | 8.40 s · 84.0 s | **3.68 s · 36.8 s** |
| *sim* variations failed (of 21) | 5 | 3 | **0** |
| Tests | 157 | 187 | **231**, all passing |
| Deploy | copy files | copy files | bundle built and imported on the Mac first; stale saved tuning set aside |

All three columns were scored in the same, current simulator, each at the speeds that version actually ran. The comparison is fair to the old setups, but it still rests on the simulator's assumed speed table and grip, so read the lap times as relative.

---

## What changed, why it is better for the car, and the theory

### 1. Steering law: PD on one image row → pure pursuit on the ground

**Before.** The detector reports where the line's inside edge crosses one image row (62% down the frame). The controller steers by `KP × (edge position − 0.40) + KD × (its rate of change)`, in pixel units.

Three problems follow from that design:
- **The gain changes with the geometry.** A one-pixel error at that row is a different distance on the ground depending on lens distortion and where in the row it falls. KP therefore means different things on straights and in curves, which is why separate straight/curve KPs were needed.
- **The controller can't tell "off the line" from "in a curve".** In a curve the edge slides across the image even when the car is exactly on its path. A one-row PD sees that as error: it either steers twice (cuts the inside) or needs a lag to settle (runs wide). The 9:31 pm setup's 4.0 / 4.5 KP and 0.46 target were attempts to fight this, and they crashed in curves on the real car.
- **No heading term.** A single offset at one row can't say whether the car is pointed along the line or across it. The D term approximates that from frame-to-frame changes, which amplifies detector noise.

**After.** Steering is computed in metres on the ground:
1. **Project the edge.** The tracked edge pixels are projected onto the ground through the camera model (`GroundModel`, fisheye with the fitted lens).
2. **Build the ride line.** The edge points are shifted sideways, along their own normal, by the offset the edge has when the car rides at its target. This is the path the rear axle should drive (`linetrack/geometry.py: pursuit_goal`).
3. **Pick the goal point** on the ride line `Ld = 0.55 m + 0.30 s × speed` ahead of the rear axle. If the edge ends sooner, which happens in a tight curve leaving the image, the farthest visible point is used.
4. **Steer by geometry.** The car turns onto the one circle that passes through the rear axle, is tangent to the car's heading, and goes through the goal: curvature `κ = 2·sin(α)/Ld = 2y/Ld²`, wheel angle `δ = atan(L·κ)` with wheelbase L = 0.1741 m (`pursuit_steer`). The angle becomes a servo command through the full-lock angle (18°).
5. **Hand it to the controller.** The command goes into the existing `SteeringController` as its feedforward (`ff_scale = 1`), so the same smoothing, rate limit, max-steering clamp, stale-frame and line-lost safety apply unchanged (`pi/donkey_part.py: LineFollowerPilot._pursuit`, `linetrack/controller.py`). If no goal can be found this frame, it falls back to plain PD for that frame.

**Why it is better for the car (theory):**
- **Curves are exact by construction.** On a circular path of radius R with the goal on it, `2y/Ld² = 1/R` for any `Ld < 2R`. The command is the path's own curvature, so a curve needs no tracking error to hold, and there's no integrator to wind up. That is precisely the case PD handled worst, and this track is 71% curve.
- **It contains the heading term PD was missing.** For small angles, a car offset `e` from the path and pointing `ψ` off it sees the goal at `y ≈ −(e + Ld·ψ)`. So pure pursuit ≈ `(2L/Ld²)·(e + Ld·ψ)`: P on offset with a built-in heading (damping) term weighted by Ld. It's the same structure as the `−Kp(y_e + K_d·sin ψ)` law the DIYRobocars racer Cycloid (a1k0n) uses. The damping comes from geometry, not from differentiating a noisy pixel signal.
- **One meaning across speeds.** The effective gain is `2L/Ld²`. Growing Ld with speed keeps the preview time (`Ld/v` ≈ 0.6 s at race pace) roughly constant, so the loop responds in the same time at 0.75 and at 1.00 throttle. This is gain scheduling done by geometry instead of by a table of KPs.
- **Physical units, one shared value.** Steering is a wheel angle mapped through one full-lock value (18°) shared by the car and the simulator (`Config.steer_max_deg`, `STEER_MAX_DEG`, `sim/profile.json`; a test ties them together). The old feedforward used a 25° placeholder and under-steered about 28%.

**What PD still does (`PURSUIT_PD = 0.10`).** 10% of KP/KD stays on, measured directly in the image. The ground projection can be biased by an imperfect camera model; the image measurement can't, so this small term corrects the model's bias.

In a curve the PD target is moved to where the edge *should* appear when the car is on its path (`GroundModel.expected_x_ratio`). Without that, PD would add a second helping of steering on top of pursuit and cut the inside.

Simulated range: 0.10–0.15 works. At 0.05 there's too little bias correction; at 0.25 PD's curve-fighting comes back.

**Evidence:** *sim*, PD's best setup is clean only to 0.75 / 0.70 (6.6 s lap) under the current camera estimate; pure pursuit is clean to 1.00 / 0.90 (3.7 s). **Caveat:** pure pursuit has never driven the real car. The "PD fallback" preset is there for that.

### 2. Delay compensation (`PURSUIT_PREDICT_S = 0.15`)

**Before.** Every steering decision used the camera frame as if it showed the present. In reality the command reaches the wheels after:
- the frame's exposure and age (up to 50 ms at 20 fps)
- processing (~33 ms)
- the PCA9685/servo update
- the servo's travel

Fitting command → image response in the Nov 8 logs gives **~150 ms (range 100–200)**.

**After.** `predict_goal` (`linetrack/geometry.py`) assumes the car keeps its current speed `v` and wheel angle `δ` for `τ = 0.15 s`.
- It integrates the kinematic bicycle along that arc: distance `s = v·τ`, yaw `θ = s·tan(δ)/L`.
- It moves the goal point into the car's frame at the arc's end (translate by the chord, rotate by `−θ`).
- Pure pursuit then aims from where the car will be when the command takes effect.
- Speed comes from the throttle → speed model; `δ` from the current steering command.
- Zero speed, `τ = 0`, or a goal that would land behind the car leave the goal unchanged.

**Why it is better for the car (theory):** a pure delay `τ` adds phase lag `ω·τ` at every frequency without changing the gain. At the loop's crossover that eats the phase margin, which shows up as late turn-in, overshoot and late unwinding.

Concretely, at 1.5 m/s on the 0.56 m inner radius (yaw rate `v/R` ≈ 2.7 rad/s), 150 ms is **22 cm of travel and ~15° of rotation**. The car would steer for a curve it entered a car-length ago and unwind for an exit it already passed, so it runs wide at exits.

Predicting the pose is the standard fix, the idea behind a Smith predictor: the controller acts on the state that exists when its action lands. A longer lookahead alone only smooths things; it doesn't remove the lag.

**Evidence:** *sim*, 1.00 / 0.92 with pairs, wide runs fall **123 → 60** and failing pairs 4 → 1 as τ goes 0 → 0.15. 0.15–0.20 is the plateau and 0.30 starts to hurt (it over-predicts). The plateau sits right at the ~150 ms measured from the real logs, which is good evidence the effect is physical rather than a simulator artefact. This is what lets the race pace pass every variation.

### 3. Camera model: 0.17 m / 8° → 0.12 m / 15°

**Before.** Height 0.17 m was a guess, and tilt 8° came from treating the wall base as the horizon.

**After.** Three independent sources:
- **Waveshare side photos:** scaled by the known 174.1 mm wheelbase, they put the lens centre at **0.118 m**.
- **The to-scale Waveshare mat diagram (3.46 mm/px):** its 0.27 m lane width, against the camera fit's lane width, implies **0.124 m**.
- **The real recorded frames:** they show the two lines 0.47 of the image width apart at the steering row. Only ~0.12 m / 15° reproduces that; 0.17 m / 8° gives 0.25. The fisheye fit to the real frames (`tools/calibrate_camera.py`) gives **15.2°**.

The wall base sits *below* the true horizon, so 8° was a lower bound.

The values changed in `pi/line_config.py`, `sim/profile.json` (marked "estimated", with sources) and the `CarModel` defaults. The `race_eval` camera variations are re-centred on 11 / 14 cm and 12 / 18°.

**Why it matters for the car (theory):** with the tilt fixed, every ground distance the projection reports scales with the assumed height. A model that is `s = 0.17/0.12 = 1.42×` too high reports a goal at `(s·x, s·y)`. Pure pursuit then commands `2(s·y)/(s·d)² = (1/s)·2y/d²`: **30% less curvature than needed, so it runs wide.** Its lookahead is also really `Ld/s`, closer than intended.

Tilt errors distort far points more than near ones, which corrupts the measured curvature. This is why the earlier pure-pursuit setup, which passed under the old model, ran wide once the camera was corrected, and why everything was re-tuned on top of the new model.

**Rule of thumb:** if unsure, err **low on height and high on tilt**. That way pursuit over-steers slightly and cuts inside, which the race allows, instead of running wide.

**Caveat:** still not measured with a ruler. That's the single most valuable 30 seconds before the race. There are dashboard sliders for it (next section).

### 4. Steering rate 3 → 6 /s, smoothing 0.35 → 0.20

**Before.** The steering command could change by at most 3 units/s, on a −1…+1 scale, after an EMA smoother with a = 0.35.

**After.** 6 /s and a = 0.20 (`STEERING_RATE_LIMIT`, `STEERING_SMOOTHING`).

**Why it is better for the car:** curve entry needs roughly 0 → full lock.
- At 3 /s that takes 0.33 s. At 1.5 m/s that's **0.5 m of travel, over half the 0.88 m quarter-turn arc** of a 0.56 m curve, so the car was guaranteed to enter wide.
- At 6 /s it takes 0.17 s. That's still slower than the servo can physically move (assumed ~300°/s ≈ 16 command units/s), so the limit smooths rather than cripples.
- The EMA's time constant (`−dt/ln a` at 20 fps) drops from ~48 ms to ~31 ms, which also cuts phase lag.

**Evidence:** *sim*, at 1.00 / 0.92, worst line loss fell from 9% to under 1% between rate 3 and rate 5–10; 6 is the smallest clean step.

### 5. Line target 0.46 → 0.40, lookahead and speeds

- **Target 0.40.**
  - Under pure pursuit the car holds its target accurately (no steady-state error in curves), so the target *is* the path.
  - 0.40 keeps the body clear of the inside tape with margin on the outside.
  - 0.42+ ran wide with extra delay in *sim*; 0.46 put the real car on the left tape.
- **Lookahead `0.55 m + 0.30 s × v`.**
  - Shorter (0.35) fails with delay: the goal is so close the loop is too stiff for a 150 ms lag.
  - A speed gain of 0.2 fails with delay; 0.4 fails at an 18° tilt (cuts too much).
- **Race 1.00 / 0.95 / 0.90 (straight / entering curve / curve).**
  - The track is 71% curve, so curve speed sets the lap time.
  - Curve 0.91 also passes every variation; 0.92 fails "fast car + low grip", so 0.90 keeps one step of margin.
  - An off-track costs 5 s, more than the whole 10-lap gain of the next step.
- **Safe race 0.92 / 0.89 / 0.86:** passes everything *without* delay compensation, so it's the fallback if compensation is switched off.
- **PD fallback 0.90 / 0.85 → 0.75 / 0.70:** under the new camera PD only stays clean up to there.

### 6. Saved tuning can no longer override a new version

**Before.** At start-up the program applies the Pi's saved `tuning.json` on top of `line_config.py`. A file saved under older defaults (the Pi's real one says target 0.46, KP 4.0) would silently beat every new default after a pull and deploy. You'd drive the old setup while the dashboard's presets described the new one. `car.sh` only caught files from before pure pursuit existed.

**After** (`pi/webui.py`):
- At start-up `Hub` fingerprints the defaults it was built from (`defaults_fingerprint`: a sorted, rounded hash of every default knob value).
- **Save on car** stamps that fingerprint into `tuning.json`.
- `load_saved` applies a file only if its fingerprint matches. Otherwise (older defaults, no stamp, unreadable) it renames the file `tuning.json.stale-<time>`, logs a warning on the dashboard, and keeps the new defaults.

**Why it is better:** a pull plus deploy always takes effect, while tuning saved on top of the *current* defaults still survives restarts. Nothing is deleted, so an old file can be restored by renaming it. The check lives in the program itself, so it works however the program is started.

### 7. Deploying is checked before it touches the Pi

**Before.** `deploy.sh` copied a hand-maintained list of files. A module added to the code but not to the list would only fail on the car at start-up.

**After.**
- `deploy.sh` has one file list (`PI_FILES`, `LT_FILES`), used both to build the bundle and to check it.
- `./deploy.sh --check` unpacks the exact bundle into an empty folder on the Mac and imports everything `drive_line.py` imports. It then builds the pilot, detector and dashboard from that bundle's `line_config.py`, and reads the race preset back.
- Every real deploy runs this first; nothing is copied if it fails. I proved it by dropping `lap_tuning.py` from the list: it failed with nothing installed.
- `./car.sh check` is a preflight that needs no Pi: git state, SSH key, the bundle, and the tests.
- `./car.sh pull` is `git pull --ff-only`, then the quick check.

**Why it is better:** the failure moves from the track, with the car in hand and a clock running, to the desk, with a clear message.

### 8. Live camera sliders, and new experiment switches

- **Camera height / tilt sliders** (`Hub.set_camera`). The pilot and detector share one `CarModel`; a slider change builds a new ground projection and swaps it in whole, so the 20 Hz loop never sees a half-built model. Presets don't touch these sliders, because they're measurements, not tuning.
- **Delay compensation slider** (`pursuit_predict_s`): 0 turns it off.
- **`CONF_SLOW_BELOW`** (off): below a detection confidence, throttle blends linearly toward curve speed, reaching it at the detector's minimum confidence (0.35), never below.
  - It scales the speed plan's *target*, not last frame's capped throttle, so it can't ratchet downwards.
  - *Theory:* a low-confidence frame is a noisy measurement, and slowing shortens the distance driven on it.
  - *sim* can't judge it, because rendered frames are always clean. Real tracking confidence is 0.7–0.85, worst 5% ~0.5, so try 0.6 on the car.

### 9. Lap tuner, lap timer and presets

Built in the earlier Nov 10 session, before this one; committed in `10d40ac`.
- **Lap Tuner page** (`pi/laptuner.html`, now `/`; the old dashboard is `/classic`).
  - Every tunable value is a live slider.
  - **Save on car** persists them.
  - Presets are built on the server from `line_config.py` (`Hub.presets`), so the page holds no tuning numbers.
- **Lap tuner strategy** (`pi/autotune.py`):
  - Climbs speed lap by lap: straights by one step, curves by half a step.
  - Needs 2 clean laps per speed, and keeps a 1-step margin at a limit.
  - A failing section steps back to its last *fully confirmed* speed and is held.
  - A brief line loss under 0.3 s isn't a failure.
  - It never retunes KP/KD under pure pursuit, because pursuit's "weave" isn't a gain problem. It used to drive KP 3.5 → 0.5.
- **Lap timer** (`pi/lap_tuning.py: LapTimer`):
  - A lap is 4 straight/curve changes.
  - Brief section flickers are debounced, and impossible lap times are rejected.
  - It pauses when motion stops, and totals 10 laps with the +5 s / +10 s penalties.
- **Curve detection thresholds** 0.25/0.9 → 0.6/1.2: on real frames this labels 30% straight / 70% curve, matching the traced track.

### 10. Simulator calibration and tooling

- **Track** traced from Waveshare's to-scale mat (`sim/tracks/waveshare_inner_line.csv`): 4.98 m lap, 71% curve, inner radius ~0.56 m. It replaces a generic stadium.
- **Full-lock angle 18°**, fitted with `tools/calibrate_sim.py` (the only angle reproducing the recorded runs), and re-checked under the new camera.
- **Every physical value has a source and status** in `sim/profile.json`: verified, measured, estimated or assumed. Assumed values carry a range, which `race_eval` sweeps.
- **New tools:** `tools/race_eval.py` (race-day robustness check), `simdrive.py` (shared driver + physics variations), `sim_limits.py`, `sim_shootout.py`, `curve_audit.py`, `calibrate_sim.py`.
- **PD gain schedule regenerated** under the new camera and shipped to the Pi (`pi/gain_schedule.json`; the Pi had the Nov 8 one).

### 11. Tests: 157 (8 pm) → 187 (9:31 pm) → 231

New tests cover:
- the delay-compensation maths (unchanged at zero speed or delay; a translation on a straight; a point on a circle stays on that circle and advances by the arc driven)
- the confidence slowdown (off by default; curve speed at minimum confidence; exactly halfway at half confidence)
- the stale-settings guard
- the Safe race preset

Fixed tests:
- The "KP too high weaves" test only raised `kp`, which the section gains shadow, so it never tested a high KP.
- Two tests had the old schedule's numbers hard-coded; they now read the shipped file.
- The feedforward test now pins the current finding: under the corrected camera, curvature feedforward makes PD *worse* (0.75: 14 → 25 off-tracks), so `FF_GAIN` stays 0.

Result: 11 failing → 0.

---

## Commits

- **`6a1d578`** (Nov 10): deploy bundle check, `./car.sh check` / `pull`, stale-tuning guard, README, this changelog, `AGENTS.md` / `CLAUDE.md`. Sections 6, 7, 11.
- **`ea0f2ec`** (Nov 10): delay compensation, Race 1.00 / 0.90, `CONF_SLOW_BELOW`, sliders. Sections 2, 5, 8. Corner speed planning (`CORNER_PLAN`) was re-checked and left off: it relies on the assumed grip and speed table, so it never limited speed. Revisit once those are measured.
- **`10d40ac`** (Nov 10): pure pursuit, camera re-fit, rate and smoothing, presets, lap tuner, timer, simulator calibration and tools, regenerated schedule, test fixes. Sections 1, 3, 4, 5, 9, 10.
- **Nov 9, 8:21–9:31 pm** (`356315e` … `88375a3`): lap tuner first version, section throttle targets, full-scale throttle, section KP 4.0 / 4.5 with target 0.46, slowing on visible bends.
- **Nov 9, up to ~8 pm** (`847d98a` … `99c1647`): OpenCV detector, PD controller, simulator, gain schedule, classic dashboard, replay tools. Full history: `git log`, `docs/HANDOFF_history.md`.

---

## Still unknown: what would make the simulator trustworthy

1. **Camera height and tilt:** ruler and phone level. Enter them on the dashboard sliders and in `sim/profile.json`. This is the biggest lever on pure pursuit (section 3).
2. **Throttle → speed table:** time the car over a known distance at throttle 0.7, 0.9 and 1.0. Every lap time and the lookahead's speed term rest on it.
3. **Full-lock angle:** measure the full-lock turning radius R; angle = atan(0.1741 / R).
4. **Grip and the real delay:** procedures in `sim/CALIBRATION.md`. Grip would let corner speed planning do its job.
5. **On the car**, same battery, 10 laps each, compared by penalty-adjusted time:
   - delay compensation 0 vs 0.15
   - `CONF_SLOW_BELOW` 0 vs 0.6
