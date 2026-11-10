# PiRacer line follower

A Waveshare PiRacer Pro that races the orange Waveshare mat on its own, using one fisheye camera, OpenCV and no neural network. It follows the **inside (left) orange line, counter-clockwise**, for the ESE 1110 race: 10 laps, +5 s per off-track, +10 s per manual reposition.

> **AI agents / new teammates:** read [`CHANGELOG.md`](CHANGELOG.md) first (what changed, why, and what is still unverified), then [`HANDOFF.md`](HANDOFF.md) for the rules (no Pi writes, deploys or pushes without the owner's OK).

---

## Race day: four commands

```bash
git clone https://github.com/RaahilRussell/piracer-linefollow.git   # first time only
cd piracer-linefollow
./car.sh pull     # latest code from GitHub + a full check of the exact files the car will get (no Pi needed)
./car.sh          # finds the Pi, installs if anything changed, starts the program, opens the dashboard
```

The car starts in **Manual** mode. **Nothing moves until you drive it or choose Full Auto.** On the dashboard:

1. **Measure the camera once.** Measure the lens-centre height above the floor with a ruler, and the tilt with a phone level held on the camera board. Enter both on the **Camera height** and **Camera tilt** sliders, then **Save on car**. These are the biggest unknowns: the defaults (0.12 m, 15°) are estimates.
2. Press **First slow lap**, switch to Full Auto, and watch two laps.
3. Press **Race**. That's the race setup.
4. If it runs wide, press **Safe race**. If the steering itself misbehaves (weaving, steering the wrong way), press **PD fallback**.

**Stopping:** gamepad **Y** is the emergency stop, **X** quits the program, **A** cycles the mode. The dashboard has **STOP** (Space key). The physical switch cuts battery power.

---

## First-time setup on a new Mac

```bash
python3 -m pip install -r requirements.txt -r requirements-test.txt
```

- **SSH key:** copy `~/.ssh/id_piracer` (and its `.pub`) from the Mac that set up the car. The Pi only accepts this key. Never type or send the Pi's password anywhere.
- **Same Wi-Fi:** the Mac must be on the same Wi-Fi as the Pi. `./car.sh` finds it by its SSH identity even when its IP changes.
- **Check:** run `./car.sh check`. It prints `ready` or tells you exactly what's missing.

---

## `./car.sh`: everything for the real car

| Command | What it does |
|---|---|
| `./car.sh` | Find the Pi, deploy if the code changed, start the program, open the dashboard, stay attached (Ctrl-C only detaches) |
| `./car.sh pull` | `git pull` the latest `main`, then a quick check |
| `./car.sh check` | No Pi needed: git state, SSH key, **the exact bundle the car gets** (built and imported on the Mac), the tests |
| `./car.sh check --quick` | Same, without the tests |
| `./car.sh restart` | Stop the program, install the new version, start again (use after `pull` while it's running) |
| `./car.sh status` | Is the Pi up, is the program running, which version is installed |
| `./car.sh stop` / `open` / `log` / `ssh` / `find` | Stop the program · open the dashboard · follow its output · shell on the Pi · just find the Pi |

**Safeguards built in:**
- **Bundle check:** `./deploy.sh` builds the exact bundle on the Mac and imports it before anything reaches the Pi. A missing file fails here, not on the car.
- **No mid-run installs:** it refuses to install over a running drive program.
- **No writes outside `~/linefollow`:** it never writes anywhere else on the Pi, and checks that `~/mycar` (the car's own calibration) is unchanged.
- **Stale settings are set aside:** settings saved with **Save on car** only apply if they were saved under the same `line_config.py` defaults. After a pull that changes the defaults, the old `tuning.json` is moved aside (`tuning.json.stale-…`) and the new defaults take effect.

---

## The dashboard presets

All values come from [`pi/line_config.py`](pi/line_config.py); the page has no numbers of its own. Lap times are **simulated**: the throttle-to-speed table is assumed, so the lap timer on the car is the real number.

| Preset | Straight / curve throttle | Simulated lap | Use it for |
|---|---|---|---|
| **Race** | 1.00 / 0.90 | 3.7 s | The race. Passes all 21 simulated "what if we guessed wrong" variations, including two wrong at once |
| **Safe race** | 0.92 / 0.86 | 4.1 s | If Race runs wide on the real car |
| **First slow lap** | 0.75 / 0.70 | ~6.7 s | First laps of the day (the speed the car has already driven) |
| **PD fallback** | 0.75 / 0.70, PD steering | ~6.7 s | If pure pursuit itself misbehaves |

The **Lap Tuner** page (`/`) can also climb speed lap by lap, hold each section at its limit, and time a 10-lap race with penalties. `/classic` is the older full dashboard.

---

## How it drives

```
camera frame (OV5647 fisheye, 20 fps, 240x192)
  -> orange-line detector         linetrack/detector.py    inside edge of the left line, right line as backup
  -> ground projection            linetrack/geometry.py    pixels -> metres (camera 0.12 m high, 15 deg down)
  -> pure pursuit                 pi/donkey_part.py        aim at the ride line Ld = 0.55 m + 0.30 s * speed ahead
       + delay compensation                                aim from where the car will be 0.15 s later
       + small PD correction                               10% of KP/KD in image space
  -> speed plan                   pi/donkey_part.py        straight / transition / curve throttle by measured curvature
  -> steering servo + motor       DonkeyCar (~/mycar, read-only)
```

**Safety in the loop:**
- **Stale frame:** no throttle if the frame is older than 0.35 s.
- **Line lost:** it coasts 0.5 s holding its heading, then slows to a stop.
- **Unsure which line:** if it can't tell which line it's on for 2.5 s, it stops.
- **Gamepad wins:** the gamepad always overrides Full Auto.

### The settings that matter most

| Setting (`pi/line_config.py`) | Now | Why |
|---|---|---|
| `CAM_HEIGHT_M`, `CAM_PITCH_DEG` | 0.12 m, 15° | Pure pursuit aims through these. Too high or too flat makes it run wide. **Measure them.** |
| `PURSUIT_PREDICT_S` | 0.15 s | Delay compensation. Matches the ~150 ms command-to-motion lag in real logs. 0 turns it off |
| `TARGET_X_RATIO` | 0.40 | Where the line's inside edge sits in the image. Higher = closer to the inside line |
| `PURSUIT_LD_M`, `PURSUIT_LD_GAIN_S` | 0.55 m, 0.30 s | Lookahead. Shorter fails with delay; longer cuts corners |
| `STEERING_RATE_LIMIT` | 6 /s | At 3 the car fell behind in tight curves at race pace |
| `RACE_SPEED_*`, `SAFE_SPEED_*` | 1.00/0.95/0.90, 0.92/0.89/0.86 | The Race and Safe race presets |
| `MAX_STEERING` | 1.0 | Full lock. At 0.8 the real car lost the line in the tightest curves |

Every one of these is also a dashboard slider that changes the running car on the next frame.

---

## Simulator and tests (Mac, no car)

```bash
python3 -B -m unittest discover -s tests -q   # 231 tests
python3 tools/race_eval.py --pairs            # race values x 21 physics variations (grip, delay, camera, speed...)
python3 tools/race_eval.py --speeds 0.96/0.89 --pairs --set pursuit_predict_s=0   # try other values
python3 pi/sim_dashboard.py --sim 8893        # the real dashboard driving a simulated car: http://localhost:8893
python3 -m sim view                           # 2D simulator: http://localhost:8892
python3 -m sim params                         # which car values are verified / estimated / assumed
```

What the simulator knows:
- **Verified or measured:** wheelbase, size, camera FOV, the track (traced from Waveshare's to-scale mat), the processing time.
- **Estimated:** camera mount, 18° full-lock steering angle (fitted to recorded runs).
- **Assumed:** throttle → speed table, grip, servo speed.

`race_eval` therefore tests every setup with each assumed value moved across its plausible range. Treat simulated lap times as relative, not absolute. To replace assumptions with measurements, see [`sim/CALIBRATION.md`](sim/CALIBRATION.md).

**Replay real recordings:** `python3 tools/backtest.py --frames-dir runs/<run> --dt 0.05`. It replays the detector and pilot on recorded frames. That shows tracking, not counterfactual lap times.

---

## Repository map

| Path | What |
|---|---|
| `car.sh`, `deploy.sh` | Everything for the real car (Mac side) |
| `pi/` | Runs on the Pi: `drive_line.py` (launcher), `line_config.py` (**all settings**), `donkey_part.py` (pilot + detector part), `webui.py` + `laptuner.html` + `dashboard.html` (dashboard), `autotune.py` / `lap_tuning.py` (lap tuner, lap timer) |
| `linetrack/` | Detector, PD controller, geometry (ground projection, pure pursuit, delay compensation). Shared by car, simulator and tests |
| `sim/` | Vehicle, camera and track simulator. `profile.json` holds every physical value with its source and status |
| `tools/` | `race_eval.py` (race check), `simdrive.py`, `sim_limits.py`, `calibrate_sim.py`, `calibrate_camera.py`, `backtest*.py` |
| `tests/` | Unit and simulator tests |
| `CHANGELOG.md` | What changed and why, newest first, including the comparison with Nov 9 evening |
| `HANDOFF.md` | Session handoff and rules for agents working on the car |

**Undo everything on the Pi:** `rm -rf ~/linefollow`. Nothing outside that folder is ever written.
