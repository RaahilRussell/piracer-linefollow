# ~/linefollow: orange left-line follower (on the Pi)

Self-contained. It reads the car's settings from `~/mycar` but **never writes there**, and nothing else on the Pi was changed.

| File | What it is |
|---|---|
| `drive_line.py` | Launcher. Reuses `~/mycar/manage.py`'s camera, web UI, gamepad and drivetrain helpers (imported, not edited). |
| `line_config.py` | All line-follower settings (target, gains, throttle, safety). Edit this, not `myconfig.py`. |
| `donkey_part.py` | `LineDetectorPart` (every mode, draws the web UI overlay) and `LineFollowerPilot` (pilot modes). |
| `linetrack/` | Detector and PD controller (same code as on the Mac). |

## Offline check (no camera, no motors)
```bash
cd ~/linefollow && ~/env/bin/python drive_line.py --check frames
```

## Drive
Stop any running `manage.py drive` first (only one program can own the camera and PWM).
```bash
cd ~/linefollow && ~/env/bin/python drive_line.py --js
```
Web UI: `http://<pi-ip>:8887`. It shows the detection overlay: green = tracked left edge, cyan = target, amber = right line (fallback).

Gamepad:
- **Y = EMERGENCY STOP** (back to user mode, throttle 0).
- **A = mode**: user → local_angle → local → user.
  - `user`: you drive. The overlay still shows what the pilot sees.
  - `local_angle`: the pilot steers and you control throttle. Use this first.
  - `local`: the pilot steers and drives at `LINE_THROTTLE` (0.20, capped at 0.30).

Safety: pilot throttle is 0 whenever the line is LOST or the camera frame is STALE (>0.35 s old). After 0.6 s without the line, steering re-centres.

## Dashboard (http://<pi-ip>:8890)
Starts with `drive_line.py`. It shows:
- **Live:** the camera with the tracked line drawn on, and live charts of error, P, D, steering, throttle and camera motion.
- **Speed limits:** overall top speed plus a max for each mode (User / Pilot steers / Full auto), each with its PWM, and Crawl/Slow/Medium presets. The same settings are `TOP_THROTTLE`, `MAX_USER`, `MAX_PILOT_STEERS` and `MAX_AUTO` in `line_config.py`.
- **Tune:** knobs that change the running car on the next frame. Save writes `tuning.json`; Revert goes back to `line_config.py`.
- **Section recovery tune:** position the car in Manual, select Full Auto and click **Tune straight** or **Tune corner** twice. A confirmed section boundary ends the trial and stops driving. Too-short samples report insufficient data. A suggested gain change stops the trial for another placement and verification. **End trial & stop driving** finishes the current trial manually. **Tune both automatically** preserves the configured speed plan and evaluates each contiguous segment independently; gain changes require both sections to pass again. This checks tracking behavior, not physical lane clearance or fastest lap time.
- **Gamepad priority:** raw physical sticks override Full Auto to Manual in the same control loop; reverse remains available. Physical input cancels the dashboard joystick. In Pilot Steers, normal human throttle stays active, while steering input or reverse takes full Manual control. Auto stays off after a trial until explicitly selected again.
- **Stops:** dashboard STOP zeros throttle in both directions and returns to Manual. Quit Program stops and exits; gamepad X does the same. Shut Down Pi stops and shuts down the computer. The car's physical switch is still needed to disconnect battery power. Gamepad Y retains the controller's emergency stop.
- **Replay speed transitions:** `python3 tools/backtest.py --frames-dir /path/to/run --variants baseline --speed-straight 0.60 --speed-corner 0.55 --speed-transition 0.57 --json /tmp/replay.json`. Geometry is connected to the actual pilot; JSON includes section frame counts and throttle changes. These are replayed control outputs, not counterfactual physical lap times.
- **Section-aware speed:** Full Auto can use separate straight, transition, and corner throttle targets. Curvature starts the slowdown before steering peaks, and the ramp limits make entry/exit gradual. Replay first: the curvature estimate must be early and stable on the real track before enabling this on the car.
- **Lap speed sliders:** Tune now exposes straight, corner, transition, slowdown and acceleration sliders directly. Small straight corrections leave speed available; large steering demand or image error asks for the lower recovery speed even on a straight. Corner/transition targets are kept within the straight-speed ceiling. Drive shows why the speed target is reduced. Changing speeds during a trial stops it so old-speed samples cannot certify new settings. On-Pi edits apply live; Mac edits affect only simulation/replay. Save Tuning persists settings.
- **One driving console:** Drive & tune brings camera, manual touchpad, section speed/PD sliders, slow/faster presets, guided trials, live checks, recommended fixes and Save together. Lookahead, model tools, charts and logs are expandable. Physical sticks retain priority. The touchpad explicitly takes Manual control before sending steering/throttle.
- **One-button recording review on the Mac:** launch `python3 -B pi/review_dashboard.py --pi CURRENT_PI_IP`, then open `http://localhost:8899`. This proxies the Pi's actual camera and controls while running analysis locally. **Download & analyze recordings** copies `/home/pi/linefollow/data` through the existing SSH key into a new `analysis_runs/<job>/` folder. It runs inventory, all-frame baseline/selected-setting replay, per-section KP/KD model comparisons, a data-informed dropout/timing stress hypothesis, rendered camera tests and uncertain-car tests. Progress and a downloadable JSON report appear on the same screen. No settings are applied by analysis. Cancel keeps partial copies. Never open dashboard.html with `file://` for operation; use the served HTTP console.
- **Evidence limits:** replay tracking percentage is not lap-time improvement. Run-level held-out reporting is provided when multiple runs exist; no independent validation is claimed from a single run. Timing/dropout hypotheses are kept separate from calibrated physical parameters. Camera/track geometry, steering angle, speed response, grip and braking still need measurements. Reports do not change simulator calibration or car configuration.
- **Log:** line lost and found, backup line, stalls, camera freezes, mode changes, stops and knob changes, saved in `~/linefollow/logs/`. There are also live tracking-error tiles (average and worst over 10 s).
- **Drive from here:** a dead-man web joystick (keys W/A/S/D), plus mode buttons.
- **Stop buttons:**
  - STOP (Space) keeps the program running.
  - Quit program needs a restart.
  - Shut down Pi powers it off.
- **Past runs:** every recording, with its problems listed, a timeline and frame-by-frame playback.

Try it on the Mac with no car: `python3 pi/sim_dashboard.py runs/<run>` and open http://localhost:8890.

## Simulator (on the Mac)
```bash
python3 -m sim view          # interactive 2D simulator: http://localhost:8892
python3 -m sim params        # what's verified / measured / still assumed
python3 -m sim.tune          # recompute the per-speed KP/KD schedule (dashboard presets)
python3 pi/sim_dashboard.py --sim 8891   # this dashboard, driving the simulated car
```
Measure the car with `sim/CALIBRATION.md` before trusting the faster profiles (marked ⚠ on the dashboard).

## Backtesting a change (on the Mac)
```bash
python3 tools/backtest.py                 # current line_config vs every variant, on all recorded data
python3 tools/backtest.py --variants old,baseline

# Replay a new camera folder (for example, tomorrow's 15k frames)
python3 tools/backtest.py --frames-dir /path/to/frames --dt 0.05 --datasets new_frames \
  --variants baseline,confirm3,swap+coast --json /tmp/linefollow-backtest.json
```
Add a variant to `VARIANTS` in `tools/backtest.py`. Turn something on in `line_config.py` only if it improves `wrong`/`stops` without hurting `tracked%`.

## Test order
1. **Wheels lifted**, `local_angle`, no throttle. Move the car by hand. If the edge is right of the cyan line, the wheels must turn **right**. If mirrored, set `INVERT_STEERING = True` in `line_config.py`.
2. On the track in `local_angle`: you control throttle; tune `KP`, `KD`, `LOOKAHEAD_RATIO`.
3. `local` at low `LINE_THROTTLE`, someone holding the gamepad with a thumb on Y.

## Revert (removes everything we added)
```bash
rm -rf ~/linefollow
```
