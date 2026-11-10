# Calibrating the simulator to our PiRacer Pro

The simulator reads every physical number from `sim/profile.json`. Each one is marked:

- **verified:** a manufacturer specification, for example the 174.1 mm wheelbase.
- **measured:** taken from this car, either its `~/mycar/myconfig.py` or our own tests.
- **assumed:** a placeholder for something nobody has measured yet. Each has a range, and the tools sweep it rather than trust it.

The steps below turn each **assumed** value into a **measured** one. After each step:

1. Edit the entry in `sim/profile.json`.
2. Set its `"value"`, set `"status": "measured"`, and change `"source"` to say how and when you measured it.
3. Run the checks:
   ```bash
   python3 -m sim params          # check what's still assumed
   python3 -m sim.tune            # recompute the gain schedule on the measured car
   ```

**Safety:** do every driving step in **User** mode, with someone holding the gamepad (Y = E-stop) or the dashboard STOP. Never change `~/mycar/myconfig.py`. To get a fixed throttle for a test, set the dashboard's **Max: User mode** to the value you want and push the stick fully forward.

You'll need a tape measure, masking tape, a phone that films slow motion (240 fps is ideal), and a stopwatch.

---

## 1. Track geometry: `track_straight_m`, `track_radius_m`, `lane_width_m`, `tape_width_m`
- **Tape width:** measure across one orange tape.
- **Lane width:** measure from the inside edge of the left tape to the inside edge of the right tape, square to the tape. Take it in 3 places and average.
- **Turn radius:** measure the left (inside) line's radius in each turn: half the distance straight across the turn, measured along the left tape's centre.
- **Straight length:** measure the left tape's straight between where the two turns start.

If the track isn't a simple oval, mark a point on the left tape every 20 cm and measure each one's (x, y) from a corner of the stage. Load the points with `Track.from_points`.

## 2. Steering map: `steer_angle_table`, the most important one
A steering **command** isn't an angle. Donkeycar maps the command linearly onto a PWM pulse: −1 → 450 (`STEERING_LEFT_PWM`), +1 → 300 (`STEERING_RIGHT_PWM`), so 0 → 375. What the wheels then do depends on the servo and linkage.

1. **Find straight-ahead.** Drive slowly for 3 m at command 0. If the car curves, the true straight PWM isn't 375. Find the command that drives straight; its PWM is `375 − 75 × command`.
2. **Measure the turning radius at several commands:** −1.0, −0.75, −0.5, −0.25, +0.25, +0.5, +0.75, +1.0.
   - Drive a slow full circle (crawl speed, so nothing slides).
   - Mark the rear-wheel centre's path with tape at a few points, and measure the circle's diameter `Dc`.
   - Compute `R = Dc / 2` and `angle = atan(0.1741 / R)`, in degrees. That's the bicycle-model angle; Ackermann splits it into inner and outer wheels.
   - PWM = `375 − 75 × command`.
3. Enter `[[pwm, angle_deg], …]` with **right turns positive** and left turns negative, including the straight-ahead point. Left and right are often **asymmetric**; keep both sides.

## 3. Speed per PWM: `speed_table`, `start_pwm`
1. Lay two tape marks 2.00 m apart on a straight, with a run-up of at least 1.5 m before the first.
2. For each **Max: User mode** value 0.50, 0.55, 0.60, 0.65, 0.70, 0.80, 0.90, 1.00:
   - Drive full stick, and time the car between the marks (or count video frames).
   - speed = 2.00 / time.
   - PWM = `370 + 37 × value` (stopped 370, forward max 407).
3. **Start threshold:** from standstill, raise the stick slowly and note the value at which the car starts moving. `start_pwm = 370 + 37 × value`. Do it 3 times.
4. Enter `[[pwm, m/s], …]`, including `[370, 0]` and the highest PWM that still doesn't move.

Battery charge changes this, so measure on a charged pack and note the voltage in the source.

## 4. Acceleration and braking: `accel_max`, `brake_coast`, `brake_active`
Film from the side at 240 fps, with tape marks every 25 cm.
- **accel_max:** full stick from standstill. Take the speed reached after 0.5 s (from the marks) divided by 0.5.
- **brake_coast:** at steady speed, snap the stick to neutral. The speed drop divided by the time to stop.
- **brake_active:** at steady speed, press **Y** (E-stop), which sends reverse pulses (the ESC brake). Same calculation.

## 5. Tyre grip: `mu_lat`
1. On a clear area, drive full-lock left circles, a little faster each time.
2. Watch for the circle getting **wider** at the same lock. That's the tyres sliding.
3. Take the fastest speed `v` before it widens, measured by timing one lap of the circle: `v = 2πR / lap time`. The radius `R` comes from step 2.
4. `mu_lat = v² / (R × 9.81)`.

Do it on the **track mat**, because grip on the mat is what matters.

## 6. Servo: `servo_rate_dps`, `servo_tau_s`
Film the front wheels from above at 240 fps while flicking the steering stick from full left to full right.
- **servo_rate_dps:** (left angle + right angle, from step 2) ÷ the time the wheels take to sweep across.
- **servo_tau_s:** how long the wheels take to settle in the last 10% of the movement, divided by 2.

## 7. Camera mount: `cam_height_m`, `cam_pitch_deg`, `cam_fwd_m` (and the lens model)
1. **cam_height_m:** the height of the lens centre above the floor.
2. **cam_fwd_m:** the horizontal distance from the rear axle to the lens.
3. **cam_pitch_deg:** how far the camera tilts down from horizontal. Measure with a phone inclinometer held against the camera board.
4. **Check the lens model** (this tests "160° is the diagonal field of view of a fisheye"):
   - Park the car square on a flat floor.
   - Lay tape strips across the floor at 30, 40, 50, 60, 80 and 100 cm ahead of the rear axle.
   - Save one frame (a recording, or the dashboard video).
   - For each strip, note the image row it appears on, as a fraction of 192.
   - Compare with `python3 -c "from sim.params import Profile; from sim.camera import Camera; c=Camera(Profile.load()); print([round(c.row_distance(r),2) for r in (0.45,0.5,0.55,0.62,0.7,0.8,0.9)])"`.
   - If they disagree after steps 1–3, the lens isn't the assumed model. Note it in `camera_fov_deg`'s source and tell whoever maintains `sim/camera.py`.

Until this step is done, height and tilt are **estimated** (Nov 10), not measured: **0.12 m** and **15°**.
- Height: the Waveshare side photos, scaled by the 174.1 mm wheelbase, put the lens centre at 0.118 m. Separately, `tools/calibrate_camera.py` gives a 0.366 m lane at 0.17 m; the to-scale Waveshare mat has a 0.27 m lane on the straights, which scales the height to 0.124 m.
- Tilt: `tools/calibrate_camera.py` fits 15.2° to the recorded frames. Only about 15° at 0.12 m reproduces the real line spacing (0.47 of the image width at the 0.62 row). The old 8° treated the wall base as the horizon; the wall base sits below it, so 8° is a lower bound.
- If you change them, set `CAM_HEIGHT_M` / `CAM_PITCH_DEG` in `pi/line_config.py` (or the dashboard's camera sliders) **and** `sim/profile.json`. Getting the height too high or the tilt too low makes pure pursuit run wide.

## 8. Delays: `actuator_delay_s` (and check `processing_s`)
- **processing_s** is already measured on the Pi (17 ms offline, 33–51 ms while driving). The dashboard's "Loop · detect" tile shows it live.
- **actuator_delay_s:**
  1. Film the dashboard screen and the front wheels together at 240 fps.
  2. Press a mode button that makes the pilot steer, or move the web joystick.
  3. The time from the screen change to the wheels starting to move, minus the network time, is roughly it.
  4. A cleaner way is to log `time.monotonic()` when the PWM is written: ask for this in a later session.

## 9. Line noise and dropout: `meas_noise`, `meas_dropout`
Record the car parked on the track for 10 s in Pilot steers mode, with the throttle untouched, so the recorder saves frames.
- **meas_noise:** the standard deviation of `x_ratio` in that run's `log.csv`.
- **meas_dropout:** the fraction of rows with `found = 0` during normal driving on a good track.

## 10. Camera-motion score: the auto-crawl / stall proxy
The simulator uses `motion ≈ 0.9 + 8 × speed`, which is an **assumption**. From the step-3 runs, the recordings give the real motion score at known speeds; fit a line and update `World.observe` in `sim/world.py`.

---

## What each value affects
| Value | Affects |
|---|---|
| steer_angle_table | which corners are possible at all; steering gain in the loop (KP/KD scale with it) |
| mu_lat | how fast each corner can be taken; slipping |
| speed_table, start_pwm | which throttle gives which speed; the speed profiles |
| cam_* (geometry) | how many metres "20% of the image width" means; preview distance; loop gain |
| servo, delays | phase margin: how high KP/KD can go before weaving |
| track geometry | everything about laps and corners |
