"""Line-follower settings for drive_line.py.

This file is ours (lives in ~/linefollow). It never touches ~/mycar/myconfig.py.
Any setting of linetrack.config.Config can be overridden here by writing its
name in UPPER CASE (e.g. KP = 2.0 overrides kp). Anything not listed keeps the
default from linetrack/config.py, which was tuned on real stage frames.
"""

# --- What to follow ---------------------------------------------------------
TARGET_X_RATIO = 0.40      # Nov 10 re-fit (camera 0.12 m / 15 deg): 0.38-0.40 passes every simulated variation; 0.42+ runs wide with delay.
                           # With PD steering (PURE_PURSUIT = False) use 0.40: PD cuts curves, and 0.46 put it on the left tape.
# Camera orientation is already set by ~/mycar's CAMERA_HFLIP / CAMERA_VFLIP; do not flip frames again here.
LOOKAHEAD_RATIO = 0.62     # row (fraction of image height) where the error is measured

# --- Orange detection -----------------------------------------------------------
# Stage measurements: tape S median 151 (p5 119); wood floor S median 53 but up to
# ~150 in places. S_LO 100: wood 1% passes (fake lines). 120: wood 0.02% but both-lines
# drops 86%->68% on stage frames. 110: wood 0.2%, both-lines 83%.
S_LO = 110
LOOKAHEAD_FROM_POINTS = True   # steer from where the edge really is at the lookahead row

# --- Steering (angle -1 = left, +1 = right) ----------------------------------
KP = 3.5                   # known good on the car (Nov 9 morning); 4.0+ crashed in simulated curves
KD = 0.4                   # was 0.25
STRAIGHT_KP = 3.5
CORNER_KP = 3.5            # 4.5 caused curve off-tracks in the simulator; steering is at full lock in curves anyway
MAX_STEERING = 1.0         # HARD STANDARD: full left lock (PWM 450, ~18 deg fitted to recorded runs). At 0.8 the car lost the line 58% of frames in tight curves.
STEERING_RATE_LIMIT = 6.0  # max change per second. Nov 10: 3.0 lost the line up to 9% at 1.00/0.92; 5-10 hold it (6 = smallest clean step)
STEERING_SMOOTHING = 0.20  # EMA, 0 = none. 0.35 (the PD setting) lagged pure pursuit at curve entry in the simulator
INVERT_STEERING = False    # if the lifted-wheels test is mirrored, flip THIS (never the PWM calibration)

# --- Corner geometry (linetrack/geometry.py; backtest: tools/backtest_corner.py) ------
# The camera's view of the edge is projected onto the ground and a circle fitted to it:
#   feedforward steering = atan(wheelbase / R)    (PD then corrects only what's left)
#   corner speed         = sqrt(GRIP_MU * 9.81 * R) * CORNER_MARGIN, braking early enough
# Simulator (10 variations of the unmeasured values): FF_GAIN 0.6 adds no crashes from Crawl
# to Fast and halves them at Max (37 -> 19); 1.0 is too much until STEER_MAX_DEG is measured.
# NOT YET VALIDATED ON REAL FRAMES. The code is right on simulated camera images (corr 0.76 with the
# true curvature, through the real detector). The recorded runs can't confirm it: they aren't clean
# laps, and the camera self-calibration (tools/calibrate_camera.py) finds the default camera model
# already consistent (both lines parallel within ~5.5%, bending the same way 92% of the time).
# KEEP FF_GAIN = 0 ON THE CAR until a clean slow lap (recording on: log.csv now has curv / bend_near_m / ff)
# shows straights near 0 and corners near 1 / (measured corner radius).
FF_GAIN = 0.0
# --- Steering law: pure pursuit (default) or PD + feedforward ----------------------------------------
# Pure pursuit aims at the point of the ride line Ld ahead: wheel angle = atan(2 * wheelbase * sin(alpha) / Ld).
# SIMULATOR ONLY, never driven on the real car. Nov 10 re-fit with the camera at 0.12 m / 15 deg: pure pursuit is
# clean in all 21 race_eval variations up to 0.92 / 0.86 (4.1 s lap); PD is clean only up to 0.75 / 0.70 (6.9 s).
# Check: python3 tools/race_eval.py --pairs. If it misbehaves on the car, press the dashboard's "PD fallback".
PURE_PURSUIT = True
PURSUIT_LD_M = 0.55        # lookahead distance at standstill, metres from the rear axle (Nov 10: 0.45-0.55 ok; 0.35 fails with delay)
PURSUIT_LD_GAIN_S = 0.30   # extra lookahead per m/s (speed from SPEED_TABLE below, which is assumed). 0.2 fails with delay, 0.4 at pitch 18
PURSUIT_PD = 0.10          # Nov 10: 0.10-0.15 ok, 0.05 and 0.25 fail. Share of KP/KD (3.5/0.4 -> 0.35/0.04) added on top of pure pursuit as image-space feedback.
                           # Pure pursuit aims through the camera ground model (pitch/height/FOV assumed); this PD term
                           # closes whatever offset a wrong model leaves.
PURSUIT_PREDICT_S = 0.15   # racer-v2: delay compensation, aim from where the car will be this long after the frame (0 = off).
                           # Matches the ~150 ms command-to-motion lag in the Nov 8 logs. Sim at 1.00/0.92 --pairs: 0 -> 123 wide
                           # (4 pairs fail), 0.15 -> 60 (1 pair), 0.20 same, 0.30 worse. NOT YET DRIVEN: if it misbehaves, set 0.
CONF_SLOW_BELOW = 0.0      # racer-v2 EXPERIMENT, off: below this detection confidence, blend throttle down to curve speed.
                           # Real tracking confidence is usually 0.7-0.85 (worst 5%: ~0.5); try 0.6.
CORNER_EXIT_STEER = 0.0    # curve exit hold: no measurable effect in the simulator, so off
CORNER_PLAN = False        # corner speed planning: adds little until grip is measured
CORNER_MARGIN = 0.8
CORNER_MIN_THROTTLE = 0.55
CURVATURE_SMOOTHING = 0.5
# Car model the geometry uses (sim/profile.json values; ASSUMED until measured: sim/CALIBRATION.md)
WHEELBASE_M = 0.1741
STEER_MAX_DEG = 18.0       # wheel angle at full steering command. FITTED to the recorded runs (tools/calibrate_sim.py),
                           # same value as the simulator (sim/profile.json); a slider on the dashboard. Measure it to be sure.
CAMERA_FOV_DEG = 160.0
CAM_HEIGHT_M = 0.12        # ESTIMATED Nov 10 (Waveshare photos 0.118 m; mat lane width -> 0.124 m). Was 0.17. Measure with a ruler.
CAM_PITCH_DEG = 15.0       # ESTIMATED Nov 10 (camera fit 15.2; line spacing in real frames). Was 8. If unsure, err high / err low on height:
                           # that way pure pursuit cuts inside instead of running wide.
CAM_FWD_M = 0.15
GRIP_MU = 0.5
BRAKE_DECEL = 1.5
CONTROL_DELAY_S = 0.08
SPEED_TABLE = [[370, 0.0], [387, 0.0], [390, 0.35], [407, 1.8]]

# --- Pilot speed in 'local' mode -----------------------------------------------
# AUTO_CRAWL = True: the pilot finds the slowest throttle that keeps the car
# rolling, using the camera to tell if it is moving (score ~0.9 still, 2.5+ rolling).
# It starts at CRAWL_START, creeps up CRAWL_RAMP_UP per second until it moves,
# backs off if it goes faster than CRAWL_FAST_MOTION, and never exceeds CRAWL_MAX.
# Measured: wheels start between stick ~0.45 and ~0.55 (PWM ~387-390).
AUTO_CRAWL = True
CRAWL_START = 0.40         # PWM ~385
CRAWL_MAX = 1.00           # respect the raised Full Auto cap when auto-crawl is used
CRAWL_RAMP_UP = 0.10       # throttle per second while not moving
CRAWL_MOVE_MOTION = 2.5    # camera motion score that counts as rolling
CRAWL_FAST_MOTION = 6.0    # above this it eases off

# --- Telling the two lines apart (linetrack/detector.py; backtest: tools/backtest.py) ---
# Each boundary keeps its identity from frame to frame: a line lying where LEFT was is LEFT.
# Geometry only starts a track (two lines one lane apart, or one line by the side it passes).
TRACK_MEMORY_FRAMES = 8    # a boundary that vanishes keeps its place this many frames (0.4 s)
LONE_CONFIRM_FRAMES = 1    # 2-3 = a single new line must hold still that many frames before it is trusted
LANE_WIDTH_RATIO = 0.47    # lane width at the lookahead row before it has been learned (fraction of width)
MASK_GROW_PX = 3           # strict orange grows this far into nearly-orange pixels (glare holes); 0 = off
MAX_EDGE_ROUGHNESS_PX = 2.5  # ragged-edged orange (wood grain, skin) is not tape; 0 = off

# --- Lost-line behaviour ---------------------------------------------------------
# Lost line: hold heading, keep rolling LOST_COAST_S, ramp down over LOST_SLOW_S, then stop.
LOST_HOLD_HEADING = True
LOST_COAST_S = 0.5
LOST_SLOW_S = 0.5

# --- One line only = maybe the wrong line ----------------------------------------
# With only one orange line identified, the pilot never speeds up; if that lasts
# longer than this, throttle goes to 0 until both lines are seen again.
# Real laps: one line only in ~35% of frames (inside line leaves view in turns),
# longest stretch ~1.2 s at human speed. Raise this if it stops in every turn. 0 = off.
SINGLE_LINE_STOP_S = 2.5

# --- Speed plan for Full auto (0 = off: use auto-crawl / fixed throttle) --------
# Curvature drives the handoff: straight -> transition -> corner. The plan slews throttle
# down/up so a bend is entered progressively and the car only accelerates after curvature clears.
# The auto-tuner fills straight/corner; transition defaults to their midpoint.
# START-UP speeds: what the real car has driven (0.70-0.75 with KP 3.5, Nov 9). The lap tuner climbs from here.
SPEED_STRAIGHT = 0.75
SPEED_TRANSITION = 0.75
SPEED_CORNER = 0.70
# RACE VALUES (simulated only; camera 0.12 m / 15 deg, steering rate 6, delay compensation 0.15 s):
# 1.00 / 0.90 = 3.69 s lap and passes all 21 race_eval variations, two-unknowns-wrong pairs included. 1.00 / 0.91
# also passes (3.67 s) but 0.92 fails fast car + low grip, so 0.90 keeps one step of margin. Without delay
# compensation 1.00 / 0.90 fails 3 pairs: if PURSUIT_PREDICT_S is set to 0, press "Safe race" instead.
# The dashboard's "Race" preset. Every speed in the simulator rests on the assumed SPEED_TABLE, so trust the lap tuner.
RACE_SPEED_STRAIGHT = 1.00
RACE_SPEED_TRANSITION = 0.95
RACE_SPEED_CORNER = 0.90
# SAFE RACE: 0.92 / 0.86 = 4.1 s lap, the fastest pair that also passes every two-unknowns-wrong pair. The "Safe race" preset.
SAFE_SPEED_STRAIGHT = 0.92
SAFE_SPEED_TRANSITION = 0.89
SAFE_SPEED_CORNER = 0.86
TUNE_RACE_FIRST = False    # False: "Start lap tuner" climbs from the speeds on the car. True: jump to the race values first.
SPEED_RAMP_DOWN = 2.0
SPEED_RAMP_UP = 0.4      # throttle units / s when exiting a bend

# --- Auto-tuner (dashboard "Auto-tune": start slow, fix gains, step speed up) ------
AUTOTUNE_START = 0.70      # used only by the classic strategies; the lap tuner starts from the speed plan above
AUTOTUNE_STEP = 0.04       # straight step; curves climb half as fast
AUTOTUNE_WINDOW_S = 10.0   # driving judged per step (about 1-2 laps)

# --- Track (only for the dashboard's lap-time ESTIMATE; sliders under "Lap timer") --------------
TRACK_LENGTH_M = 4.98      # inside line of the Waveshare mat, traced (sim/tracks/waveshare_inner_line.csv)
TRACK_CURVE_SHARE = 0.71   # share of that length that is curve

# --- Throttle (used in 'local' mode only if AUTO_CRAWL = False and the speed plan is off) ---
LINE_THROTTLE = 0.35       # pilot crawl (~PWM 383). 0.20 (~377) did not move the car from a stop.
MAX_THROTTLE = 1.00        # hard cap for any pilot throttle (fixed or auto-crawl)

# --- Top speed for EVERY mode (gamepad stick included) -----------------------
# Final forward throttle sent to the motor is clipped to this (1.0 = the car's
# normal full stick). Reverse / E-stop braking is never limited.
# PWM ~= 370 + 37 * throttle (stopped 370, full 407). If the car won't move
# at all, raise it in steps of 0.05.
TOP_THROTTLE = 1.00        # (~PWM 407) overall cap; reverse / emergency braking remains available.

# --- Max speed per mode (each is ALSO limited by TOP_THROTTLE) -------------------
# Easiest way to set speeds. Wheels only start turning from about 0.45-0.55.
MAX_USER = 0.60            # you drive (gamepad / web joystick)
MAX_PILOT_STEERS = 0.60    # pilot steers, you give throttle  (A once)
MAX_AUTO = 1.00            # full auto (A twice): also caps auto-crawl

# --- Slow down in turns (pilot modes only, including your stick in local_angle) ---
# The car is faster than it can turn, so throttle is scaled by how hard the
# wheels are turned:  scale = max(MIN_TURN_SCALE, 1 - TURN_SLOWDOWN * |steer| / MAX_STEERING)
# Example with TURN_SLOWDOWN 0.6 and MIN_TURN_SCALE 0.8: straight = 100%, half turn = 80%,
# full turn = 80%. Currently TURN_SLOWDOWN = 0.0, so there is no turn slow-down at all.
TURN_SLOWDOWN = 0.0        # off while AUTO_CRAWL holds the minimum speed (slowing more would stall it)
MIN_TURN_SCALE = 0.8       # at crawl speed, slowing more than this risks stalling the motor in turns

# --- Recording (for debugging; only while the pilot runs) --------------------
RECORD = True              # saves camera frames + decisions to ~/linefollow/data/<time>/

# --- Safety ------------------------------------------------------------------
MIN_CONFIDENCE = 0.35
LOST_TIMEOUT_S = 0.6       # hold last steering this long after losing the line, then centre
MAX_FRAME_AGE_S = 0.35     # no new camera frame for this long -> STALE -> throttle 0
RESET_AFTER_GAP_S = 0.3    # pilot state is reset if it wasn't called for this long (mode switch)

# --- Web UI ------------------------------------------------------------------
SHOW_OVERLAY = True        # Donkeycar's page (port 8887) shows the annotated detection image
WEB_UI = True              # our dashboard: live video + PID charts, knobs, web driving, past runs
WEB_PORT = 8890            # open http://<pi-ip>:8890 ; knob changes saved there go to tuning.json
