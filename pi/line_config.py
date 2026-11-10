"""Line-follower settings for drive_line.py.

This file is ours (lives in ~/linefollow). It never touches ~/mycar/myconfig.py.
Any setting of linetrack.config.Config can be overridden here by writing its
name in UPPER CASE (e.g. KP = 2.0 overrides kp). Anything not listed keeps the
default from linetrack/config.py, which was tuned on real stage frames.
"""

# --- What to follow ---------------------------------------------------------
TARGET_X_RATIO = 0.40      # left orange line's inside edge sits left-of-center; 0.80 = other side
# Camera orientation is already set by ~/mycar's CAMERA_HFLIP / CAMERA_VFLIP; do not flip frames again here.
LOOKAHEAD_RATIO = 0.62     # row (fraction of image height) where the error is measured

# --- Orange detection -----------------------------------------------------------
# Stage measurements: tape S median 151 (p5 119); wood floor S median 53 but up to
# ~150 in places. S_LO 100: wood 1% passes (fake lines). 120: wood 0.02% but both-lines
# drops 86%->68% on stage frames. 110: wood 0.2%, both-lines 83%.
S_LO = 110
LOOKAHEAD_FROM_POINTS = True   # steer from where the edge really is at the lookahead row

# --- Steering (angle -1 = left, +1 = right) ----------------------------------
KP = 3.5                   # was 2.2: corrected too gently and drifted off the mat (error 0.1 -> steer 0.35)
KD = 0.4                   # was 0.25
MAX_STEERING = 0.8
STEERING_RATE_LIMIT = 3.0  # max change per second
STEERING_SMOOTHING = 0.35  # EMA, 0 = none
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
CORNER_PLAN = False        # corner speed planning: adds little until grip is measured
CORNER_MARGIN = 0.8
CORNER_MIN_THROTTLE = 0.55
CURVATURE_SMOOTHING = 0.5
# Car model the geometry uses (sim/profile.json values; ASSUMED until measured: sim/CALIBRATION.md)
WHEELBASE_M = 0.1741
STEER_MAX_DEG = 25.0       # wheel angle at full steering command: MEASURE (most important)
CAMERA_FOV_DEG = 160.0
CAM_HEIGHT_M = 0.17
CAM_PITCH_DEG = 8.0
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
CRAWL_MAX = 0.60           # PWM ~392 hard ceiling (humans drive at ~1.0 = PWM 407)
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
SPEED_STRAIGHT = 0.55   # conservative first-pass throttle command; increase only after passes
SPEED_TRANSITION = 0.55 # neutral handoff for the first passes
SPEED_CORNER = 0.55     # begin at crawl pace; lower only after confirming it still rolls
SPEED_RAMP_DOWN = 1.0    # throttle units / s when entering a bend
SPEED_RAMP_UP = 0.4      # throttle units / s when exiting a bend

# --- Auto-tuner (dashboard "Auto-tune": start slow, fix gains, step speed up) ------
AUTOTUNE_START = 0.55      # first speed (just above where the motor starts)
AUTOTUNE_STEP = 0.03       # speed added after each healthy window
AUTOTUNE_WINDOW_S = 10.0   # driving judged per step (about 1-2 laps)

# --- Throttle (used in 'local' mode only if AUTO_CRAWL = False and the speed plan is off) ---
LINE_THROTTLE = 0.35       # pilot crawl (~PWM 383). 0.20 (~377) did not move the car from a stop.
MAX_THROTTLE = 0.60        # hard cap for any pilot throttle (fixed or auto-crawl)

# --- Top speed for EVERY mode (gamepad stick included) -----------------------
# Final forward throttle sent to the motor is clipped to this (1.0 = the car's
# normal full stick). Reverse / E-stop braking is never limited.
# PWM ~= 370 + 37 * throttle (stopped 370, full 407). If the car won't move
# at all, raise it in steps of 0.05.
TOP_THROTTLE = 0.60        # (~PWM 392) also limits your stick. Humans drive at ~1.0 (PWM 407).

# --- Max speed per mode (each is ALSO limited by TOP_THROTTLE) -------------------
# Easiest way to set speeds. Wheels only start turning from about 0.45-0.55.
MAX_USER = 0.60            # you drive (gamepad / web joystick)
MAX_PILOT_STEERS = 0.60    # pilot steers, you give throttle  (A once)
MAX_AUTO = 0.60            # full auto (A twice): also caps auto-crawl

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
