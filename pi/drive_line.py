#!/usr/bin/env python3
"""Drive the PiRacer with the orange left-line follower.

Reuses the car's own setup from ~/mycar (camera, web UI, gamepad, PWM drivetrain)
by importing manage.py read-only. Nothing in ~/mycar is written: we never chdir
there, there is no tub writer, and Python bytecode caching is turned off so no
__pycache__ appears in ~/mycar.

Usage (on the Pi):
    cd ~/linefollow && ~/env/bin/python drive_line.py --js       # drive
    cd ~/linefollow && ~/env/bin/python drive_line.py --check DIR  # offline: no camera, no motors

Gamepad: A = mode (user -> local_angle -> local -> user), Y = emergency stop,
         X = OFF: stops the car and quits the program (restart needed).
  user        human steers + throttle (pilot idle; web UI still shows detection)
  local_angle pilot steers, human throttle   <- first on-track tests
  local       pilot steers + throttle. Throttle source, first that applies:
              speed plan (SPEED_STRAIGHT > 0), else auto-crawl (AUTO_CRAWL, the
              default), else fixed LINE_THROTTLE. Then capped by MAX_THROTTLE,
              MAX_AUTO and TOP_THROTTLE (all in line_config.py).
"""
import sys

sys.dont_write_bytecode = True  # must come before importing anything from ~/mycar

import argparse
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))
MYCAR = "/home/pi/mycar"
sys.path.insert(0, HERE)
sys.path.insert(1, MYCAR)  # for `import manage` and `from my_joystick import ...`

import donkeycar as dk  # noqa: E402
import manage  # noqa: E402  (read-only; docopt only runs under its __main__)
from donkeycar.parts.pipe import Pipe  # noqa: E402
from donkeycar.parts.throttle_filter import ThrottleFilter  # noqa: E402

import line_config  # noqa: E402
from donkey_part import (AutoCrawl, KillSwitch, LineDetectorPart, LineFollowerPilot, car_model,  # noqa: E402
                         LineRecorder, SpeedPlan, ThrottleCap, ThrottleMonitor, load_line_config,
                         make_throttle_cap)
from autotune import AutoTuner  # noqa: E402
import webui  # noqa: E402


def build_parts():
    lcfg, extras = load_line_config(line_config)
    crawl = None
    if extras["AUTO_CRAWL"]:
        crawl = AutoCrawl(start=extras["CRAWL_START"], hi=min(extras["CRAWL_MAX"], lcfg.max_throttle),
                          ramp_up=extras["CRAWL_RAMP_UP"], move_thresh=extras["CRAWL_MOVE_MOTION"],
                          fast_thresh=extras["CRAWL_FAST_MOTION"])
    plan = None
    if extras["SPEED_STRAIGHT"] > 0:
        plan = SpeedPlan(extras["SPEED_STRAIGHT"], extras["SPEED_CORNER"] or extras["SPEED_STRAIGHT"],
                         transition=extras["SPEED_TRANSITION"] or None,
                         ramp_down=extras["SPEED_RAMP_DOWN"], ramp_up=extras["SPEED_RAMP_UP"])
    pilot = LineFollowerPilot(lcfg, reset_after_gap_s=extras["RESET_AFTER_GAP_S"], crawl=crawl, speed_plan=plan,
                              single_line_stop_s=extras["SINGLE_LINE_STOP_S"],
                              coast_s=extras["LOST_COAST_S"], slow_s=extras["LOST_SLOW_S"])
    model = car_model(extras)
    pilot.model = model
    det = LineDetectorPart(lcfg, show_overlay=extras["SHOW_OVERLAY"], model=model)
    det.pilot = pilot
    pilot.bend_source = lambda: det.bend      # corner geometry for feedforward / corner speed
    return lcfg, extras, det, pilot


def drive(use_joystick: bool, myconfig: str):
    cfg = dk.load_config(config_path=os.path.join(MYCAR, "config.py"), myconfig=myconfig)
    lcfg, extras, det, pilot = build_parts()
    print(f"[linefollow] target={lcfg.target_x_ratio} kp={lcfg.kp} kd={lcfg.kd} "
          f"pilot throttle={'AUTO_CRAWL ' + str(extras['CRAWL_START']) + '..' + str(extras['CRAWL_MAX']) if extras['AUTO_CRAWL'] else lcfg.throttle} "
          f"max speed: overall {extras['TOP_THROTTLE']} user {extras['MAX_USER']} pilot-steers {extras['MAX_PILOT_STEERS']} auto {extras['MAX_AUTO']} "
          f"invert={lcfg.invert_steering} record={extras['RECORD']}")

    cap = make_throttle_cap(lcfg, extras)
    kill = KillSwitch()
    hub = None
    if extras["WEB_UI"]:
        hub = webui.Hub(lcfg, pilot, det, crawl=pilot.crawl, cap=cap, data_dir=os.path.join(HERE, "data"),
                        tuning_path=os.path.join(HERE, "tuning.json"), log_dir=os.path.join(HERE, "logs"))
        hub.log("info", f"Program started (max speed: user {cap.limit('user'):.2f}, pilot steers "
                        f"{cap.limit('local_angle'):.2f}, full auto {cap.limit('local'):.2f})", "start")
        applied = hub.load_saved()
        if applied:
            print(f"[linefollow] loaded saved tuning.json: {applied}", flush=True)

    V = dk.vehicle.Vehicle()
    manage.add_camera(V, cfg, "single")                       # -> cam/image_array (RGB)
    if hub:  # BEFORE the controllers: a mode picked on the dashboard is adopted like pressing A
        V.add(webui.WebModePart(hub), inputs=["user/mode"], outputs=["user/mode"])
    ctr = manage.add_user_controller(V, cfg, use_joystick)    # web UI + gamepad (Y = E-stop)
    V.add(webui.ManualOverridePart(pilot, ctr, hub),
          inputs=["user/mode", "user/steering", "user/throttle"],
          outputs=["user/mode", "user/steering", "user/throttle"])
    if hub:  # dashboard joystick (dead-man) replaces the gamepad sticks only while held
        V.add(webui.WebDrivePart(hub), inputs=["user/steering", "user/throttle"],
              outputs=["user/steering", "user/throttle"])
    V.add(Pipe(), inputs=["user/steering"], outputs=["user/angle"])
    V.add(ThrottleFilter(), inputs=["user/throttle"], outputs=["user/throttle"])

    # detection runs in every mode so the web UI shows what the pilot sees
    V.add(det, inputs=["cam/image_array"],
          outputs=["line/x_ratio", "line/found", "line/frame_age", "line/overlay", "line/motion", "line/both"])
    V.add(manage.UserPilotCondition(show_pilot_image=True),
          inputs=["user/mode", "line/overlay", "line/overlay"],
          outputs=["run_user", "run_pilot", "ui/image_array"])
    V.add(pilot, inputs=["user/mode", "line/x_ratio", "line/found", "line/frame_age", "line/motion", "line/both"],
          outputs=["pilot/angle", "pilot/throttle"], run_condition="run_pilot")
    V.add(manage.DriveMode(cfg.AI_THROTTLE_MULT),
          inputs=["user/mode", "user/angle", "user/throttle", "pilot/angle", "pilot/throttle"],
          outputs=["steering", "throttle"])
    V.add(Pipe(), inputs=["throttle"], outputs=["debug/drivemode_throttle"])
    # top-speed limit (every mode) + slow down in turns (pilot modes); braking/E-stop untouched
    V.add(cap, inputs=["throttle", "steering", "user/mode"], outputs=["throttle"])
    if hub:  # 1 s of forced zero forward throttle after a dashboard STOP
        V.add(webui.WebGuardPart(hub), inputs=["throttle"], outputs=["throttle"])
    # gamepad X = OFF: zero steering/throttle, then shut the whole program down
    V.add(kill, inputs=["steering", "throttle"], outputs=["steering", "throttle"])
    if hasattr(ctr, "set_button_down_trigger"):
        ctr.set_button_down_trigger("x", kill.trigger)  # replaces 'erase records' (no tub here anyway)
    V.add(ThrottleMonitor(), inputs=["user/mode", "user/throttle", "pilot/throttle",
                                     "debug/drivemode_throttle", "throttle", "steering", "line/motion"])
    if hub:  # last before the motors: what is actually being sent, for the dashboard
        V.add(webui.TelemetryPart(hub), inputs=["user/mode", "user/throttle", "steering", "throttle",
                                                "line/overlay", "line/motion", "line/both",
                                                "line/x_ratio", "line/found"])
        hub.autotuner = AutoTuner(pilot, start=extras["AUTOTUNE_START"], step=extras["AUTOTUNE_STEP"],
                                  window_s=extras["AUTOTUNE_WINDOW_S"], ceiling=cap.limit("local"), log=hub.log)
        hub.joystick = ctr if hasattr(ctr, "emergency_stop") else None
        hub.kill = kill
        hub.allow_poweroff = True
        _, url = webui.start_server(hub, port=int(extras["WEB_PORT"]))
        print(f"[linefollow] DASHBOARD: {url}", flush=True)
    V.add(webui.AutoMotionGuard(pilot), inputs=["user/mode", "throttle"], outputs=["throttle"])
    if extras["RECORD"]:
        V.add(LineRecorder(os.path.join(HERE, "data"), det),
              inputs=["cam/image_array", "user/mode", "line/x_ratio", "line/found",
                      "line/frame_age", "steering", "throttle", "line/motion"],
              run_condition="run_pilot")
    manage.add_drivetrain(V, cfg)

    print(f"[linefollow] web UI: http://<pi-ip>:{cfg.WEB_CONTROL_PORT}   A = mode, Y = E-STOP, X = OFF (quit)", flush=True)
    if hasattr(ctr, "print_controls"):
        ctr.print_controls()
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


def check(frames_dir: str):
    """Offline check: config loads, manage imports, parts run on saved frames. No hardware."""
    import glob

    import cv2

    cfg = dk.load_config(config_path=os.path.join(MYCAR, "config.py"), myconfig="myconfig.py")
    assert hasattr(manage, "DriveMode") and hasattr(manage, "add_drivetrain")
    print(f"[check] car config: {cfg.CAMERA_TYPE} {cfg.IMAGE_W}x{cfg.IMAGE_H} "
          f"loop {cfg.DRIVE_LOOP_HZ} Hz, drivetrain {cfg.DRIVE_TRAIN_TYPE}")
    lcfg, extras, det, pilot = build_parts()
    cap = make_throttle_cap(lcfg, extras)
    assert cap.run(1.0) == extras["TOP_THROTTLE"] and cap.run(-1.0, 0.8, "local") == -1.0
    print(f"[check] max forward throttle: user {cap.limit('user'):.2f}, pilot steers {cap.limit('local_angle'):.2f}, "
          f"full auto {cap.limit('local'):.2f} (overall {cap.top:.2f})")
    print(f"[check] full stick -> {cap.run(1.0, 0.0, 'local'):.3f} straight, "
          f"{cap.run(1.0, lcfg.max_steering / 2, 'local'):.3f} half turn, "
          f"{cap.run(1.0, lcfg.max_steering, 'local'):.3f} full turn; E-stop brake passes")
    files = sorted(glob.glob(os.path.join(frames_dir, "*.jpg")))
    if not files:
        sys.exit(f"[check] no .jpg frames in {frames_dir}")
    n_found, ms = 0, []
    for f in files:
        rgb = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2RGB)  # what the camera part delivers
        t0 = time.perf_counter()
        x, found, age, _, motion, both = det.run(rgb)
        angle, thr = pilot.run("local_angle", x, found, age, motion, both)
        ms.append((time.perf_counter() - t0) * 1000)
        n_found += found
        assert thr == 0.0, "pilot throttle must be 0 outside 'local' mode"
    ms.sort()
    print(f"[check] {len(files)} frames: tracked {n_found} ({100 * n_found / len(files):.0f}%), "
          f"detector+pilot+overlay {sum(ms) / len(ms):.1f} ms mean, {ms[int(0.95 * (len(ms) - 1))]:.1f} ms p95")
    print("[check] OK (no camera opened, no PWM written)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--js", action="store_true", help="use the physical gamepad (also on by default via USE_JOYSTICK_AS_DEFAULT)")
    ap.add_argument("--myconfig", default="myconfig.py", help="car config overrides in ~/mycar (read only)")
    ap.add_argument("--check", metavar="FRAMES_DIR", help="offline self-test on saved .jpg frames; touches no hardware")
    a = ap.parse_args()
    if a.check:
        check(a.check)
    else:
        drive(a.js, a.myconfig)
