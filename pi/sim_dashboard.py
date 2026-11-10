#!/usr/bin/env python3
"""Run the dashboard on the Mac with no car: replays a recorded run's frames
through the real detector, pilot and dashboard parts at 20 Hz, looping.

    python3 pi/sim_dashboard.py runs/20261108-185226      # then open http://localhost:8890

Steering/throttle only go to the screen. Modes, knobs, web joystick, STOP and
Quit all work; Shut down Pi is disabled here.
"""
import glob
import os
import sys
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.dirname(HERE)]

import line_config  # noqa: E402
import webui  # noqa: E402
from donkey_part import (AutoCrawl, KillSwitch, LineDetectorPart, LineFollowerPilot,  # noqa: E402
                         ThrottleCap, load_line_config, make_throttle_cap)


def main(run_dir, port=8890):
    frames = sorted(glob.glob(os.path.join(run_dir, "*.jpg")))
    if not frames:
        sys.exit(f"no frames in {run_dir}")
    lcfg, ex = load_line_config(line_config)
    crawl = AutoCrawl(start=ex["CRAWL_START"], hi=min(ex["CRAWL_MAX"], lcfg.max_throttle),
                      ramp_up=ex["CRAWL_RAMP_UP"], move_thresh=ex["CRAWL_MOVE_MOTION"],
                      fast_thresh=ex["CRAWL_FAST_MOTION"]) if ex["AUTO_CRAWL"] else None
    pilot = LineFollowerPilot(lcfg, crawl=crawl, single_line_stop_s=ex["SINGLE_LINE_STOP_S"],
                              coast_s=ex["LOST_COAST_S"], slow_s=ex["LOST_SLOW_S"])
    det = LineDetectorPart(lcfg)
    det.pilot = pilot
    pilot.bend_source = lambda: det.bend
    cap = make_throttle_cap(lcfg, ex)
    kill = KillSwitch()
    hub = webui.Hub(lcfg, pilot, det, crawl=crawl, cap=cap,
                    data_dir=os.path.dirname(os.path.abspath(run_dir)),
                    tuning_path=os.path.join(HERE, "tuning_sim.json"),
                    log_dir=os.path.join(os.path.dirname(HERE), "logs_sim"))
    hub.source, hub.source_detail = "replay", f"recorded run {os.path.basename(os.path.abspath(run_dir))}"
    hub.log("info", f"Replay started: {os.path.basename(os.path.abspath(run_dir))}", "start")
    hub.kill = kill  # Quit program ends this replay; Shut down Pi stays disabled
    wmode, wdrive, guard, tele = webui.WebModePart(hub), webui.WebDrivePart(hub), webui.WebGuardPart(hub), webui.TelemetryPart(hub)
    srv, url = webui.start_server(hub, port=port)
    print(f"dashboard: http://localhost:{port}  (also {url})  replaying {len(frames)} frames from {run_dir}", flush=True)

    mode = "local_angle"
    i = 0
    try:
        while True:
            t0 = time.monotonic()
            rgb = cv2.cvtColor(cv2.imread(frames[i % len(frames)]), cv2.COLOR_BGR2RGB)
            i += 1
            mode = wmode.run(mode)
            u_steer, u_thr = wdrive.run(0.0, 0.0)                 # no gamepad on the Mac
            x, found, age, overlay, motion, both = det.run(rgb)
            if mode != "user":
                p_angle, p_thr = pilot.run(mode, x, found, age, motion, both)
            else:
                p_angle, p_thr = None, None
            if mode == "user":
                steering, throttle = u_steer, u_thr
            elif mode == "local_angle":
                steering, throttle = p_angle or 0.0, u_thr
            else:
                steering, throttle = p_angle or 0.0, p_thr or 0.0
            throttle = guard.run(cap.run(throttle, steering, mode))
            steering, throttle = kill.run(steering, throttle)
            tele.run(mode, u_thr, steering, throttle, overlay, motion, both, x, found)
            time.sleep(max(0.0, 0.05 - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("replay stopped (Quit program or Ctrl-C)")
    finally:
        srv.shutdown()


def main_sim(port=8890):
    """The PiRacer simulator (sim/, profile sim/profile.json) as the 'car': synthetic camera
    frames go through the REAL detector + pilot + caps, exactly like drive_line.py."""
    sys.path.insert(0, os.path.dirname(HERE))
    from autotune import AutoTuner
    from sim.params import Profile
    from sim.world import World
    webui.Hub.LIVE_NAME = "gain_schedule_live_sim.json"   # simulated passes must never count as car-verified
    lcfg, ex = load_line_config(line_config)
    crawl = AutoCrawl(start=ex["CRAWL_START"], hi=1.0)
    pilot = LineFollowerPilot(lcfg, crawl=crawl, single_line_stop_s=0.0,
                              coast_s=ex["LOST_COAST_S"], slow_s=ex["LOST_SLOW_S"])
    det = LineDetectorPart(lcfg)
    det.pilot = pilot
    pilot.bend_source = lambda: det.bend
    cap = make_throttle_cap(lcfg, ex)
    kill = KillSwitch()
    hub = webui.Hub(lcfg, pilot, det, crawl=crawl, cap=cap, data_dir=os.path.join(os.path.dirname(HERE), "runs"),
                    tuning_path=os.path.join(HERE, "tuning_sim.json"), log_dir=os.path.join(os.path.dirname(HERE), "logs_sim"))
    hub.kill = kill
    hub.autotuner = AutoTuner(pilot, start=ex["AUTOTUNE_START"], step=ex["AUTOTUNE_STEP"],
                              window_s=ex["AUTOTUNE_WINDOW_S"], ceiling=cap.limit("local"), log=hub.log)
    wmode, wdrive, guard, tele = webui.WebModePart(hub), webui.WebDrivePart(hub), webui.WebGuardPart(hub), webui.TelemetryPart(hub)
    prof = Profile.load()
    hub.source, hub.source_detail = "simulator", f"PiRacer simulator ({len(prof.assumed_in_use())} assumed values)"
    w = World(prof, vision="rendered", seed=1, target_ratio=lcfg.target_x_ratio, row_ratio=lcfg.lookahead_ratio)
    srv, url = webui.start_server(hub, port=port)
    hub.log("info", f"PiRacer simulator ({len(prof.assumed_in_use())} assumed values in sim/profile.json). "
                    "Off track = car put back at the start.", "start")
    print(f"dashboard (simulator): http://localhost:{port}", flush=True)
    mode, seen_off = "user", 0
    try:
        while True:
            t0 = time.monotonic()
            mode = wmode.run(mode)
            u_steer, u_thr = wdrive.run(0.0, 0.0)
            _, _, _, age, motion_proxy, img = w.observe()
            if img is None:
                w.tick(0.0, 0.0)
                continue
            x, found, age, overlay, motion, both = det.run(img)
            motion = motion_proxy                         # synthetic frames have no texture: use the speed proxy
            if mode != "user":
                p_angle, p_thr = pilot.run(mode, x, found, age, motion, both)
            else:
                p_angle = p_thr = None
            if mode == "user":
                steering, throttle = u_steer, u_thr
            elif mode == "local_angle":
                steering, throttle = p_angle or 0.0, u_thr
            else:
                steering, throttle = p_angle or 0.0, p_thr or 0.0
            throttle = guard.run(cap.run(throttle, steering, mode))
            steering, throttle = kill.run(steering, throttle)
            w.tick(steering, throttle)
            if w.metrics.off_track > seen_off:
                seen_off = w.metrics.off_track
                hub.log("bad", "Simulated car went off track: put back at the start", "line")
            tele.run(mode, u_thr, steering, throttle, overlay, motion, both, x, found)
            time.sleep(max(0.0, w.dt_ctrl - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("simulator stopped")
    finally:
        srv.shutdown()


if __name__ == "__main__":
    if "--sim" in sys.argv:
        main_sim(int(sys.argv[sys.argv.index("--sim") + 1]) if len(sys.argv) > sys.argv.index("--sim") + 1 else 8890)
        sys.exit(0)
    main(sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(os.path.join(os.path.dirname(HERE), "runs", "*")))[-1],
         int(sys.argv[2]) if len(sys.argv) > 2 else 8890)
