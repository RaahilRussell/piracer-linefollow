"""Interactive viewer server: runs the simulation live and streams it to sim/viewer.html.

  python3 -m sim view      ->  http://localhost:8892
Standard library only. The simulation runs in a background thread at real time
(x the chosen speed-up); the page sends settings and manual driving commands.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from .actuators import SteeringMap, ThrottleMap
from .controllers import PilotController
from .params import Profile
from .vehicle import Extensions
from .world import World

HERE = os.path.dirname(os.path.abspath(__file__))


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class Session:
    def __init__(self):
        self.lock = threading.Lock()
        self.base = Profile.load()
        self.settings = {"mode": "auto", "throttle": 0.60, "straight": 0.0, "corner": 0.0, "kp": None, "kd": None,
                         "target": None, "row": None, "fps": None, "latency": None, "mu": None,
                         "slip": True, "servo": True, "accel": True, "delay": True, "vision": "geometric",
                         "speedup": 1.0, "running": True}
        self.manual = {"steer": 0.0, "thr": 0.0, "t": -1e9}
        self.sweep = {"state": "idle", "rows": []}
        self.build()

    def build(self):
        s = self.settings
        prof = self.base if s["mu"] is None else self.base.with_values(mu_lat=float(s["mu"]))
        self.clk = Clock()
        ov = {k: s[k] for k in ("kp", "kd") if s[k] is not None}
        if s["target"] is not None:
            ov["target_x_ratio"] = s["target"]
        if s["row"] is not None:
            ov["lookahead_ratio"] = s["row"]
        speed = (s["straight"], s["corner"] or s["straight"]) if s["straight"] else s["throttle"]
        self.ctrl = PilotController("viewer", ov, speed=speed, clock=self.clk)
        self.world = World(prof, Extensions(slip=s["slip"], servo=s["servo"], accel=s["accel"], delay=s["delay"]),
                           vision=s["vision"], seed=1, target_ratio=self.ctrl.cfg.target_x_ratio,
                           row_ratio=self.ctrl.cfg.lookahead_ratio, camera_fps=s["fps"], processing_s=s["latency"])
        self.prof = prof
        self.err_hist = []
        self.last_obs = (None, False, False)

    # ---------------------------------------------------------------- loop
    def run(self):
        last = time.monotonic()
        while True:
            time.sleep(0.01)
            now = time.monotonic()
            with self.lock:
                if not self.settings["running"]:
                    last = now
                    continue
                budget = (now - last) * self.settings["speedup"]
                last = now
                w = self.world
                while budget >= w.dt_ctrl:
                    budget -= w.dt_ctrl
                    self._tick()
                last -= budget / max(self.settings["speedup"], 1e-6)

    def _tick(self):
        w, s = self.world, self.settings
        self.clk.t = w.t
        x, found, both, age, motion, img = w.observe()
        angle, thr, cmd = self.ctrl.step(x, found, both, age, motion, img, edges=w.edges)
        if s["mode"] == "manual":
            fresh = time.monotonic() - self.manual["t"] < 0.4
            angle, thr = (self.manual["steer"], self.manual["thr"]) if fresh else (0.0, 0.0)
        self.last_obs = (x if img is None else self.ctrl.det._det.x_ratio if self.ctrl.det and self.ctrl.det._det else x,
                         found, both)
        w.tick(angle, thr)
        if cmd is not None and cmd.status == "TRACKING":
            self.err_hist.append((round(w.t, 2), round(cmd.error, 4), round(angle, 3)))
            self.err_hist = self.err_hist[-200:]

    # ---------------------------------------------------------------- state for the page
    def state(self):
        w = self.world
        v = w.veh
        s = v.s
        sm = SteeringMap(self.prof)
        x, found, both = self.last_obs
        ax, ay = s.x, s.y
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        look_d = w.cam.row_distance(w.row_ratio)
        # where the left edge SHOULD be (target column) and where the lookahead ground point is
        r = min(w.cam.H - 1, int(w.row_ratio * w.cam.H))
        ti = int(np.clip(round(w.target_ratio * w.cam.W - 0.5), 0, w.cam.W - 1))
        tg = (w.cam.gx[r, ti], w.cam.gy[r, ti])
        target_pt = (ax + tg[0] * c - tg[1] * sn, ay + tg[0] * sn + tg[1] * c) if np.isfinite(tg[0]) else None
        seen_pt = None
        if x is not None and found:
            xi = int(np.clip(round(x * w.cam.W - 0.5), 0, w.cam.W - 1))
            g = (w.cam.gx[r, xi], w.cam.gy[r, xi])
            if np.isfinite(g[0]):
                seen_pt = (ax + g[0] * c - g[1] * sn, ay + g[0] * sn + g[1] * c)
        row_pts = []
        for i in range(0, w.cam.W, 8):
            g = (w.cam.gx[r, i], w.cam.gy[r, i])
            if np.isfinite(g[0]):
                row_pts.append((ax + g[0] * c - g[1] * sn, ay + g[0] * sn + g[1] * c))
        kappa_max_l = math.tan(sm.max_angle_rad()[0]) / v.L
        off = w.track.offset_from_left(s.x, s.y)
        feas, _ = w.feasibility(max(s.v, 0.01)) if s.v > 0.05 else ([], [])
        return {
            "t": round(w.t, 2), "pose": [s.x, s.y, s.yaw], "v": round(s.v, 3), "yaw_rate": round(math.degrees(s.yaw_rate), 1),
            "a_lat": round(s.a_lat, 2), "a_lat_max": round(s.a_lat_max, 2), "slipping": s.slipping, "braking": s.braking,
            "steer_cmd": round(s.steer_cmd, 3), "steer_pwm": s.steer_pwm, "delta_deg": round(math.degrees(s.delta), 1),
            "delta_target_deg": round(math.degrees(s.delta_target), 1),
            "thr_cmd": round(s.thr_cmd, 3), "thr_pwm": s.thr_pwm,
            "radius_cmd": round(1 / s.kappa_cmd, 3) if abs(s.kappa_cmd) > 1e-6 else None,
            "radius_actual": round(1 / s.kappa, 3) if abs(s.kappa) > 1e-6 else None,
            "radius_min": round(1 / kappa_max_l, 3),
            "footprint": v.footprint(), "wheelbase": v.L, "track_w": self.prof.body_width_m - 0.02,
            "delta": s.delta, "path": v.projected_path(1.0),
            "x_ratio": None if x is None else round(float(x), 3), "found": bool(found), "both": bool(both),
            "target_ratio": w.target_ratio, "row_ratio": w.row_ratio, "look_m": round(look_d, 3),
            "target_pt": target_pt, "seen_pt": seen_pt, "row_pts": row_pts,
            "offset_left_cm": round(100 * off, 1), "ride_cm": round(100 * w._ride, 1),
            "metrics": w.metrics.summary(), "feas": feas, "err_hist": self.err_hist[-120:],
            "settings": self.settings, "kp": self.ctrl.cfg.kp, "kd": self.ctrl.cfg.kd,
            "fps": w.camera_fps, "latency": w.processing_s, "mu": self.prof.mu_lat,
        }

    def camera_jpeg(self):
        w = self.world
        s = w.veh.s
        img = w.cam.render((s.x, s.y, s.yaw), w.track)
        r = min(w.cam.H - 1, int(w.row_ratio * w.cam.H))
        img[r, :] = (0, 230, 255)
        tx = int(w.target_ratio * w.cam.W)
        img[r - 6:r + 7, tx] = (0, 255, 255)
        x = self.last_obs[0]
        if x is not None:
            xi = int(np.clip(x * w.cam.W, 0, w.cam.W - 1))
            img[r - 6:r + 7, max(0, xi - 1):xi + 1] = (40, 220, 40)
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST), cv2.COLOR_RGB2BGR))
        return buf.tobytes()

    def run_sweep(self, kind):
        from .__main__ import run_one
        with self.lock:
            self.sweep = {"state": "running", "kind": kind, "rows": []}
            prof, s = self.prof, dict(self.settings)
        tm = ThrottleMap(prof)
        rows = []
        if kind == "speeds":
            for thr in (0.50, 0.55, 0.60, 0.65, 0.70, 0.80, 0.90, 1.00):
                w, _ = run_one(prof, seconds=30, speed=thr, overrides={k: s[k] for k in ("kp", "kd") if s[k] is not None})
                r = w.metrics.summary()
                pwm = tm.pwm(thr)
                r.update(throttle=thr, pwm=pwm, steady_mps=round(tm.steady_speed(pwm, True), 2))
                rows.append(r)
        else:
            for kp in (1.5, 2.5, 3.5, 5.0, 7.0):
                for kd in (0.0, 0.4, 0.8):
                    w, _ = run_one(prof, seconds=30, speed=s["throttle"], overrides={"kp": kp, "kd": kd})
                    r = w.metrics.summary()
                    r.update(kp=kp, kd=kd)
                    rows.append(r)
        with self.lock:
            self.sweep = {"state": "done", "kind": kind, "rows": rows}


def params_payload(prof):
    out = []
    for k, e in prof.as_dict().items():
        out.append({"name": k, "value": e["value"], "unit": e.get("unit", ""), "status": e.get("status"),
                    "source": e.get("source", ""), "range": e.get("range")})
    return out


def serve(port=8892):
    sess = Session()
    threading.Thread(target=sess.run, daemon=True).start()

    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = self.path.split("?")[0]
            try:
                if p in ("/", "/index.html"):
                    with open(os.path.join(HERE, "viewer.html"), "rb") as f:
                        return self._send(200, f.read(), "text/html; charset=utf-8")
                if p == "/api/track":
                    with sess.lock:
                        tr = sess.world.track
                        return self._send(200, json.dumps({"left": tr.left[::2].tolist(), "right": tr.right[::2].tolist(),
                                                           "tape_w": tr.tape_w}).encode())
                if p == "/api/params":
                    with sess.lock:
                        return self._send(200, json.dumps(params_payload(sess.prof)).encode())
                if p == "/api/state":
                    with sess.lock:
                        st = sess.state()
                        st["sweep"] = sess.sweep
                    return self._send(200, json.dumps(st, default=float).encode())
                if p == "/camera.jpg":
                    with sess.lock:
                        jpg = sess.camera_jpeg()
                    return self._send(200, jpg, "image/jpeg")
                return self._send(404, b"{}")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
            p = self.path
            try:
                if p == "/api/settings":
                    rebuild = False
                    with sess.lock:
                        for k, v in body.items():
                            if k in sess.settings:
                                sess.settings[k] = v
                                rebuild = rebuild or k not in ("running", "speedup", "mode")
                        if rebuild or body.get("reset"):
                            sess.build()
                    return self._send(200, b'{"ok":true}')
                if p == "/api/manual":
                    sess.manual = {"steer": float(body.get("steer", 0)), "thr": float(body.get("thr", 0)), "t": time.monotonic()}
                    return self._send(200, b'{"ok":true}')
                if p == "/api/sweep":
                    threading.Thread(target=sess.run_sweep, args=(body.get("kind", "speeds"),), daemon=True).start()
                    return self._send(200, b'{"ok":true}')
                return self._send(404, b"{}")
            except Exception as e:
                return self._send(400, json.dumps({"error": str(e)}).encode())

    srv = ThreadingHTTPServer(("127.0.0.1", port), H)
    print(f"PiRacer simulator: http://localhost:{port}", flush=True)
    srv.serve_forever()
