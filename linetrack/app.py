"""Processing loop + local browser dashboard (stdlib HTTP server, no frameworks).

Runs on the Mac. Open http://localhost:8090 after starting.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from .carlink import MjpegReader, WsDriver
from .config import Config
from .controller import DRIVE, MODES, PREVIEW, STEER_ONLY, SteeringController
from .detector import LeftLineDetector

DRIVE_CONFIRM_PHRASE = "ONBOARD FAILSAFE VERIFIED"


class FileSource:
    """Replays saved frames (a file, a glob, or a directory) as if they were live."""

    def __init__(self, pattern: str, fps: float = 20.0):
        p = Path(pattern)
        files = sorted(glob.glob(str(p / "*.jpg")) + glob.glob(str(p / "*.png"))) if p.is_dir() else sorted(glob.glob(pattern))
        if not files:
            raise SystemExit(f"no images match {pattern}")
        self.frames = [cv2.imread(f) for f in files]
        self.fps = fps
        self.t0 = time.monotonic()
        self.connected, self.error, self.raw_fps, self.camera_fps = True, "", fps, fps

    def start(self):
        return self

    def stop(self):
        pass

    def decode_latest(self):
        k = int((time.monotonic() - self.t0) * self.fps)
        return k, self.frames[k % len(self.frames)].copy(), self.t0 + k / self.fps


class Engine:
    def __init__(self, cfg: Config, source, driver: WsDriver | None, enable_drive: bool,
                 config_path: str):
        self.cfg = cfg
        self.src = source
        self.driver = driver
        self.enable_drive = enable_drive
        self.config_path = config_path
        self.detector = LeftLineDetector(cfg)
        self.ctrl = SteeringController(cfg)
        self.mode = PREVIEW
        self.mode_note = "preview only: car is not being commanded"
        self.lock = threading.Lock()
        self.frame_cv = threading.Condition()
        self.jpeg: bytes | None = None
        self.jpeg_seq = 0
        self.state: dict = {}
        self.last_heartbeat = time.monotonic()
        self._stop = threading.Event()
        self._fps = 0.0
        self._lost_since: float | None = None

    # ------------------------------------------------------------ control
    def set_mode(self, mode: str, confirm: str = "") -> tuple[bool, str]:
        with self.lock:
            if mode not in MODES:
                return False, f"unknown mode {mode}"
            if mode == PREVIEW:
                self._to_preview("preview selected")
                return True, self.mode_note
            if self.driver is None:
                return False, "no vehicle link (running on saved frames)"
            if mode == STEER_ONLY and confirm != "WHEELS LIFTED":
                return False, "steering test needs confirmation: wheels lifted off the ground"
            if mode == DRIVE:
                if not self.enable_drive:
                    return False, "DRIVE is disabled. Restart with --enable-drive after verifying an on-car failsafe."
                if confirm != DRIVE_CONFIRM_PHRASE:
                    return False, f'type exactly "{DRIVE_CONFIRM_PHRASE}" to enable driving'
            self.ctrl.reset()
            self.detector.reset()
            self.driver.arm()
            self.mode = mode
            self._lost_since = None
            self.last_heartbeat = time.monotonic()
            self.mode_note = ("STEERING ONLY: throttle forced to 0" if mode == STEER_ONLY
                              else f"DRIVING: throttle {self.cfg.throttle:.2f} while TRACKING")
            return True, self.mode_note

    def estop(self, why: str = "E-STOP"):
        with self.lock:
            self._to_preview(why)

    def _to_preview(self, why: str):
        if self.driver is not None:
            self.driver.disarm(why)
        self.mode = PREVIEW
        self.mode_note = why
        self.ctrl.reset()

    def heartbeat(self):
        self.last_heartbeat = time.monotonic()

    # ------------------------------------------------------------ loop
    def run(self):
        last_seq, t_prev = -1, time.monotonic()
        fps_n, fps_t = 0, time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            # dashboard dead-man: if nobody is watching, nothing moves
            if self.mode != PREVIEW and now - self.last_heartbeat > self.cfg.dashboard_heartbeat_s:
                self.estop("stopped: dashboard heartbeat lost")
            if self.driver is not None and self.mode != PREVIEW and not self.driver.armed:
                self.estop(f"stopped: vehicle link failed ({self.driver.error})")

            seq, frame, t_frame = self.src.decode_latest()
            age = now - t_frame if frame is not None else 1e9
            if frame is None or seq == last_seq:
                if age > self.cfg.max_frame_age_s and now - t_prev > 0.05:
                    cmd = self._apply(None, now, now - t_prev, age, None)  # stale -> throttle 0
                    t_prev = now
                    with self.frame_cv:
                        self.state = {**self.state, "status": "STALE", "mode": self.mode,
                                      "mode_note": self.mode_note, "steering": round(cmd.angle, 3),
                                      "throttle": 0.0, "frame_age_ms": round(min(age, 99) * 1000),
                                      "video_connected": getattr(self.src, "connected", False),
                                      "video_error": getattr(self.src, "error", "")}
                time.sleep(0.003)
                continue
            last_seq = seq
            t0 = time.perf_counter()
            det = self.detector.detect(frame)
            dt = now - t_prev
            t_prev = now
            cmd = self._apply(det, now, dt, age, frame)
            proc_ms = (time.perf_counter() - t0) * 1000
            fps_n += 1
            if now - fps_t >= 1.0:
                self._fps, fps_n, fps_t = fps_n / (now - fps_t), 0, now
            self._publish(frame, det, cmd, proc_ms, age)

    def _apply(self, det, now, dt, age, frame):
        found = bool(det and det.found)
        xr = det.x_ratio if det else None
        with self.lock:
            cmd = self.ctrl.update(xr, found, now, dt, age, self.mode)
            if self.mode == DRIVE:
                if cmd.status != "TRACKING":
                    self._lost_since = self._lost_since or now
                    if now - self._lost_since > self.cfg.lost_timeout_s:
                        self._to_preview(f"stopped: line {cmd.status} for > {self.cfg.lost_timeout_s}s")
                else:
                    self._lost_since = None
            if self.driver is not None and self.mode != PREVIEW:
                self.driver.set(cmd.angle, cmd.throttle if self.mode == DRIVE else 0.0)
        return cmd

    # ------------------------------------------------------------ render
    def _publish(self, frame, det, cmd, proc_ms, age):
        st = {
            "status": cmd.status, "mode": self.mode, "mode_note": self.mode_note,
            "confidence": round(det.confidence, 3), "reason": det.reason,
            "source": det.source if det.found else "none",
            "left_conf": round(det.left_confidence, 2), "right_conf": round(det.right_confidence, 2),
            "lane_width": round(det.lane_width_ratio, 3),
            "x_ratio": None if det.x_ratio is None else round(det.x_ratio, 4),
            "target": self.cfg.target_x_ratio, "error": round(cmd.error, 4),
            "steering": round(cmd.angle, 3), "raw": round(cmd.raw, 3),
            "throttle": round(cmd.throttle if self.mode == DRIVE else 0.0, 3),
            "fps": round(self._fps, 1), "proc_ms": round(proc_ms, 2),
            "frame_age_ms": round(age * 1000, 1),
            "camera_fps": round(getattr(self.src, "camera_fps", 0), 1),
            "video_connected": getattr(self.src, "connected", False),
            "video_error": getattr(self.src, "error", ""),
            "ws_connected": bool(self.driver and self.driver.connected),
            "ws_sent": self.driver.sent if self.driver else 0,
            "ws_last": self.driver.last_msg if self.driver else None,
            "drive_enabled": self.enable_drive,
            "stripes": [{"role": s.role, "near_x": round(s.near_x / (frame.shape[1] / det.scale), 3),
                         "lean": round(s.lean, 2), "area": s.area} for s in det.stripes],
        }
        img = render(frame, det, cmd, st, self.cfg)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with self.frame_cv:
            self.state = st
            if ok:
                self.jpeg = buf.tobytes()
                self.jpeg_seq += 1
            self.frame_cv.notify_all()

    def stop(self):
        self._stop.set()


ROLE_COLORS = {"LEFT": (0, 220, 0), "RIGHT_REF": (0, 190, 255), "RIGHT": (0, 0, 255), "other": (150, 150, 150)}
ROLE_LABELS = {"LEFT": "TRACKED LEFT", "RIGHT_REF": "RIGHT (fallback ref)", "RIGHT": "IGNORED (right)", "other": "ignored"}


def render(frame, det, cmd, st, cfg, zoom=2):
    H, W = frame.shape[:2]
    big = cv2.resize(frame, (W * zoom, H * zoom), interpolation=cv2.INTER_LINEAR)
    s = zoom
    # ROI shading
    ry = int(cfg.roi_top_ratio * H * s)
    big[:ry] = (big[:ry] * 0.45).astype(np.uint8)
    # stripes: bounding boxes with role labels (makes a wrong pick obvious)
    sc = det.scale * s
    for stp in det.stripes:
        x, y, w, h = stp.bbox
        col = ROLE_COLORS.get(stp.role, (150, 150, 150))
        cv2.rectangle(big, (int(x * sc), int(y * sc)), (int((x + w) * sc), int((y + h) * sc)), col, 2)
        label = ROLE_LABELS.get(stp.role, stp.role)
        cv2.putText(big, label, (int(x * sc) + 2, max(12, int(y * sc) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
    # target line (cyan)
    tx = int(cfg.target_x_ratio * W * s)
    cv2.line(big, (tx, ry), (tx, H * s), (255, 255, 0), 2)
    # lookahead row
    ly = int(cfg.lookahead_ratio * H * s)
    cv2.line(big, (0, ly), (W * s, ly), (255, 255, 0), 1, cv2.LINE_AA)
    # edge points + fitted curve (green)
    for x, y in det.edge_points:
        cv2.circle(big, (int(x * s), int(y * s)), 2, (0, 255, 0), -1)
    if len(det.curve) > 1:
        cv2.polylines(big, [(det.curve * s).astype(np.int32)], False, (0, 255, 0), 2, cv2.LINE_AA)
    # right boundary inside edge (amber); used only when the left line is lost
    for x, y in det.right_points:
        cv2.circle(big, (int(x * s), int(y * s)), 2, (0, 190, 255), -1)
    if len(det.right_curve) > 1:
        cv2.polylines(big, [(det.right_curve * s).astype(np.int32)], False, (0, 190, 255), 2, cv2.LINE_AA)
    if det.lookahead_x is not None:
        p = (int(det.lookahead_x * s), int(det.lookahead_y * s))
        cv2.circle(big, p, 7, (255, 0, 255), 2)
        cv2.line(big, (tx, p[1]), p, (255, 0, 255), 1)
    # HUD
    good = st["status"] == "TRACKING"
    col = (0, 220, 0) if good else (0, 0, 255)
    cv2.rectangle(big, (0, 0), (W * s, 44), (0, 0, 0), -1)
    src = {"left": "LEFT line", "right": "RIGHT line - lane", "none": "-"}.get(st.get("source", "none"), "-")
    cv2.putText(big, f"{st['status']} [{src}]  conf {st['confidence']:.2f}  {st['mode'].upper()}",
                (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    cv2.putText(big, f"steer {cmd.angle:+.2f} ({'RIGHT' if cmd.angle > 0.02 else 'LEFT' if cmd.angle < -0.02 else 'STRAIGHT'})"
                f"  thr {st['throttle']:.2f}  {st['fps']:.0f}fps  {st['proc_ms']:.1f}ms  age {st['frame_age_ms']:.0f}ms",
                (6, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    # steering bar
    cx, by = W * s // 2, H * s - 10
    cv2.rectangle(big, (cx - 100, by - 6), (cx + 100, by + 6), (40, 40, 40), -1)
    cv2.rectangle(big, (cx, by - 6), (cx + int(100 * cmd.angle), by + 6), (0, 200, 255), -1)
    cv2.line(big, (cx, by - 9), (cx, by + 9), (255, 255, 255), 1)
    # mask panel: colour components by role
    m = det.mask
    panel = np.zeros((m.shape[0], m.shape[1], 3), np.uint8)
    panel[m > 0] = (110, 110, 110)
    if det.selected is not None:
        x, y, w, h = det.selected.bbox
        sub = m[y:y + h, x:x + w] > 0
        panel[y:y + h, x:x + w][sub] = (0, 220, 0)
    for stp in det.stripes:
        if stp.role in ("RIGHT", "RIGHT_REF"):
            x, y, w, h = stp.bbox
            sub = m[y:y + h, x:x + w] > 0
            panel[y:y + h, x:x + w][sub] = (0, 0, 220) if stp.role == "RIGHT" else (0, 190, 255)
    panel = cv2.resize(panel, (W * s, H * s), interpolation=cv2.INTER_NEAREST)
    cv2.putText(panel, "green=left  amber=right ref  red=ignored", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return np.hstack([big, panel])


# ---------------------------------------------------------------- dashboard
PAGE = (Path(__file__).with_name("dashboard.html")).read_text


def make_handler(engine: Engine):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            b = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path == "/":
                b = PAGE().encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path.startswith("/api/state"):
                engine.heartbeat()
                self._json({**engine.state, "config": engine.cfg.as_dict()})
            elif self.path.startswith("/stream.mjpg"):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                seen = -1
                try:
                    while True:
                        with engine.frame_cv:
                            engine.frame_cv.wait_for(lambda: engine.jpeg_seq != seen, timeout=1.0)
                            seen, jpg = engine.jpeg_seq, engine.jpeg
                        if jpg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._json({"ok": False, "msg": "bad json"}, 400)
            if self.path == "/api/estop":
                engine.estop("E-STOP pressed")
                return self._json({"ok": True})
            if self.path == "/api/mode":
                ok, msg = engine.set_mode(body.get("mode", ""), body.get("confirm", ""))
                return self._json({"ok": ok, "msg": msg})
            if self.path == "/api/config":
                body.pop("car_url", None)  # connection settings are CLI-only
                applied = engine.cfg.update(body)
                return self._json({"ok": True, "applied": applied})
            if self.path == "/api/save":
                engine.cfg.save(engine.config_path)
                return self._json({"ok": True, "msg": f"saved {engine.config_path}"})
            self.send_error(404)

    return H


def main(argv=None):
    ap = argparse.ArgumentParser(description="Orange left-line tracker for Donkeycar (runs on the Mac)")
    ap.add_argument("--car", help="Donkeycar base URL (default from config)")
    ap.add_argument("--config", default="config.json", help="JSON config file (created by Save)")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--target", type=float, help="override target_x_ratio, e.g. 0.20 or 0.80")
    ap.add_argument("--source", help="replay saved frames (file, glob or dir) instead of the live stream")
    ap.add_argument("--enable-drive", action="store_true",
                    help="allow DRIVE mode (throttle>0). Only after verifying an on-car failsafe.")
    args = ap.parse_args(argv)

    cfg = Config.load(args.config)
    if args.car:
        cfg.car_url = args.car.rstrip("/")
    if args.target is not None:
        cfg.target_x_ratio = args.target

    if args.source:
        src, driver = FileSource(args.source).start(), None
    else:
        src = MjpegReader(cfg.car_url + cfg.video_path).start()
        ws = cfg.car_url.replace("http://", "ws://").replace("https://", "wss://") + cfg.ws_path
        driver = WsDriver(ws, cfg.send_hz).start()

    eng = Engine(cfg, src, driver, args.enable_drive, args.config)

    def shutdown(*_):
        eng.estop("shutdown")
        if driver:
            driver.close()
        eng.stop()
        os._exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    threading.Thread(target=eng.run, daemon=True, name="engine").start()
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(eng))
    srv.daemon_threads = True
    print(f"video : {args.source or cfg.car_url + cfg.video_path}")
    print(f"mode  : PREVIEW (car is not commanded). DRIVE {'available' if args.enable_drive else 'disabled'}")
    print(f"open  : http://localhost:{args.port}")
    try:
        srv.serve_forever()
    finally:
        shutdown()


if __name__ == "__main__":
    main()
