"""Talking to the existing Donkeycar web server (nothing on the Pi is changed).

* MjpegReader: reads GET /video (multipart/x-mixed-replace) on a background thread
  and keeps ONLY the newest JPEG. The consumer decodes just that one with OpenCV,
  so old frames are dropped instead of queued. (cv2.VideoCapture on an HTTP MJPEG
  URL buffers internally and adds latency, so we parse the multipart ourselves.)
  Donkeycar re-sends the same JPEG many times per second (~140/s measured), so
  freshness is measured from the last time the JPEG *content* changed.

* WsDriver: sends {"angle","throttle","drive_mode","recording"} to /wsDrive, the
  same messages the Donkeycar web UI sends. It sends NOTHING while disarmed
  (preview mode), and sends a burst of zero-throttle messages whenever it is
  disarmed, e-stopped, or closed.
  The standard Donkeycar handler keeps the last angle/throttle when the socket
  drops, so a network loss between Mac and Pi does NOT stop the car by itself.
"""
from __future__ import annotations

import json
import threading
import time
import urllib.request
import zlib

import cv2
import numpy as np


class MjpegReader:
    def __init__(self, url: str, timeout: float = 3.0, opener=None):
        self.url = url
        self.timeout = timeout
        self._open = opener or (lambda u, t: urllib.request.urlopen(u, timeout=t))
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._t_changed = 0.0
        self._crc = None
        self.connected = False
        self.error = ""
        self.raw_fps = 0.0       # JPEGs received / s (including repeats)
        self.camera_fps = 0.0    # distinct images / s
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="mjpeg")

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()

    def latest(self):
        """(seq, jpeg_bytes, t_content_changed). seq increases only on new content."""
        with self._lock:
            return self._seq, self._jpeg, self._t_changed

    def decode_latest(self):
        seq, jpg, t = self.latest()
        if jpg is None:
            return seq, None, t
        return seq, cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR), t

    def _publish(self, jpg: bytes):
        crc = zlib.crc32(jpg)
        now = time.monotonic()
        with self._lock:
            if crc != self._crc:
                self._crc = crc
                self._jpeg = jpg
                self._seq += 1
                self._t_changed = now
                return True
        return False

    def _run(self):
        backoff = 0.5
        while not self._stop.is_set():
            try:
                resp = self._open(self.url, self.timeout)
                self.connected, self.error, backoff = True, "", 0.5
                self._consume(resp)
            except Exception as e:  # network errors, timeouts, closed stream
                self.error = f"{type(e).__name__}: {e}"
            self.connected = False
            self._stop.wait(backoff)
            backoff = min(backoff * 2, 3.0)

    def _consume(self, resp):
        buf = bytearray()
        n_raw = n_new = 0
        t0 = time.monotonic()
        while not self._stop.is_set():
            chunk = resp.read(8192)
            if not chunk:
                raise ConnectionError("stream closed")
            buf += chunk
            while True:
                a = buf.find(b"\xff\xd8")
                if a < 0:
                    del buf[:-1]
                    break
                b = buf.find(b"\xff\xd9", a + 2)
                if b < 0:
                    if a:
                        del buf[:a]
                    break
                n_raw += 1
                n_new += self._publish(bytes(buf[a:b + 2]))
                del buf[:b + 2]
            if len(buf) > 2_000_000:   # corrupt stream guard
                buf.clear()
            dt = time.monotonic() - t0
            if dt >= 1.0:
                self.raw_fps, self.camera_fps = n_raw / dt, n_new / dt
                n_raw = n_new = 0
                t0 = time.monotonic()


class WsDriver:
    """Sends commands only while armed. `connect` is injectable for tests."""

    def __init__(self, ws_url: str, send_hz: float = 20.0, connect=None):
        self.ws_url = ws_url
        self.period = 1.0 / send_hz
        self._connect = connect or self._default_connect
        self._ws = None
        self._lock = threading.Lock()
        self._armed = False
        self._ever_armed = False   # preview-only sessions never open the control socket
        self._angle = 0.0
        self._throttle = 0.0
        self.connected = False
        self.error = ""
        self.sent = 0
        self.last_msg: dict | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="wsdrive")

    @staticmethod
    def _default_connect(url):
        from websockets.sync.client import connect
        return connect(url, open_timeout=2, close_timeout=1)

    def start(self):
        self._thread.start()
        return self

    @property
    def armed(self):
        return self._armed

    def set(self, angle: float, throttle: float):
        with self._lock:
            self._angle, self._throttle = float(angle), float(throttle)

    def arm(self):
        with self._lock:
            self._angle, self._throttle = 0.0, 0.0
            self._armed = True
            self._ever_armed = True

    def disarm(self, reason: str = ""):
        """E-stop: zero throttle immediately (synchronously), then stop sending."""
        with self._lock:
            self._armed = False
            self._angle, self._throttle = 0.0, 0.0
        if not self._ever_armed:
            return
        for _ in range(3):
            self._send_now(0.0, 0.0)

    estop = disarm

    def close(self):
        self.disarm("close")
        self._stop.set()
        try:
            if self._ws:
                self._ws.close()
        except Exception:
            pass

    def _ensure(self):
        if self._ws is None:
            self._ws = self._connect(self.ws_url)
            self.connected, self.error = True, ""
        return self._ws

    def _send_now(self, angle, throttle) -> bool:
        msg = {"angle": round(angle, 4), "throttle": round(throttle, 4),
               "drive_mode": "user", "recording": False}
        try:
            self._ensure().send(json.dumps(msg))
            self.sent += 1
            self.last_msg = msg
            return True
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            self.connected = False
            try:
                if self._ws:
                    self._ws.close()
            except Exception:
                pass
            self._ws = None
            return False

    def _run(self):
        while not self._stop.is_set():
            t = time.monotonic()
            with self._lock:
                armed, a, th = self._armed, self._angle, self._throttle
            if armed:
                if not self._send_now(a, th):
                    # connection failed while armed: drop to disarmed; caller sees it
                    with self._lock:
                        self._armed = False
            self._stop.wait(max(0.0, self.period - (time.monotonic() - t)))
