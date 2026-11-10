"""Run: python3 -m unittest -v tests.test_linetrack   (from the project root)

Real frames in tests/frames/ were captured from the live Donkeycar /video stream
on 2026-11-08 (240x192). Synthetic frames are drawn with known geometry so edge
positions can be checked exactly.
"""
import glob
import io
import json
import os
import threading
import time
import unittest

import cv2
import numpy as np

from linetrack.app import DRIVE_CONFIRM_PHRASE, Engine
from linetrack.carlink import MjpegReader, WsDriver
from linetrack.config import Config
from linetrack.controller import DRIVE, PREVIEW, STEER_ONLY, SteeringController
from linetrack.detector import LeftLineDetector

HERE = os.path.dirname(__file__)
W, H = 240, 192
MAT = (200, 165, 195)      # lavender mat (BGR), like the real course
ORANGE = (50, 90, 225)


def stripe(img, x_of_y, thick_of_y, y_top, y_bot=H):
    """Fill a stripe whose LEFT edge is x_of_y(y) and width thick_of_y(y)."""
    for y in range(int(y_top), int(y_bot)):
        xl = x_of_y(y)
        xr = xl + thick_of_y(y)
        a, b = int(round(xl)), int(round(xr))
        a, b = max(a, 0), min(b, W)
        if b > a:
            img[y, a:b] = ORANGE


def scene(left_shift=0.0, right=True, left=True, curve=0.0, right_thick=3.0):
    """Two-boundary course. Left boundary starts ~0.30W at the bottom and leans RIGHT
    going up; right boundary starts ~0.92W and leans LEFT going up (more prominent)."""
    img = np.full((H, W, 3), MAT, np.uint8)
    yt = 0.42 * H
    def u(y):
        return (H - y) / (H - yt)                  # 0 at bottom, 1 at roi top
    if left:
        stripe(img, lambda y: (0.30 + left_shift) * W + 28 * u(y) + curve * W * u(y) ** 2,
               lambda y: 22 - 14 * u(y), yt)
    if right:
        # boundaries are parallel on the course, so the right one bends with the same curve
        stripe(img, lambda y: 0.92 * W - 80 * u(y) + curve * W * u(y) ** 2,
               lambda y: right_thick * (26 - 14 * u(y)), yt)
    return img


def left_right_edge_truth(y, left_shift=0.0, curve=0.0):
    yt = 0.42 * H
    uu = (H - y) / (H - yt)
    xl = (0.30 + left_shift) * W + 28 * uu + curve * W * uu ** 2
    return xl + 22 - 14 * uu


class TestDetector(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.det = LeftLineDetector(self.cfg)

    # 1 + 2 on synthetic geometry
    def test_selects_near_left_and_rejects_prominent_far_right(self):
        d = self.det.detect(scene(right_thick=3.0))
        self.assertTrue(d.found, d.reason)
        self.assertLess(d.selected.near_x / W, 0.45)
        right = [s for s in d.stripes if s.role in ("RIGHT", "RIGHT_REF")]
        self.assertEqual(len(right), 1)
        self.assertEqual(d.source, "left")
        self.assertGreater(right[0].area, d.selected.area * 2, "far-right stripe should be the bigger one")
        self.assertGreater(right[0].near_x / W, 0.7)

    def test_lone_right_stripe_is_never_tracked_as_left(self):
        self.cfg.use_right_fallback = False
        d = self.det.detect(scene(left=False))
        self.assertFalse(d.found)
        self.assertIsNone(d.selected)
        self.assertEqual(d.reason, "only right boundary visible")

    def test_lone_right_stripe_uses_fallback_with_lane_width(self):
        d = self.det.detect(scene(left=False))
        self.assertIsNone(d.selected, "right stripe must not become the LEFT track")
        self.assertTrue(d.found, d.reason)
        self.assertEqual(d.source, "right")
        self.assertEqual(d.right_selected.role, "RIGHT_REF")
        self.assertAlmostEqual(d.x_ratio, d.right_x_ratio - self.cfg.lane_width_ratio, places=3)

    def test_left_disappears_falls_back_smoothly(self):
        self.cfg.lane_width_ratio = 0.30          # synthetic lane is ~0.27 W wide at lookahead
        for _ in range(4):
            d = self.det.detect(scene())
            self.assertEqual(d.source, "left")
        left_x = d.x_ratio
        self.assertAlmostEqual(self.det._lane_w, d.right_x_ratio - left_x, delta=0.02)
        for _ in range(5):
            d = self.det.detect(scene(left=False))
            self.assertIsNone(d.selected)
            self.assertEqual(d.source, "right")
            self.assertLess(abs(d.x_ratio - left_x), 0.03, "virtual left must match the real one")

    def test_handoff_has_no_jump_even_with_wrong_lane_width(self):
        for _ in range(3):
            prev = self.det.detect(scene()).x_ratio
        self.det._lane_w = 0.45                   # simulate a badly learned lane width (true ~0.27)
        d = self.det.detect(scene(left=False))
        self.assertLess(abs(d.x_ratio - prev), 0.01, "no step at the switch")
        raw = d.right_x_ratio - 0.45
        for _ in range(30):
            d = self.det.detect(scene(left=False))
        self.assertAlmostEqual(d.x_ratio, raw, delta=0.01)   # offset faded out

    def test_horizon_fragment_not_used_as_right_reference(self):
        img = scene(right=False)
        img[int(0.46 * H):int(0.48 * H), 140:230] = ORANGE   # thin band at the top of the ROI
        d = self.det.detect(img)
        self.assertIsNone(d.right_selected)
        self.assertEqual(d.source, "left")

    def test_wood_floor_colour_mostly_rejected(self):
        # Stage wood measured H 4-11 (median 8), V 107-190 (median 151). Hue >= 10 or
        # V < 140 is rejected by colour; the median wood colour still passes and is
        # handled by the ROI (horizon ~0.42 H) and the right-reference rules instead.
        for hsv in ((10, 122, 151), (8, 122, 130), (11, 110, 180)):
            img = scene(left=False, right=False)
            img[100:190, 20:60] = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
            d = self.det.detect(img)
            self.assertEqual(int((d.mask > 0).sum()), 0, hsv)

    # 3
    def test_extracts_right_inside_edge(self):
        d = self.det.detect(scene())
        self.assertGreater(len(d.edge_points), 15)
        err = [abs(x - left_right_edge_truth(y)) for x, y in d.edge_points]
        self.assertLess(max(err), 1.6, f"edge error {max(err):.2f}px")
        # it is the right edge, not the centre/left edge: every point is right of the stripe centre
        for x, y in d.edge_points:
            self.assertGreater(x, left_right_edge_truth(y) - 4)

    # 5
    def test_curved_line_lookahead(self):
        for curve in (0.15, 0.08, -0.10, -0.18):
            self.det.reset()
            d = self.det.detect(scene(curve=curve))
            self.assertTrue(d.found, d.reason)
            truth = left_right_edge_truth(d.lookahead_y, curve=curve)
            self.assertLess(abs(d.lookahead_x - truth), 2.5, f"curve {curve}: {d.lookahead_x:.1f} vs {truth:.1f}")

    def test_upscaled_input_maps_back_to_original_coords(self):
        big = cv2.resize(scene(), (W * 2, H * 2), interpolation=cv2.INTER_NEAREST)
        d = self.det.detect(big)
        self.assertTrue(d.found)
        truth = 2 * left_right_edge_truth(d.lookahead_y / 2)
        self.assertLess(abs(d.lookahead_x - truth), 4)

    # 6
    def test_no_line(self):
        d = self.det.detect(np.full((H, W, 3), MAT, np.uint8))
        self.assertFalse(d.found)
        self.assertEqual(d.reason, "no orange")

    def test_hand_sized_blob_rejected(self):
        img = scene(left=False, right=False)
        cv2.rectangle(img, (60, 100), (180, 190), ORANGE, -1)
        self.assertFalse(self.det.detect(img).found)


class TestRealFrames(unittest.TestCase):
    """Frames captured from the live car stream."""

    def test_first_live_frame_tracks_left_fragment_not_centre_stripe(self):
        # The reference view: a small left stripe fragment (x~0.03-0.08) and the big
        # centre stripe (x~0.42). The user confirmed the LEFT one must be followed.
        det = LeftLineDetector(Config())
        for name in ("live_001", "live_060", "live_090", "live_120"):
            d = det.detect(cv2.imread(os.path.join(HERE, "frames", f"{name}.jpg")))
            self.assertTrue(d.found, f"{name}: {d.reason}")
            self.assertLess(d.selected.near_x / W, 0.1, name)
            self.assertLess(d.x_ratio, 0.12, name)
            centre = [s for s in d.stripes if s.area > 1500]
            self.assertIn(centre[0].role, ("RIGHT", "RIGHT_REF"), name)
            self.assertEqual(d.source, "left", name)

    def test_centre_stripe_alone_is_rejected(self):
        img = cv2.imread(os.path.join(HERE, "frames", "live_060.jpg"))
        img[88:126, 0:32] = img[88:126, 40:72]          # erase the left fragment
        d = LeftLineDetector(Config()).detect(img)
        self.assertFalse(d.found)

    def test_live_sequence_never_tracks_a_right_boundary(self):
        files = sorted(glob.glob(os.path.join(HERE, "frames", "live2", "*.jpg")))
        self.assertGreater(len(files), 20)
        cfg = Config()
        det = LeftLineDetector(cfg)
        tracked = 0
        for f in files:
            d = det.detect(cv2.imread(f))
            if d.found:
                tracked += 1
                s = d.selected
                if s is None:          # following the right line via fallback
                    continue
                self.assertFalse(s.near_x > cfg.reject_right_ratio * W and s.lean < -cfg.lean_tolerance,
                                 f"{os.path.basename(f)} tracked a right-boundary-like stripe")
        self.assertGreater(tracked, len(files) // 3)

    def test_stage_laps_fallback_coverage(self):
        # 298 frames of real driving on the stage loop (5 fps sample of the live stream)
        files = sorted(glob.glob(os.path.join(HERE, "frames", "stage", "*.jpg")))
        self.assertGreater(len(files), 250)
        cfg = Config()
        det = LeftLineDetector(cfg)
        res = [det.detect(cv2.imread(f)) for f in files]
        tracked = sum(r.found for r in res) / len(res)
        right = sum(r.found and r.source == "right" for r in res)
        self.assertGreater(tracked, 0.85, f"tracked {tracked:.0%}")
        self.assertGreater(right, 30, "fallback should be carrying the inside-line gaps")
        jumps = [abs(res[i].x_ratio - res[i - 1].x_ratio) for i in range(1, len(res))
                 if res[i].found and res[i - 1].found and res[i].source != res[i - 1].source]
        self.assertLess(float(np.percentile(jumps, 90)), 0.15)
        for r in res:
            if r.found and r.selected is not None:
                s = r.selected
                self.assertFalse(s.near_x > cfg.reject_right_ratio * W and s.lean < -cfg.lean_tolerance)

    def test_processing_time(self):
        det = LeftLineDetector(Config())
        img = cv2.imread(os.path.join(HERE, "frames", "live_060.jpg"))
        det.detect(img)
        t = time.perf_counter()
        for _ in range(200):
            det.detect(img)
        ms = (time.perf_counter() - t) / 200 * 1000
        self.assertLess(ms, 15, f"{ms:.2f} ms/frame")


class TestController(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.cfg.steering_smoothing = 0.0
        self.cfg.steering_rate_limit = 100.0

    def run_ctrl(self, x, mode=PREVIEW, n=5, found=True, age=0.0, t0=0.0, c=None):
        c = c or SteeringController(self.cfg)
        cmd = None
        for i in range(n):
            cmd = c.update(x, found, t0 + i * 0.05, 0.05, age, mode)
        return cmd, c

    # 4
    def test_steering_direction(self):
        self.assertGreater(self.run_ctrl(0.35)[0].angle, 0.1, "line right of target -> steer right (+)")
        self.assertLess(self.run_ctrl(0.05)[0].angle, -0.1, "line left of target -> steer left (-)")
        self.assertAlmostEqual(self.run_ctrl(0.20)[0].angle, 0.0, places=3)
        self.cfg.target_x_ratio = 0.80
        self.assertLess(self.run_ctrl(0.60)[0].angle, -0.1)
        self.cfg.invert_steering = True
        self.assertGreater(self.run_ctrl(0.60)[0].angle, 0.1)

    def test_steering_direction_from_detector(self):
        cfg = Config()
        cfg.steering_rate_limit = 100.0
        det = LeftLineDetector(cfg)
        c = SteeringController(cfg)
        base = det.detect(scene()).x_ratio
        det.reset()
        shifted = det.detect(scene(left_shift=0.12)).x_ratio
        self.assertGreater(shifted, base + 0.1)
        a1 = c.update(base, True, 0, 0.05, 0, PREVIEW).angle
        c.reset()
        a2 = c.update(shifted, True, 0, 0.05, 0, PREVIEW).angle
        self.assertGreater(a2, a1)

    # 8
    def test_clipping(self):
        self.cfg.max_steering = 0.6
        self.assertAlmostEqual(self.run_ctrl(1.0)[0].angle, 0.6)
        self.assertAlmostEqual(self.run_ctrl(-1.0)[0].angle, -0.6)

    def test_rate_limit_and_smoothing(self):
        self.cfg.steering_rate_limit = 2.0            # 0.1 per 50 ms step
        c = SteeringController(self.cfg)
        prev = 0.0
        for i in range(6):
            a = c.update(1.0, True, i * 0.05, 0.05, 0, PREVIEW).angle
            self.assertLessEqual(a - prev, 0.1 + 1e-9)
            prev = a
        self.cfg.steering_rate_limit = 100
        self.cfg.steering_smoothing = 0.5
        c = SteeringController(self.cfg)
        a = c.update(0.4, True, 0, 0.05, 0, PREVIEW).angle
        self.assertAlmostEqual(a, 0.5 * min(self.cfg.kp * 0.2, self.cfg.max_steering), places=3)

    # 6
    def test_loss_zeroes_throttle_holds_then_recentres(self):
        c = SteeringController(self.cfg)
        cmd = c.update(0.35, True, 0.0, 0.05, 0, DRIVE)
        self.assertEqual(cmd.status, "TRACKING")
        self.assertGreater(cmd.throttle, 0)
        held = cmd.angle
        cmd = c.update(None, False, 0.1, 0.05, 0, DRIVE)
        self.assertEqual((cmd.status, cmd.throttle), ("LOST", 0.0))
        self.assertAlmostEqual(cmd.angle, held)          # brief dropout: hold
        cmd = c.update(None, False, 0.1 + self.cfg.lost_timeout_s + 0.1, 0.05, 0, DRIVE)
        self.assertAlmostEqual(cmd.angle, 0.0)           # after timeout: centre

    # 7
    def test_stale_frames_zero_throttle(self):
        c = SteeringController(self.cfg)
        cmd = c.update(0.3, True, 0, 0.05, self.cfg.max_frame_age_s + 0.01, DRIVE)
        self.assertEqual((cmd.status, cmd.throttle), ("STALE", 0.0))

    def test_throttle_only_in_drive_and_capped(self):
        self.cfg.throttle = 5.0
        self.cfg.update({})                               # cap applied on update
        self.assertLessEqual(self.cfg.throttle, self.cfg.max_throttle)
        for m in (PREVIEW, STEER_ONLY):
            self.assertEqual(self.run_ctrl(0.3, mode=m)[0].throttle, 0.0)
        self.assertLessEqual(self.run_ctrl(0.3, mode=DRIVE)[0].throttle, self.cfg.max_throttle)


# ---------------------------------------------------------------- I/O fakes
class FakeWS:
    def __init__(self, fail=False):
        self.sent, self.fail, self.closed = [], fail, False

    def send(self, s):
        if self.fail:
            raise ConnectionError("link down")
        self.sent.append(json.loads(s))

    def close(self):
        self.closed = True


def jpeg(img):
    return cv2.imencode(".jpg", img)[1].tobytes()


class FakeStream(io.RawIOBase):
    def __init__(self, parts, then_raise=True):
        self.data = b"".join(b"--boundarydonotcross\r\nContent-Type: image/jpeg\r\n\r\n" + p + b"\r\n" for p in parts)
        self.pos, self.then_raise = 0, then_raise

    def read(self, n=-1):
        if self.pos >= len(self.data):
            if self.then_raise:
                raise TimeoutError("timed out")
            return b""
        out = self.data[self.pos:self.pos + n]
        self.pos += len(out)
        return out


class TestIO(unittest.TestCase):
    # 7
    def test_mjpeg_dedup_and_disconnect(self):
        a, b = jpeg(scene()), jpeg(scene(left_shift=0.05))
        opened = []

        def opener(url, t):
            opened.append(url)
            if len(opened) > 1:
                raise ConnectionRefusedError("server down")
            return FakeStream([a, a, a, b, b])

        r = MjpegReader("http://car/video", opener=opener).start()
        time.sleep(0.4)
        seq, img, t = r.decode_latest()
        self.assertEqual(seq, 2, "repeated JPEGs must not count as new frames")
        self.assertEqual(img.shape, (H, W, 3))
        self.assertFalse(r.connected)
        self.assertTrue(r.error)
        self.assertGreater(time.monotonic() - t, 0.3)   # frame age keeps growing -> STALE
        r.stop()

    def test_driver_silent_until_armed_and_close_does_not_touch_car(self):
        calls = []
        d = WsDriver("ws://car/wsDrive", 50, connect=lambda u: calls.append(u) or FakeWS()).start()
        d.set(0.5, 0.5)
        time.sleep(0.15)
        d.close()
        self.assertEqual(calls, [], "preview mode must never open the control socket")

    def test_driver_estop_sends_zero(self):
        ws = FakeWS()
        d = WsDriver("ws://car/wsDrive", 50, connect=lambda u: ws).start()
        d.arm()
        d.set(0.4, 0.2)
        time.sleep(0.1)
        self.assertTrue(any(m["throttle"] == 0.2 for m in ws.sent))
        d.estop()
        n = len(ws.sent)
        self.assertEqual(ws.sent[-1], {"angle": 0.0, "throttle": 0.0, "drive_mode": "user", "recording": False})
        time.sleep(0.1)
        self.assertEqual(len(ws.sent), n, "nothing sent after e-stop")
        d.close()

    def test_driver_link_failure_disarms(self):
        d = WsDriver("ws://car/wsDrive", 50, connect=lambda u: FakeWS(fail=True)).start()
        d.arm()
        time.sleep(0.1)
        self.assertFalse(d.armed)
        self.assertIn("link down", d.error)
        d.close()


class StaticSource:
    def __init__(self, img, fresh=True):
        self.img, self.fresh, self.seq = img, fresh, 0
        self.connected, self.error, self.camera_fps = True, "", 20

    def decode_latest(self):
        if self.fresh:
            self.seq += 1
            return self.seq, self.img.copy(), time.monotonic()
        return 1, self.img.copy(), time.monotonic() - 5.0


class TestEngineSafety(unittest.TestCase):
    def make(self, src, enable_drive=False):
        ws = FakeWS()
        drv = WsDriver("ws://car/wsDrive", 50, connect=lambda u: ws).start()
        eng = Engine(Config(), src, drv, enable_drive, "/dev/null")
        th = threading.Thread(target=eng.run, daemon=True)
        th.start()
        return eng, drv, ws

    # 9
    def test_autonomy_disabled_by_default(self):
        eng, drv, ws = self.make(StaticSource(scene()))
        try:
            time.sleep(0.2)
            self.assertEqual(eng.mode, PREVIEW)
            self.assertFalse(drv.armed)
            self.assertEqual(ws.sent, [])
            ok, msg = eng.set_mode(DRIVE, DRIVE_CONFIRM_PHRASE)
            self.assertFalse(ok)
            self.assertIn("--enable-drive", msg)
            self.assertFalse(eng.set_mode(STEER_ONLY, "")[0])
        finally:
            eng.stop(); drv.close()

    def test_drive_requires_phrase_and_estop_works(self):
        eng, drv, ws = self.make(StaticSource(scene()), enable_drive=True)
        try:
            self.assertFalse(eng.set_mode(DRIVE, "yes")[0])
            self.assertTrue(eng.set_mode(DRIVE, DRIVE_CONFIRM_PHRASE)[0])
            for _ in range(10):
                eng.heartbeat(); time.sleep(0.03)
            self.assertTrue(any(m["throttle"] > 0 for m in ws.sent))
            eng.estop()
            self.assertEqual(eng.mode, PREVIEW)
            self.assertEqual(ws.sent[-1]["throttle"], 0.0)
        finally:
            eng.stop(); drv.close()

    def test_steer_only_never_sends_throttle(self):
        eng, drv, ws = self.make(StaticSource(scene()), enable_drive=True)
        try:
            self.assertTrue(eng.set_mode(STEER_ONLY, "WHEELS LIFTED")[0])
            for _ in range(10):
                eng.heartbeat(); time.sleep(0.03)
            self.assertTrue(ws.sent)
            self.assertTrue(all(m["throttle"] == 0.0 for m in ws.sent))
            self.assertTrue(any(abs(m["angle"]) > 0.05 for m in ws.sent))
        finally:
            eng.stop(); drv.close()

    def test_stale_video_stops_drive(self):
        src = StaticSource(scene(), fresh=False)
        eng, drv, ws = self.make(src, enable_drive=True)
        try:
            eng.set_mode(DRIVE, DRIVE_CONFIRM_PHRASE)
            for _ in range(30):
                eng.heartbeat(); time.sleep(0.03)
            self.assertEqual(eng.mode, PREVIEW)
            self.assertTrue(all(m["throttle"] == 0.0 for m in ws.sent))
            self.assertIn("STALE", eng.mode_note)
        finally:
            eng.stop(); drv.close()

    def test_lost_line_stops_drive(self):
        src = StaticSource(scene())
        eng, drv, ws = self.make(src, enable_drive=True)
        try:
            eng.set_mode(DRIVE, DRIVE_CONFIRM_PHRASE)
            for _ in range(5):
                eng.heartbeat(); time.sleep(0.03)
            src.img = np.full((H, W, 3), MAT, np.uint8)  # no lines at all
            for _ in range(30):
                eng.heartbeat(); time.sleep(0.03)
            self.assertEqual(eng.mode, PREVIEW)
            self.assertEqual(ws.sent[-1]["throttle"], 0.0)
        finally:
            eng.stop(); drv.close()

    def test_dashboard_heartbeat_loss_stops(self):
        eng, drv, ws = self.make(StaticSource(scene()), enable_drive=True)
        try:
            eng.set_mode(DRIVE, DRIVE_CONFIRM_PHRASE)
            time.sleep(eng.cfg.dashboard_heartbeat_s + 0.3)   # no heartbeats
            self.assertEqual(eng.mode, PREVIEW)
            self.assertIn("heartbeat", eng.mode_note)
            self.assertEqual(ws.sent[-1]["throttle"], 0.0)
        finally:
            eng.stop(); drv.close()


if __name__ == "__main__":
    unittest.main()
