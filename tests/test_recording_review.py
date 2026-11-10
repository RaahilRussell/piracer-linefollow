"""Read-only download boundaries and local replay/simulation review."""
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "pi"), str(ROOT)]
from recording_review import RecordingReview, extract_recordings, pi_host


class RecordingReviewTests(unittest.TestCase):
    def archive(self, path, name, link=False):
        with tarfile.open(path, "w") as tf:
            item = tarfile.TarInfo(name)
            if link:
                item.type = tarfile.SYMTYPE
                item.linkname = "/tmp/outside"
                tf.addfile(item)
            else:
                item.size = 4
                tf.addfile(item, io.BytesIO(b"data"))

    def test_host_rejects_shell_options_and_injection(self):
        self.assertEqual(pi_host("192.0.2.1"), "192.0.2.1")
        for host in ("-oProxyCommand=anything", "1.2.3.4; touch bad", "host/path"):
            with self.assertRaises(ValueError):
                pi_host(host)

    def test_archive_only_extracts_recording_files(self):
        with tempfile.TemporaryDirectory() as d:
            archive = Path(d) / "in.tar"
            self.archive(archive, "./20261109-010000/000001.jpg")
            extract_recordings(archive, Path(d) / "out")
            self.assertEqual((Path(d) / "out/20261109-010000/000001.jpg").read_bytes(), b"data")

    def test_archive_blocks_traversal_and_links(self):
        with tempfile.TemporaryDirectory() as d:
            archive = Path(d) / "in.tar"
            for name, link in (("../escape", False), ("/absolute", False), ("20261109-010000/link", True)):
                self.archive(archive, name, link)
                with self.assertRaises(ValueError):
                    extract_recordings(archive, Path(d) / "out")

    def test_download_cannot_overwrite_local_files(self):
        with tempfile.TemporaryDirectory() as d:
            archive = Path(d) / "in.tar"
            self.archive(archive, "20261109-010000/000001.jpg")
            extract_recordings(archive, Path(d) / "out")
            with self.assertRaises(FileExistsError):
                extract_recordings(archive, Path(d) / "out")

    def test_no_timestamps_are_explicitly_assumed(self):
        folder = ROOT / "tests/frames/live2"
        files = sorted(folder.glob("*.jpg"))
        times, _, measured = RecordingReview.frame_times(folder, files)
        self.assertFalse(measured)
        self.assertEqual(len(times), len(files))

    def test_cancel_is_checked_by_processing(self):
        review = RecordingReview()
        review.cancel()
        with self.assertRaises(InterruptedError):
            review.check_cancel()

    def test_worker_reports_download_failure(self):
        with tempfile.TemporaryDirectory() as d:
            review = RecordingReview(d)
            with patch.object(review, "_download", side_effect=RuntimeError("offline")):
                review._worker(Path(d) / "job", "192.0.2.1", {}, False)
            self.assertEqual(review.status()["state"], "failed")
            self.assertEqual(review.status()["error"], "offline")

    def test_review_runs_real_replay_and_simulation_without_applying(self):
        knobs = {"kp": 3.5, "kd": .4, "max_auto": .6, "speed_straight": .6, "speed_corner": .55,
                 "speed_transition": .57, "speed_ramp_up": .3}
        review = RecordingReview()
        report = review.analyze(ROOT / "tests/frames", knobs, deep=True)
        self.assertGreater(report["frames"], 0)
        self.assertEqual(len(report["simulations"]), 10)
        self.assertEqual(len(report["robustness"]), 8)
        self.assertTrue(report["inferences"])
        self.assertTrue(any(r["split"] == "held_out" for r in report["runs"]))
        self.assertTrue(any("never" in s or "automatically" in s for s in report["limits"]))
        json.dumps(report, allow_nan=False)
        self.assertEqual(report["settings"], knobs)


if __name__ == "__main__":
    unittest.main()
