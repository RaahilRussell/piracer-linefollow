"""Unit and property-based tests for the guided lap speed ladder."""
import os
import sys
import unittest

from hypothesis import given, strategies as st

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "pi"))
import lap_tuning  # noqa: E402


def frames(n, *, tracking=True, error=0.04, steering=0.2):
    return [{"status": "TRACKING" if tracking else "LOST", "found": tracking,
             "error": error if tracking else None, "steering": steering} for _ in range(n)]


class TestLapAssessment(unittest.TestCase):
    def test_too_few_samples_repeat_without_speed_increase(self):
        r = lap_tuning.assess_lap(frames(39), 0.55)
        self.assertEqual(r["result"], "inconclusive")
        self.assertEqual(r["next_speed"], 0.55)

    def test_clean_lap_stages_small_speed_increase(self):
        r = lap_tuning.assess_lap(frames(100), 0.55)
        self.assertEqual(r["result"], "passed")
        self.assertEqual(r["next_speed"], 0.58)

    def test_lost_line_stops_speed_ladder(self):
        r = lap_tuning.assess_lap(frames(100, tracking=False), 0.58)
        self.assertEqual(r["result"], "failed")
        self.assertEqual(r["next_speed"], 0.58)

    def test_large_error_stops_ladder_even_if_detector_reports_tracking(self):
        r = lap_tuning.assess_lap(frames(100, error=0.28), 0.58)
        self.assertEqual(r["result"], "failed")
        self.assertEqual(r["next_speed"], 0.58)

    def test_steering_at_limit_is_reported_but_does_not_fail_good_tracking(self):
        r = lap_tuning.assess_lap(frames(100, steering=-1.0), 0.55)
        self.assertEqual(r["result"], "passed")
        self.assertEqual(r["steering_limit_pct"], 100.0)

    def test_mixed_tracking_repeats_same_speed(self):
        samples = frames(90) + frames(10, tracking=False)
        r = lap_tuning.assess_lap(samples, 0.55)
        self.assertEqual(r["result"], "repeat")
        self.assertEqual(r["next_speed"], 0.55)

    def test_configured_maximum_is_never_exceeded(self):
        r = lap_tuning.assess_lap(frames(100), 0.99)
        self.assertLessEqual(r["next_speed"], 1.0)

    def test_invalid_speed_rejected(self):
        for value in (-0.01, 1.01, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                lap_tuning.assess_lap(frames(40), value)

    @given(
        speed=st.floats(min_value=0, max_value=1, allow_nan=False, allow_infinity=False),
        n=st.integers(min_value=40, max_value=300),
        error=st.floats(min_value=0, max_value=0.12, allow_nan=False, allow_infinity=False),
        steering=st.floats(min_value=-1, max_value=1, allow_nan=False, allow_infinity=False),
    )
    def test_property_clean_tracking_never_fails_due_to_steering(self, speed, n, error, steering):
        result = lap_tuning.assess_lap(frames(n, error=error, steering=steering), speed)
        self.assertIn(result["result"], ("passed", "repeat"))
        self.assertGreaterEqual(result["next_speed"], speed)
        self.assertLessEqual(result["next_speed"], 1.0)

    @given(
        speed=st.floats(min_value=0, max_value=1, allow_nan=False, allow_infinity=False),
        n=st.integers(min_value=40, max_value=300),
        lost=st.integers(min_value=1, max_value=300),
    )
    def test_property_clear_tracking_failure_never_increases_speed(self, speed, n, lost):
        samples = frames(lost, tracking=False) + frames(n)
        if n / len(samples) < 0.85:
            result = lap_tuning.assess_lap(samples, speed)
            self.assertEqual(result["result"], "failed")
            self.assertEqual(result["next_speed"], speed)


if __name__ == "__main__":
    unittest.main()
