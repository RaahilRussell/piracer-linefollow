"""Conservative, one-lap-at-a-time speed ladder assessment."""
from __future__ import annotations

import math


def assess_lap(samples, speed, *, step=0.05, minimum_samples=40):
    """Assess recorded telemetry; steering saturation alone never defines failure."""
    speed = float(speed)
    if not math.isfinite(speed) or not 0 <= speed <= 1:
        raise ValueError("speed must be between 0 and 1")
    step = float(step)
    if not math.isfinite(step) or not 0.01 <= step <= 0.12:
        raise ValueError("speed step must be between 0.01 and 0.12")
    clean = [s for s in (samples or []) if isinstance(s, dict)]
    if len(clean) < minimum_samples:
        return {"result": "inconclusive", "samples": len(clean), "speed": speed,
                "next_speed": speed, "tracking_pct": None, "error_p90": None,
                "steering_limit_pct": None,
                "message": "Not enough telemetry; repeat this speed before increasing."}

    tracked = [s for s in clean if s.get("status") == "TRACKING" and s.get("found")]
    errors = sorted(abs(float(s["error"])) for s in tracked
                    if isinstance(s.get("error"), (int, float)) and math.isfinite(float(s["error"])))
    tracking_pct = 100 * len(tracked) / len(clean)
    p90 = errors[min(len(errors) - 1, math.ceil(0.9 * len(errors)) - 1)] if errors else None
    saturated = [s for s in clean if isinstance(s.get("steering"), (int, float))
                 and abs(float(s["steering"])) >= 0.95]
    saturation_pct = 100 * len(saturated) / len(clean)

    if tracking_pct < 85 or p90 is None or p90 > 0.22:
        result, next_speed = "failed", speed
        message = "Line tracking degraded; this is the failure point. Keep this speed stopped and retry at the last clean speed."
    elif tracking_pct >= 95 and p90 <= 0.12:
        result, next_speed = "passed", min(1.0, round(speed + step, 2))
        message = (f"Good line tracking. Next run is staged at {next_speed:.2f}; start it when ready."
                   if next_speed > speed else "Good line tracking at the configured speed limit.")
    else:
        result, next_speed = "repeat", speed
        message = "Mixed tracking; repeat this speed before increasing it."
    return {"result": result, "samples": len(clean), "speed": speed, "next_speed": next_speed,
            "tracking_pct": round(tracking_pct, 1), "error_p90": round(p90, 3) if p90 is not None else None,
            "steering_limit_pct": round(saturation_pct, 1), "message": message}
