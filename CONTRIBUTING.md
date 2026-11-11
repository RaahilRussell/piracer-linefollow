# Contributing

This is an active physical robotics project. Changes that affect steering, throttle, camera geometry, presets, deployment, or safety need extra care.

## Before changing code

1. Read [`CHANGELOG.md`](CHANGELOG.md) and [`HANDOFF_REVIEW.md`](HANDOFF_REVIEW.md).
2. Identify whether the change affects the physical car, the complimentary simulator, or both.
3. Keep tunable values represented by dashboard controls when they are intended to be changed during driving.

## Before opening a change

```bash
python3 -B -m unittest discover -s tests -q
python3 tools/race_eval.py --pairs
```

Do not deploy to the Pi or start the drive program as part of an automated check.

## Recording physical results

For every physical comparison, record:

- preset and Racing Line settings
- battery condition or voltage
- forward PWM
- straight, transition, and curve throttle
- lap count and lap-time range
- line losses, wide laps, and manual interventions

Keep physical and simulated claims separate in documentation.
