# Results

Results are only meaningful with their conditions. Battery, PWM, speed profile, camera model, track, and line-loss behavior all matter.

## Best documented physical runs

| Setup | Evidence |
|---|---|
| Racing Line promotion check | Independent clean 9-lap timing window, about 3.05 s/lap mean |
| Racing Line, 407 PWM, 1.00 / 0.80 straight/curve | 10 laps, 3.36–3.85 s, mean 3.57 s, one line loss |
| Racing Line slow validation | 12 laps, 4.46–5.01 s, zero line losses |
| Earlier Race baseline | 14 laps, 3.49–3.84 s, no wide laps |

These are documented in [`CHANGELOG.md`](../CHANGELOG.md). The current default profile and the best historical physical run are not automatically the same configuration; check the changelog before comparing numbers.

## Complimentary simulation

The race evaluator runs the real detector and pilot against a simulated camera and vehicle across varied assumptions. It is useful for repeatability and regression testing, but the model is not fully identified from physical measurements. We intentionally built more of it than the available schedule and car access strictly justified.

```bash
python3 tools/race_eval.py --pairs
```

Always label simulator results as `simulated` and physical results as `real`.
