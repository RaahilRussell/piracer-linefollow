# Results

Results are only meaningful with their conditions. Battery, PWM, speed profile, camera model, track, and line-loss behavior all matter.

## Best documented physical runs

| Setup | Evidence |
|---|---|
| Racing Line promotion check | Independent clean 9-lap timing window, about 3.05 s/lap mean |
| Racing Line slow validation | 12 laps, 4.46–5.01 s, zero line losses |
| Earlier Race baseline | 14 laps, 3.49–3.84 s, no wide laps |

## Controller progression

| Controller stage | Timing evidence | Interpretation |
|---|---|---|
| PD baseline | About **8.40 s/lap simulated** for the Nov 9, 9:31 pm setup; an earlier setup recorded 7.35 s/lap | A one-row image-error controller. It was a useful baseline, but it did not model the path shape ahead. |
| Pure pursuit / fixed-offset Race | First corrected-camera physical Race run: **3.68 s/lap mean**, 0 line losses. Later fresh-battery Race run: **3.59 s/lap mean over 10 laps** | Ground-projected steering toward a lookahead point made the car much more stable and faster than the PD baseline. |
| Racing Line + pure pursuit | Best documented physical result: **about 3.05 s/lap mean over an independent clean 9-lap window**. | A dynamic ride line uses straights, apexes, and exits instead of following one fixed offset. |

The simulator numbers are included for historical comparison, not as a claim that the car achieved those times. The physical car is the authority; the simulator was deliberately supplementary and was built under a less-than-one-week schedule with busy schedules and very limited car access.

These are documented in [`CHANGELOG.md`](../CHANGELOG.md). The current default profile and the best historical physical run are not automatically the same configuration; check the changelog before comparing numbers.

## Supplementary simulation

The race evaluator runs the real detector and pilot against a simulated camera and vehicle across varied assumptions. It is useful for repeatability and regression testing, but the model is not fully identified from physical measurements. We intentionally built more of it than the available schedule and car access strictly justified.

```bash
python3 tools/race_eval.py --pairs
```

Always label simulator results as `simulated` and physical results as `real`.
