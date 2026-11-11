# Architecture

PiRacer Pro Autonomous Racer is split into reusable perception and geometry code, a Raspberry Pi drive runtime, a browser dashboard, and a complimentary closed-loop simulator.

```text
fisheye camera
      ↓
linetrack.detector
      ↓
ground projection + bend fit
      ↓
linetrack.racing_line
      ↓
pure pursuit + PD correction
      ↓
speed plan + safety guards
      ↓
PiRacer steering and throttle
```

## Main layers

- [`linetrack/detector.py`](../linetrack/detector.py) detects orange boundaries and tracks their identity across frames.
- [`linetrack/geometry.py`](../linetrack/geometry.py) maps image points to the ground plane, fits bends, and computes pursuit steering.
- [`linetrack/racing_line.py`](../linetrack/racing_line.py) shifts the target ride line toward a controlled apex while respecting the car's turning radius.
- [`pi/donkey_part.py`](../pi/donkey_part.py) connects perception, control, throttle, line-loss handling, and telemetry to the physical drive stack.
- [`pi/webui.py`](../pi/webui.py) serves the live camera, settings, presets, lap timer, safety controls, and telemetry.
- [`sim/`](../sim/) renders a track and vehicle so the same detector and pilot can be exercised without the car.
- [`tools/`](../tools/) contains calibration, replay, backtesting, and evaluation tools.

The simulator reuses the real detector and pilot path, but it still depends on assumptions that must be checked against physical driving.
