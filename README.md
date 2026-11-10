# PiRacer line following

OpenCV line detection, a PD steering controller, a vehicle simulator, and replay tools for a small tape-following car. The simulator and replay run locally on a Mac; they do not connect to or move the car.

## Open in VS Code

Open this folder, then choose **Terminal → New Terminal**. These commands run the OpenCV preview against the included image set:

```bash
python3 -m pip install -r requirements.txt
python3 -m linetrack --source tests/frames/live2
```

For the 2D simulated car and its physical limits, run:

```bash
python3 -m sim view
```

Open **http://localhost:8892**. The car view reports steering radius, grip, off-track events, and chassis clearance to both tape boundaries.

To use the dashboard with the simulated car:

```bash
python3 pi/sim_dashboard.py --sim 8891
```

Open **http://localhost:8891**. This mode is simulated; it does not control a PiRacer.

## Replay recordings

Replay a folder of numbered JPEGs and its optional `log.csv`:

```bash
python3 tools/backtest.py --frames-dir /path/to/run --dt 0.05 --datasets new_frames --json /tmp/backtest.json
```

The replay uses the local detector and pilot code. It compares image tracking and proposed controls; it cannot show how a changed controller would physically have moved the recorded car.

## Tune simulated speed profiles

```bash
python3 -m sim.tune
```

This generates model-based KP/KD starting points at six speed levels and checks a range of car and track assumptions. It is not a real-car calibration. The car's steering angle, speed response, grip, braking, camera mount, track radius, and lane width still need measurement; see [`sim/CALIBRATION.md`](sim/CALIBRATION.md) and [`sim/profile.json`](sim/profile.json).

The controller is **PD** (proportional plus derivative); there is no integral term. It supports separate straight and corner gains and speed targets.

## Main folders

- `linetrack/`: OpenCV detector, PD controller, dashboard, and Donkeycar link.
- `sim/`: vehicle, camera, track, physical-boundary checks, and gain schedule tuner.
- `pi/`: Pi-side pilot/dashboard source and Mac-side recording review tool.
- `tools/`: frame replay, camera calibration, and contact-sheet utilities.
- `tests/`: unit tests and small image fixtures.

Local recordings, downloaded Pi data, and generated reports are excluded from version control.
