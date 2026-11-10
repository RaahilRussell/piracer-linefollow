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

## How the line is tracked

The car follows the **left** tape's inside edge; the right tape is tracked alongside it as the backup measurement (right edge minus the learned lane width). `linetrack/detector.py` does this per frame:

1. **Mask.** A strict orange threshold, grown a few pixels into nearly-orange pixels. Glare pushes parts of the tape just outside the strict range; growing fills those holes without admitting look-alike surfaces that have no strictly-orange pixels nearby.
2. **Candidates.** Orange runs on each scanline are linked row to row, bridging small gaps, so a line broken into fragments is still one candidate. Fat or ragged-edged blobs (hands, wood grain) are dropped.
3. **Identity.** Each boundary remembers the fit of its edge. A candidate lying on that fit *is* that boundary, wherever it has moved in the image. Geometry is only used to start a track: two lines one lane apart with the car between them, or a single line by the side of the car it passes on. A single near-vertical line under the car with no history is ambiguous and reported as lost.
4. **Output.** The left edge at the lookahead row, or the right edge minus the lane width, with the switch-over offset faded out.

The pilot's one-line safety stop (`SINGLE_LINE_STOP_S`) only runs while the tracker is not sure which line it has: a line that has been tracked without a break since both were in view does not trip it. The overlay shows `1-LINE` (sure) or `1-LINE?` (not sure).

Check a change against recordings before it goes on the car:

```bash
python3 tools/backtest.py --datasets carpet_run,track_run --variants baseline,no_grow,confirm2
python3 -m unittest tests.test_lane_tracker tests.test_linetrack
```

`runs/` holds recordings copied from the Pi's `~/linefollow/data` (not in version control).

## Main folders

- `linetrack/`: OpenCV detector, PD controller, dashboard, and Donkeycar link.
- `sim/`: vehicle, camera, track, physical-boundary checks, and gain schedule tuner.
- `pi/`: Pi-side pilot/dashboard source and Mac-side recording review tool.
- `tools/`: frame replay, camera calibration, and contact-sheet utilities.
- `tests/`: unit tests and small image fixtures.

Local recordings, downloaded Pi data, and generated reports are excluded from version control.
