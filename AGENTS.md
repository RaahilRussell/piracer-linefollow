# Instructions for AI agents

Before doing anything in this repository:

1. **Read [`CHANGELOG.md`](CHANGELOG.md)**: what changed, why, the current race settings, and what is still unverified.
2. **Read [`HANDOFF.md`](HANDOFF.md)**: the working rules, the race-morning checklist and the history. Where it differs from `CHANGELOG.md`, the changelog is newer.
3. **Read [`README.md`](README.md)**: how to run the car, the simulator and the tests.

Rules (from the owner):
- **Never** write to the Pi, deploy, start the drive program, commit or push without asking first. Never type or send the Pi's password.
- All tuning values live in `pi/line_config.py`. Every tunable value must stay a dashboard slider (`pi/webui.py` KNOBS + `pi/laptuner.html`); `tests/test_guided_safety.py` checks this.
- Before claiming a setting is better, check it with `python3 tools/race_eval.py --pairs`. Say "simulated" for simulator results; only real laps count as real.
- Keep the suite green: `python3 -B -m unittest discover -s tests -q`. Run `./car.sh check --quick` before any deploy.
- When you change behaviour or defaults, **add an entry at the top of `CHANGELOG.md`**: what, why, and the numbers.
