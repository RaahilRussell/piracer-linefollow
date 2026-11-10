#!/usr/bin/env bash
# Install the line follower + dashboard onto the Pi, then self-check.
# Writes ONLY inside ~/linefollow on the Pi. Does not start anything that moves the car.
#
#   ./deploy.sh <current-pi-ip>   check the bundle on this Mac, then install it on the Pi
#   ./deploy.sh --check           only check the bundle on this Mac (no Pi needed)
#
# Usually you don't call this yourself: ./car.sh deploys when the Pi's copy is out of date.
set -euo pipefail
cd "$(dirname "$0")"

# EVERYTHING the car runs. Add a file here when drive_line.py starts importing it; --check proves the list is complete.
PI_FILES=(donkey_part.py drive_line.py line_config.py webui.py dashboard.html laptuner.html tuning.py autotune.py
          diagnose.py lap_tuning.py gain_schedule.json)
LT_FILES=(linetrack/__init__.py linetrack/config.py linetrack/detector.py linetrack/controller.py linetrack/geometry.py)

bundle() { COPYFILE_DISABLE=1 tar --no-xattrs -czf - -C pi "${PI_FILES[@]}" -C .. "${LT_FILES[@]}"; }

check_bundle() { # unpack the exact bundle into an empty folder and build the car's parts from it, like drive_line.py
  if ! python3 -c "import numpy, cv2" 2>/dev/null; then
    echo "[bundle] this Mac's python3 lacks numpy/opencv, so the bundle can't be checked here:"
    echo "         python3 -m pip install -r requirements.txt"
    return 1
  fi
  local tmp; tmp="$(mktemp -d)"
  bundle | tar -xzf - -C "$tmp"
  (cd "$tmp" && python3 - <<'PY'
import os, sys
sys.path[:0] = [os.getcwd()]
import json
import line_config, autotune, diagnose, lap_tuning, tuning, webui  # noqa: F401  (everything drive_line.py imports)
from donkey_part import LineDetectorPart, LineFollowerPilot, car_model, load_line_config
cfg, ex = load_line_config(line_config)
model = car_model(ex)
pilot = LineFollowerPilot(cfg, model=model)
det = LineDetectorPart(cfg, show_overlay=False, model=model)
det.attach(pilot)
hub = webui.Hub(cfg, pilot, det, race_speeds=webui.race_speeds(ex), safe_speeds=webui.safe_speeds(ex))
json.load(open("gain_schedule.json"))
for page in ("dashboard.html", "laptuner.html"):
    assert os.path.getsize(page) > 1000, page
race = {p["id"]: p["values"] for p in hub.presets()}["race"]
print(f"[bundle] OK: {sum(len(f) for _, _, f in os.walk('.') if '__pycache__' not in _)} files import and build. Race preset {race['speed_straight']:.2f}/"
      f"{race['speed_corner']:.2f}, pure pursuit {bool(cfg.pure_pursuit)}, delay comp {cfg.pursuit_predict_s:.2f} s, "
      f"camera {model.cam_height_m:.3f} m / {model.cam_pitch_deg:.1f} deg")
PY
  ) || { rm -rf "$tmp"; echo "[bundle] FAILED: the files above would break on the car. Nothing was installed."; return 1; }
  rm -rf "$tmp"
}

if [ "${1:-}" = "--check" ]; then check_bundle; exit $?; fi
PI_IP="${1:?Usage: ./deploy.sh <current-pi-ip>   or   ./deploy.sh --check}"

echo "== checking the bundle on this Mac"
check_bundle
# The Pi's DHCP address changes. Keep host verification tied to its previously
# verified identity rather than treating each new address as a new machine.
SSH=(ssh -i "$HOME/.ssh/id_piracer" -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=8 -o HostKeyAlias=10.103.211.147 "pi@$PI_IP")

echo "== checking the Pi at $PI_IP"
"${SSH[@]}" 'echo "reachable, $(uptime -p)"'

echo "== refusing to install over a running drive program"
if "${SSH[@]}" 'ps -eo args | grep -qE "^(\S*/)?python3? (-u )?(\S*/)?(drive_line|manage)\.py"'; then
  echo "drive_line.py or manage.py is running on the Pi. Stop it first (gamepad X, or the dashboard's Quit program)."
  exit 1
fi

echo "== copying files into ~/linefollow"
bundle | "${SSH[@]}" 'mkdir -p ~/linefollow && tar -xzf - -C ~/linefollow'

echo "== offline self-check (no camera, no motors)"
"${SSH[@]}" 'cd ~/linefollow && timeout 90 ~/env/bin/python drive_line.py --check frames 2>&1 | grep "^\[check\]"'

echo "== ~/mycar must be unchanged (baseline from HANDOFF.md section 4)"
BASELINE='528feca15689d69c093ad682215f6bc94cfe57b50d1198d032e5861edc93c312  manage.py
dbeea891ccd748364e1236a5c583ae8ee598258e0fd32dff622a06cd51b69908  myconfig.py
d1f52816c59d1b2205e6807a978b413c5acc44fd0228cec01de115f863dd610d  config.py
f871c198dd4ad59980830996548d11910ace6c4ba10d9139db3467cafa26ed65  my_joystick.py
fb5cdf217a0e1782b459e64fcd1c335167ff753b62bb69588ce168a225edef6c  calibrate.py
32caa35375e528ec548ad4ed7544ed4df32a45eda394b8a8280d4c789c08c552  train.py'
CURRENT="$("${SSH[@]}" 'cd ~/mycar && sha256sum manage.py myconfig.py config.py my_joystick.py calibrate.py train.py')"
if [ "$CURRENT" = "$BASELINE" ]; then
  echo "OK: all 6 files in ~/mycar match the baseline"
else
  echo "WARNING: ~/mycar differs from the baseline. We never write there, so someone else changed it."
  echo "Re-read the calibration in myconfig.py before driving. Differences:"
  diff <(echo "$BASELINE") <(echo "$CURRENT") || true
fi

echo "== passwordless sudo (only needed for the dashboard's Shut down Pi button)"
"${SSH[@]}" 'sudo -n true 2>/dev/null && echo yes || echo "no: Shut down Pi will refuse; use Quit program then power off by hand"'

cat <<EOF

Installed. Nothing was started. To drive, on the Pi:
  cd ~/linefollow && ~/env/bin/python drive_line.py --js
then open the dashboard: http://$PI_IP:8890
EOF
