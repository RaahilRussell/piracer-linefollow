#!/usr/bin/env bash
# Install the line follower + dashboard onto the Pi, then self-check.
# Writes ONLY inside ~/linefollow on the Pi. Does not start anything that moves the car.
#
#   ./deploy.sh <current-pi-ip>
set -euo pipefail
cd "$(dirname "$0")"

PI_IP="${1:?Usage: ./deploy.sh <current-pi-ip>}"
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
COPYFILE_DISABLE=1 tar --no-xattrs -czf - \
  -C pi donkey_part.py drive_line.py line_config.py webui.py dashboard.html laptuner.html tuning.py autotune.py diagnose.py lap_tuning.py gain_schedule.json \
  -C .. linetrack/__init__.py linetrack/config.py linetrack/detector.py linetrack/controller.py linetrack/geometry.py \
  | "${SSH[@]}" 'mkdir -p ~/linefollow && tar -xzf - -C ~/linefollow'

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
