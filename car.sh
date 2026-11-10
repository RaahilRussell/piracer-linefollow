#!/usr/bin/env bash
# One command for the real car: find the Pi, install if anything changed, start the drive program,
# open the dashboard, and stay connected (reconnects by itself if Wi-Fi drops or the IP changes).
#
#   ./car.sh            find the Pi, deploy if needed, start, open the dashboard, stay attached
#   ./car.sh 10.0.0.42  same, trying this IP first
#   ./car.sh open       just open the dashboard
#   ./car.sh log        stay attached to the drive program's output
#   ./car.sh status     is the Pi up, is the program running, what's deployed
#   ./car.sh restart    stop the drive program, deploy, start again
#   ./car.sh stop       stop the drive program (the Pi stays on)
#   ./car.sh ssh        a shell on the Pi
#   ./car.sh find       only find the Pi and remember its IP
#
# Ctrl-C only detaches this Mac: the drive program keeps running on the Pi.
# The Pi is recognised by its SSH host key, never by IP, so a wrong machine is never used.
# Writes only inside ~/linefollow on the Pi (via deploy.sh). Starting the program does not move the car:
# it starts in User mode; the car moves only from the gamepad or when you choose Full Auto.
set -uo pipefail
cd "$(dirname "$0")"

KEY="$HOME/.ssh/id_piracer"
ALIAS=10.103.211.147                      # the Pi's verified host key is stored under this name
STATE=.pi_ip                              # last IP that answered (not committed)
KNOWN_IPS="10.103.209.227 10.103.211.147" # earlier addresses
PORT=8890
CAR_ARGS="${CAR_ARGS:---js}"

ssh_pi() { # ssh_pi <ip> [command...]
  local ip="$1"; shift
  ssh -i "$KEY" -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=4 -o HostKeyAlias="$ALIAS" \
      -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -o LogLevel=ERROR "pi@$ip" "$@"
}
is_our_pi() { ssh_pi "$1" true 2>/dev/null; }

scan_subnet() { # print every host on the Mac's local network with port 22 open (at most 4096 addresses)
  local iface ip mask
  iface="$(route -n get default 2>/dev/null | awk '/interface:/{print $2}')"
  [ -n "$iface" ] || return 0
  ip="$(ipconfig getifaddr "$iface" 2>/dev/null)"; mask="$(ipconfig getoption "$iface" subnet_mask 2>/dev/null)"
  [ -n "$ip" ] && [ -n "$mask" ] || return 0
  python3 - "$ip" "$mask" <<'PY' | xargs -P 200 -I{} sh -c 'nc -z -G 1 {} 22 >/dev/null 2>&1 && echo {}'
import ipaddress, sys
net = ipaddress.ip_network(f"{sys.argv[1]}/{sys.argv[2]}", strict=False)
if net.num_addresses > 4096:
    net = ipaddress.ip_network(f"{sys.argv[1]}/20", strict=False)
for h in net.hosts():
    if str(h) != sys.argv[1]:
        print(h)
PY
}

find_pi() { # sets PI; tries the fast guesses first, then scans the network
  local first="${1:-}" c host
  local saved=""; [ -f "$STATE" ] && saved="$(cat "$STATE")"
  for c in $first ${PI_IP:-} $saved $KNOWN_IPS; do
    if is_our_pi "$c"; then PI="$c"; echo "$PI" > "$STATE"; return 0; fi
  done
  for host in $( [ -f .pi_host ] && cat .pi_host ) raspberrypi donkeypi piracer; do
    c="$(dscacheutil -q host -a name "$host.local" 2>/dev/null | awk '/ip_address/{print $2; exit}')"
    if [ -n "$c" ] && is_our_pi "$c"; then PI="$c"; echo "$PI" > "$STATE"; return 0; fi
  done
  echo "   not at a known address; scanning this network for the Pi (up to ~30 s)..." >&2
  for c in $(scan_subnet); do
    if is_our_pi "$c"; then PI="$c"; echo "$PI" > "$STATE"; return 0; fi
  done
  return 1
}

need_pi() {
  echo "== finding the Pi"
  until find_pi "${1:-}"; do
    echo "   Pi not found. Is it on and on the same Wi-Fi as this Mac? Retrying in 5 s (Ctrl-C to give up;"
    echo "   or run ./car.sh <ip> with the IP from 'hostname -I' on the Pi)."
    sleep 5
  done
  ssh_pi "$PI" hostname 2>/dev/null > .pi_host || true
  echo "   Pi at $PI ($(cat .pi_host 2>/dev/null))"
}

running() { ssh_pi "$PI" 'pgrep -f "python.*[d]rive_line\.py" >/dev/null' 2>/dev/null; }
local_sha() { cat pi/*.py pi/*.html pi/gain_schedule.json linetrack/*.py deploy.sh | shasum | cut -c1-12; }
remote_sha() { ssh_pi "$PI" 'cat ~/linefollow/.deployed 2>/dev/null' 2>/dev/null; }

deploy_if_needed() {
  local want; want="$(local_sha)"
  if [ "$(remote_sha)" = "$want" ]; then echo "== Pi already has this version ($want)"; return 0; fi
  if running; then
    echo "== the Pi runs an older version, but the drive program is running, so it was left alone."
    echo "   ./car.sh restart   installs the new version and starts it again."
    return 0
  fi
  echo "== installing the new version ($want)"
  ./deploy.sh "$PI" || { echo "deploy failed; nothing was started"; exit 1; }
  ssh_pi "$PI" "echo $want > ~/linefollow/.deployed"
}

wait_dashboard() {
  local i
  for i in $(seq 1 60); do
    curl -s -m 2 "http://$PI:$PORT/api/status" >/dev/null 2>&1 && return 0
    sleep 1
  done
  return 1
}

start() {
  if running; then echo "== drive program already running"; return 0; fi
  # A tuning.json saved by the version before pure pursuit (target 0.46, KP 4/4.5) would override every new
  # default at start-up: keep a copy and start clean.
  ssh_pi "$PI" 'f=~/linefollow/tuning.json; if [ -f "$f" ] && ! grep -q pure_pursuit "$f"; then
      mv "$f" "$f.old-$(date +%Y%m%d-%H%M)"; echo "   old tuning.json (from before pure pursuit) moved aside"; fi'
  echo "== starting the drive program (it starts in User mode: nothing moves until you drive or pick Full Auto)"
  ssh_pi "$PI" "mkdir -p ~/linefollow/logs && cd ~/linefollow && \
      setsid nohup ~/env/bin/python -u drive_line.py $CAR_ARGS > ~/linefollow/logs/drive.log 2>&1 < /dev/null &"
  if ! wait_dashboard; then
    echo "   the dashboard did not come up in 60 s; last lines of the program's output:"
    ssh_pi "$PI" 'tail -25 ~/linefollow/logs/drive.log'
    exit 1
  fi
}

open_dash() { echo "== dashboard: http://$PI:$PORT"; open "http://$PI:$PORT" 2>/dev/null || true; }

attach() { # follow the program's output; reconnect whenever the link drops
  echo "== attached (Ctrl-C detaches this Mac; the car keeps running. ./car.sh stop to stop it)"
  while true; do
    ssh_pi "$PI" 'tail -n 20 -F ~/linefollow/logs/drive.log' 2>/dev/null
    echo "-- connection to the Pi lost; reconnecting..."
    sleep 2
    find_pi "$PI" 2>/dev/null || { sleep 3; continue; }
    echo "-- reconnected at $PI (dashboard http://$PI:$PORT)"
  done
}

case "${1:-}" in
  open)    need_pi; open_dash ;;
  log)     need_pi; attach ;;
  ssh)     need_pi; exec ssh -i "$KEY" -o IdentitiesOnly=yes -o HostKeyAlias="$ALIAS" "pi@$PI" ;;
  find)    need_pi ;;
  stop)    need_pi; ssh_pi "$PI" 'pkill -INT -f "python.*[d]rive_line\.py"; sleep 2; pkill -f "python.*[d]rive_line\.py"; true'; echo "== stopped" ;;
  status)  need_pi
           echo "deployed: $(remote_sha || echo none)   this Mac: $(local_sha)"
           if running; then echo "drive program: running   dashboard: http://$PI:$PORT"; else echo "drive program: not running"; fi ;;
  restart) need_pi
           if running; then ssh_pi "$PI" 'pkill -INT -f "python.*[d]rive_line\.py"; sleep 2; pkill -f "python.*[d]rive_line\.py"; true'; sleep 1; fi
           deploy_if_needed; start; open_dash; attach ;;
  ""|[0-9]*) need_pi "${1:-}"; deploy_if_needed; start; open_dash; attach ;;
  *) sed -n 2,14p "$0"; exit 1 ;;
esac
