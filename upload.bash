#!/usr/bin/env bash
# upload.bash - one-command build + flash + boot check for the SensorSync-Logger.
#
#   ./upload.bash            # build, flash the attached Teensy, wait for it to boot
#   ./upload.bash -b         # build only, no flash
#
# Refuses to flash while a recorder holds the serial port (stop it first).

set -u
cd "$(dirname "$0")"

# pio from PATH, else the default PlatformIO install
PIO=$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")
[ -x "$PIO" ] || { echo "!! pio not found - install PlatformIO first" >&2; exit 1; }

find_port() {
	local p
	for p in /dev/serial/by-id/*[Tt]eensy*; do [ -e "$p" ] && { echo "$p"; return 0; }; done
	for p in /dev/ttyACM*; do [ -e "$p" ] && { echo "$p"; return 0; }; done
	return 1
}

if [ "${1:-}" = "-b" ]; then
	exec "$PIO" run
fi

# a live reader (record_ctl worker / pio monitor) blocks clean re-enumeration
port=$(find_port) && fuser -s "$port" 2>/dev/null && {
	echo "!! $port is busy (recorder or monitor running) - stop it first, e.g.: ./record_ctl.bash stop" >&2
	exit 1
}

"$PIO" run -t upload || exit 1

# boot check: wait for re-enumeration, then for the ready banner / first heartbeat
printf 'waiting for the board to come back'
for _ in $(seq 1 20); do
	sleep 0.5
	printf '.'
	port=$(find_port) && break
done
echo
[ -n "${port:-}" ] || { echo "!! board did not re-enumerate - replug and check" >&2; exit 1; }
stty -F "$port" raw -echo 115200 2>/dev/null
banner=$(timeout 8 grep -m1 -a -e 'SensorSync-logger ready' -e '#IDLE' "$port" 2>/dev/null)
if [ -n "$banner" ]; then
	echo "OK: $port -> $banner"
else
	echo "flashed, but no boot banner seen on $port within 8 s - check manually" >&2
	exit 1
fi
