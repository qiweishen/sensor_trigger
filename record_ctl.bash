#!/usr/bin/env bash
# record_ctl.sh - programmatic session control for the SensorSync-Logger.
#
# Call this from your acquisition driver. Each `start` opens a NEW file
#   logs/session_<YYYYmmdd_HHMMSS>.log   and prints its path to stdout.
#
#   record_ctl.sh start [outdir]   # begin a new session; prints the log path; returns at once
#   record_ctl.sh stop             # end the current session
#   record_ctl.sh new  [outdir]    # stop the current one (if any) + start a new one
#   record_ctl.sh status           # "recording -> <file>"  or  "idle"
#
# Env:  BAUD (default 115200)   RECORD_STATE (default /tmp/sensorsync_record.state)
# One reader per port: don't also run record.sh / pio device monitor on the same Teensy.
# The logger keeps a single file per session; a brief USB glitch reconnects and APPENDS
# to the SAME file (same session). A new file is created only by `start` / `new`.

set -u
BAUD="${BAUD:-115200}"
STATE="${RECORD_STATE:-/tmp/sensorsync_record.state}"   # line1 = worker PID, line2 = logfile

find_port() {
	local p
	for p in /dev/serial/by-id/*[Tt]eensy* /dev/serial/by-id/usb-*; do [ -e "$p" ] && { echo "$p"; return 0; }; done
	for p in /dev/ttyACM* /dev/ttyUSB*; do [ -e "$p" ] && { echo "$p"; return 0; }; done
	return 1
}

running_pid() { [ -f "$STATE" ] && head -1 "$STATE" 2>/dev/null; }
is_running() { local p; p=$(running_pid); [ -n "${p:-}" ] && kill -0 "$p" 2>/dev/null; }

# Background worker (own process group via setsid): read the port into $1, forever, sending
# 'h' on each (re)connect so the file always has a header. Killed as a group by stop().
_worker() {
	local file="$1" port
	while :; do
		port=$(find_port) || { sleep 1; continue; }
		stty -F "$port" raw -echo "$BAUD" 2>/dev/null || { sleep 1; continue; }
		( sleep 0.3; printf 'h\n' > "$port" 2>/dev/null ) &
		cat "$port" >> "$file" 2>/dev/null   # blocks until the port drops
		sleep 0.5
	done
}

start() {
	local outdir="${1:-./logs}"
	if is_running; then echo "already recording: $(sed -n 2p "$STATE")" >&2; return 1; fi
	mkdir -p "$outdir"
	local file="$outdir/session_$(date +%Y%m%d_%H%M%S).log"
	setsid bash "$0" __worker "$file" </dev/null >/dev/null 2>&1 &
	local pid=$!
	printf '%s\n%s\n' "$pid" "$file" > "$STATE"
	echo "$file"                              # <- the driver reads this to know the log path
}

stop() {
	if [ ! -f "$STATE" ]; then echo "not recording" >&2; return 1; fi
	local pid file; pid=$(running_pid); file=$(sed -n 2p "$STATE")
	[ -n "${pid:-}" ] && kill -TERM -"$pid" 2>/dev/null   # negative PID = kill the whole group
	rm -f "$STATE"
	local sz=""
	[ -f "$file" ] && sz=" ($(wc -c <"$file" 2>/dev/null) bytes)"
	echo "stopped: $file$sz" >&2
}

case "${1:-}" in
	__worker) shift; _worker "$1" ;;                       # internal
	start)    shift; start "$@" ;;
	stop)     stop ;;
	new)      shift; stop >/dev/null 2>&1; sleep 0.3; start "$@" ;;
	status)   if is_running; then echo "recording -> $(sed -n 2p "$STATE")"; else echo "idle"; fi ;;
	*) echo "usage: $0 {start [outdir] | stop | new [outdir] | status}" >&2; exit 2 ;;
esac
