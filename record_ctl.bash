#!/usr/bin/env bash
# record_ctl.bash - session control for the SensorSync-Logger.
#
# Each `start` = one new file logs/session_<YYYYmmdd_HHMMSS>.log, path on stdout.
#
#   record_ctl.bash start [outdir]   # begin a session; prints the log path; returns at once
#   record_ctl.bash stop             # end the current session
#   record_ctl.bash new  [outdir]    # stop (if any) + start
#   record_ctl.bash status           # "recording -> <file> (port: ...)" or "idle"
#
# Board side: `start` sends "START <file>" (board zeroes all counts and begins),
# `stop` sends "STOP" (board -> idle, counts cleared). C++ form: session_client.cpp.
#
# Env:  BAUD (default 115200)   RECORD_STATE (default /tmp/sensorsync_record.state)
# One reader per port: don't also run pio device monitor on the same Teensy.
# One file per session. A brief USB glitch reconnects and appends to the SAME
# file without re-START (seq stays continuous). If the board REBOOTS mid-run, the
# watchdog re-sends START: a new "#SESSION,START" marker lands in the same file
# and postprocess.py splits the sessions.

set -u
BAUD="${BAUD:-115200}"                                  # keep 115200
STATE="${RECORD_STATE:-/tmp/sensorsync_record.state}"   # line1 = worker PID, line2 = logfile

find_port() {
	# stable by-id symlink first, then generic USB-serial fallback
	local p
	for p in /dev/serial/by-id/*[Tt]eensy* /dev/serial/by-id/usb-*; do [ -e "$p" ] && { echo "$p"; return 0; }; done
	for p in /dev/ttyACM* /dev/ttyUSB*; do [ -e "$p" ] && { echo "$p"; return 0; }; done
	return 1
}

running_pid() { [ -f "$STATE" ] && head -1 "$STATE" 2>/dev/null; }
is_running() { local p; p=$(running_pid); [ -n "${p:-}" ] && kill -0 "$p" 2>/dev/null; }

# Background worker (own process group via setsid): capture the port into $1.
# First connect: send START at once. Watchdog: "#IDLE" in fresh output means the
# board is not in a session (boot / reboot / lost START) -> (re)send START.
_worker() {
	local file="$1" port catpid first=1
	while :; do
		port=$(find_port) || { sleep 1; continue; }
		stty -F "$port" raw -echo "$BAUD" 2>/dev/null || { sleep 1; continue; }
		cat "$port" >> "$file" 2>/dev/null &
		catpid=$!
		if [ "$first" = 1 ]; then
			sleep 0.3
			printf 'START %s\n' "$file" > "$port" 2>/dev/null
			first=0
		fi
		while kill -0 "$catpid" 2>/dev/null; do    # until the port drops
			sleep 6
			if tail -c 512 "$file" 2>/dev/null | grep -q '#IDLE'; then
				printf 'START %s\n' "$file" > "$port" 2>/dev/null
			fi
		done
		wait "$catpid" 2>/dev/null
		sleep 0.5
	done
}

start() {
	local outdir="${1:-./logs}" file pid
	if is_running; then echo "already recording: $(sed -n 2p "$STATE")" >&2; return 1; fi
	mkdir -p "$outdir"
	file="$outdir/session_$(date +%Y%m%d_%H%M%S).log"
	setsid bash "$0" __worker "$file" </dev/null >/dev/null 2>&1 &
	pid=$!
	printf '%s\n%s\n' "$pid" "$file" > "$STATE"
	echo "$file"                              # driver reads this: the session's log path
}

stop() {
	if [ ! -f "$STATE" ]; then echo "not recording" >&2; return 1; fi
	local pid file sz="" port
	pid=$(running_pid)
	file=$(sed -n 2p "$STATE")
	port=$(find_port) && printf 'STOP\n' > "$port" 2>/dev/null   # board -> idle
	sleep 0.3                                                    # let #SESSION,STOP land
	# PID-reuse guard: group-kill only a process that is really our worker
	if [ -n "${pid:-}" ] && grep -qa __worker "/proc/$pid/cmdline" 2>/dev/null; then
		kill -TERM -"$pid" 2>/dev/null
	fi
	rm -f "$STATE"
	[ -f "$file" ] && sz=" ($(wc -c < "$file") bytes)"
	echo "stopped: $file$sz" >&2
}

status() {
	local file port
	if is_running; then
		file=$(sed -n 2p "$STATE")
		port=$(find_port) || port="NONE - no board attached"
		echo "recording -> $file (port: $port)"
	else
		echo "idle"
	fi
}

case "${1:-}" in
	__worker) shift; _worker "$1" ;;                       # internal
	start)    shift; start "$@" ;;
	stop)     stop ;;
	new)      shift; stop >/dev/null 2>&1; sleep 0.3; start "$@" ;;
	status)   status ;;
	*) echo "usage: $0 {start [outdir] | stop | new [outdir] | status}" >&2; exit 2 ;;
esac
