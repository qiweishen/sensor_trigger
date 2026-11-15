#!/usr/bin/env bash
# Background recorder around session_client.
#   start [outdir] [freqspec]   prints the new log path once the board is #READY
#   stop                        STOP, wait for the client, report the result
#   status
# Env: SENSOR_PORT (default: first Teensy port), SESSION_CLIENT.
set -u
DIR=$(cd "$(dirname "$0")" && pwd)
CLIENT="${SESSION_CLIENT:-$DIR/session_client}"
STATE=/tmp/sensorsync_record.state	# "<pid> <log>"

port() {
	[ -n "${SENSOR_PORT:-}" ] && { echo "$SENSOR_PORT"; return; }
	ls /dev/serial/by-id/*[Tt]eensy* /dev/ttyACM* 2>/dev/null | head -n 1
}

case "${1:-}" in
start)
	[ -f "$STATE" ] && { echo "already recording: $(cut -d' ' -f2- "$STATE")" >&2; exit 1; }
	mkdir -p "${2:-./logs}" || exit 1
	log="$(cd "${2:-./logs}" && pwd)/session_$(date +%Y%m%d_%H%M%S).log"
	setsid "$CLIENT" "$(port)" "$log" 0 "${3:-}" </dev/null >"$log.stderr" 2>&1 &
	echo "$! $log" >"$STATE"
	for _ in $(seq 50); do
		grep -q '^recording' "$log.stderr" && { echo "$log"; exit 0; }
		kill -0 $! 2>/dev/null || break
		sleep 0.1
	done
	kill -INT $! 2>/dev/null
	rm -f "$STATE"
	tail -n 1 "$log.stderr" >&2
	exit 1
	;;
stop)
	read -r pid log <"$STATE" 2>/dev/null || { echo "not recording" >&2; exit 1; }
	kill -INT "$pid" 2>/dev/null
	while kill -0 "$pid" 2>/dev/null; do sleep 0.1; done
	rm -f "$STATE"
	grep -q '^done' "$log.stderr" || { tail -n 1 "$log.stderr" >&2; exit 1; }
	echo "stopped: $log" >&2
	;;
status)
	[ -f "$STATE" ] && echo "recording -> $(cut -d' ' -f2- "$STATE")" || echo idle
	;;
*)
	echo "usage: $0 start [outdir] [freqspec] | stop | status" >&2
	exit 2
	;;
esac
