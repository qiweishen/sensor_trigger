#!/usr/bin/env bash
# Shell lifecycle wrapper around the C++ session client; one protocol implementation.
# start [outdir] [FX:hz,JAI:hz] | stop | new [outdir] [FX:hz,JAI:hz] | status
# Env: SENSOR_PORT, SESSION_CLIENT, RECORD_STATE. USB baud is nominally 115200.
set -u
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
STATE="${RECORD_STATE:-/tmp/sensorsync_record.state}"
CLIENT="${SESSION_CLIENT:-$SCRIPT_DIR/session_client}"

find_port() {
    if [ -n "${SENSOR_PORT:-}" ]; then printf '%s\n' "$SENSOR_PORT"; return; fi
    local ports
    shopt -s nullglob
    ports=(/dev/serial/by-id/*[Tt]eensy*)
    [ "${#ports[@]}" -ne 0 ] || ports=(/dev/ttyACM*)
    if [ "${#ports[@]}" -ne 1 ]; then
        echo "set SENSOR_PORT: expected exactly one Teensy serial port" >&2
        return 1
    fi
    printf '%s\n' "${ports[0]}"
}

check_client() {
    if [ ! -x "$CLIENT" ]; then
        echo "build client first: g++ -std=c++20 -O2 -pthread -DSESSION_CLIENT_DEMO session_client.cpp -o session_client" >&2
        return 1
    fi
}

# Match both the PID and its exact logfile argument before sending a signal.
load_state() {
    [ -f "$STATE" ] || return 1
    mapfile -t RUN < "$STATE"
    [ "${#RUN[@]}" -eq 3 ] || { echo "invalid recorder state: $STATE" >&2; return 1; }
}
alive() { kill -0 "${RUN[0]}" 2>/dev/null && grep -Fzxq -- "${RUN[1]}" "/proc/${RUN[0]}/cmdline" 2>/dev/null; }
result() { cat -- "${RUN[1]}.status" 2>/dev/null || true; }

start() {
    if [ -f "$STATE" ]; then echo "existing run: use stop to collect its result first" >&2; return 1; fi
    check_client || return 1
    local outdir="${1:-./logs}" spec="${2:-}" port file pid i
    if [ -n "$spec" ]; then spec="freq=$spec"; fi
    port=$(find_port) || return 1
    mkdir -p -- "$outdir" || return 1
    outdir=$(cd -- "$outdir" && pwd) || return 1
    file="$outdir/session_$(date +%Y%m%d_%H%M%S_%N).log"
    setsid "$CLIENT" "$port" "$file" 0 "$spec" "$file.status" </dev/null >"$file.stderr" 2>&1 9>&- &
    pid=$!
    RUN=("$pid" "$file" "$port")
    if ! printf '%s\n' "${RUN[@]}" > "$STATE"; then
        kill -TERM "$pid" 2>/dev/null
        wait "$pid" 2>/dev/null
        return 1
    fi
    for ((i=0; i<80; ++i)); do
        case "$(result)" in
            RUNNING) printf '%s\n' "$file"; return 0 ;;
            FAILED|DONE) break ;;
        esac
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
    done
    alive && kill -TERM "$pid" 2>/dev/null
    echo "START failed; log: $file; details: $file.stderr (use stop to clear state)" >&2
    return 1
}

stop() {
    load_state || { echo "not recording or invalid state" >&2; return 1; }
    if alive; then kill -TERM "${RUN[0]}" 2>/dev/null || true; fi
    local final i
    for ((i=0; i<80; ++i)); do
        final=$(result)
        case "$final" in
            DONE|FAILED) break ;;
        esac
        alive || { final=FAILED; break; }
        sleep 0.1
    done
    if [ "$final" != DONE ] && [ "$final" != FAILED ]; then
        echo "STOP timed out; state retained: $STATE" >&2
        return 1
    fi
    rm -f -- "$STATE"
    if [ "$final" != DONE ]; then
        echo "recording failed: ${RUN[1]} (see ${RUN[1]}.stderr)" >&2
        return 1
    fi
    echo "stopped and verified: ${RUN[1]}" >&2
}

status() {
    if ! [ -f "$STATE" ]; then echo idle; return; fi
    load_state || return 1
    local final
    final=$(result)
    if [ "$final" = RUNNING ] && alive; then
        echo "recording -> ${RUN[1]} (port: ${RUN[2]})"
    else
        case "$final" in DONE|FAILED) ;; *) final=FAILED ;; esac
        echo "$final -> ${RUN[1]} (use stop to collect the result)"
    fi
}

# Serialize controllers; close this descriptor in the recorder so it cannot hold the lock.
exec 9>"$STATE.lock" || exit 1
flock -x 9 || exit 1
case "${1:-}" in
    start) shift; start "$@" ;;
    stop) stop ;;
    new) shift; if [ -f "$STATE" ]; then stop || exit 1; fi; start "$@" ;;
    status) status ;;
    *) echo "usage: $0 {start [outdir] [FX:hz,JAI:hz] | stop | new [outdir] [FX:hz,JAI:hz] | status}" >&2; exit 2 ;;
esac
