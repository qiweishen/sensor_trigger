#!/usr/bin/env bash
# record.sh - log the SensorSync-Logger serial stream to a timestamped file.
#
#   * auto-detects the Teensy serial port
#   * one new timestamped file per (re)connection:  logs/session_YYYYmmdd_HHMMSS.log
#   * asks the board to re-print its #LOG/#TRIG/#STROBE header on connect (needs the
#     firmware 'h' command), so the log is self-contained even if you start it after boot
#   * auto-reconnects if the Teensy is reset / re-plugged
#   * pure stdlib (bash + stty + cat) - nothing to install
#
# Usage:
#   ./record.sh                # -> ./logs/session_*.log at 115200 baud
#   ./record.sh /path/to/dir   # choose the output directory
#   BAUD=9600 ./record.sh      # override the baud rate
#
# Stop with Ctrl-C. Do NOT run `pio device monitor` at the same time (only one reader).

set -u
BAUD="${BAUD:-115200}"
OUTDIR="${1:-./logs}"
mkdir -p "$OUTDIR"

CATPID=""
cleanup() { [ -n "$CATPID" ] && kill "$CATPID" 2>/dev/null; echo; echo "record.sh: stopped."; exit 0; }
trap cleanup INT TERM

find_port() {
  # Prefer the stable by-id symlink (survives a rename across reconnects), then ttyACM/USB.
  local p
  for p in /dev/serial/by-id/*[Tt]eensy* /dev/serial/by-id/usb-*; do
    [ -e "$p" ] && { echo "$p"; return 0; }
  done
  for p in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$p" ] && { echo "$p"; return 0; }
  done
  return 1
}

echo "record.sh: baud $BAUD, output $OUTDIR/ . Waiting for the Teensy... (Ctrl-C to stop)"
while true; do
  PORT="$(find_port)" || { sleep 1; continue; }
  TS="$(date +%Y%m%d_%H%M%S)"
  F="$OUTDIR/session_${TS}.log"
  stty -F "$PORT" raw -echo "$BAUD" 2>/dev/null || { sleep 1; continue; }
  echo ">> $(date '+%F %T')  recording $PORT  ->  $F"

  # Start logging in the background, then request a fresh header (captured by the running cat).
  cat "$PORT" >> "$F" &
  CATPID=$!
  sleep 0.3
  printf 'h\n' > "$PORT" 2>/dev/null || true

  wait "$CATPID"           # blocks until the port disappears (reset / unplug)
  CATPID=""
  echo "<< $(date '+%F %T')  port lost; reconnecting..."
  sleep 1
done
