#!/usr/bin/env bash
# ./upload.bash      build + flash (stop any recorder first)
# ./upload.bash -b   build only
set -eu
cd "$(dirname "$0")"
PIO=$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")
[ "${1:-}" = -b ] && exec "$PIO" run
ENV=teensy41
[ -z "${DISPLAY:-}" ] && ENV=teensy41_cli  # headless: command-line loader
exec "$PIO" run -e "$ENV" -t upload
