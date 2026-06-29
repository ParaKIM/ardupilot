#!/bin/sh

set -eu

AUTOTESTDIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPOROOT=$(CDPATH= cd -- "$AUTOTESTDIR/../.." && pwd)
PYTHON=${PYTHON:-python3}

if ! command -v fgfs >/dev/null 2>&1; then
    echo "fgfs was not found. Install FlightGear on this GUI machine first." >&2
    exit 1
fi

"$AUTOTESTDIR/fg_plane_view.sh" &
FG_PID=$!

cleanup() {
    kill "$FG_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

cd "$REPOROOT"
"$PYTHON" "$AUTOTESTDIR/sim_vehicle.py" \
    -v ArduPlane \
    -f quadplane \
    --enable-fgview \
    --speedup 1 \
    "$@"
