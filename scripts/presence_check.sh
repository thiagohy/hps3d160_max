#!/usr/bin/env bash
# Guided presence / vital signs check.
#
#   ./scripts/presence_check.sh <port> [seconds]
#
# Step 1 (A): leave the sensor's field of view, then run this script.
# Step 2 (B): stand in front of the sensor (0.5 - 2 m, still) and run it again.
# The two captures are compared automatically.
set -euo pipefail
cd "$(dirname "$0")/.."

PORT="${1:-/dev/ttyACM0}"
SECONDS_PER_RUN="${2:-30}"
NODE=./build/hps3d160_max_node
OUT=${OUT_DIR:-/tmp/hps_presence_check}

if [[ ! -x "$NODE" ]]; then
    echo "build first: ./scripts/build.sh" >&2
    exit 1
fi

read -r -p "Label this capture (A = empty scene, B = with person): " label
dir="$OUT/$label"
rm -rf "$dir"
mkdir -p "$dir"

echo "capturing ${SECONDS_PER_RUN}s into $dir ..."
sudo chmod 777 "$PORT" 2>/dev/null || true
"$NODE" --port "$PORT" --presence --max-frames 0 --record-dir "$dir" &
NODE_PID=$!
sleep "$SECONDS_PER_RUN"
kill -INT "$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true

echo
echo "--- presence report ---"
grep -h "\[presence\]" "$dir"/*.log 2>/dev/null | tail -5 || true

echo
echo "--- comparison with the other capture ---"
other=""
for d in "$OUT"/*/; do
    [[ "$d" == "$dir/" ]] && continue
    other="$d"
done
if [[ -n "$other" ]]; then
    python3 scripts/compare_presence.py "$other" "$dir"
else
    echo "only one capture so far - run it again with the person in front of the sensor"
fi
