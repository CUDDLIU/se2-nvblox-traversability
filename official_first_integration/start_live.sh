#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
mkdir -p "$HERE/runtime" "$HERE/../scanplanner_integration/runtime"
# Share the old session's lock: only one integration may own the robot.
exec 9>"$HERE/../scanplanner_integration/runtime/session.lock"
flock -n 9 || { echo 'A navigation session is already running'; exit 1; }
source /opt/ros/humble/setup.bash
export PYTHONUNBUFFERED=1
if [[ ! -f "$HERE/../global_planner/astar.so" || "$HERE/../global_planner/astar.cpp" -nt "$HERE/../global_planner/astar.so" ]]; then
  bash "$HERE/../global_planner/build_native.sh"
fi
exec /home/nvidia/scanplanner_test/se2-terrain-venv/bin/python "$HERE/run.py" "$@"
