#!/usr/bin/env bash
set -eo pipefail
scan_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$scan_root/scanplanner_integration/runtime"
exec 9>"$scan_root/scanplanner_integration/runtime/session.lock"
flock -n 9 || { echo 'ScanPlanner session is already running.' >&2; exit 1; }
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
source "$scan_root/scanplanner_ws/install/setup.bash"
scan_python=${SE2_SCAN_PYTHON:-$(dirname -- "$scan_root")/se2-terrain-venv/bin/python}
exec "$scan_python" "$scan_root/scanplanner_integration/run.py" "$@"
