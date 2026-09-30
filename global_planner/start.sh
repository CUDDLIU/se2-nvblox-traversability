#!/usr/bin/env bash
set -eo pipefail
planner_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
planner_root=$(dirname -- "$planner_dir")
mkdir -p -- "$planner_dir/runtime"
exec 9>"$planner_dir/runtime/viewer.lock"
flock -n 9 || { echo '全局规划窗口已打开，请使用原窗口。' >&2; exit 1; }
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=${SE2_GLOBAL_DOMAIN:-79}
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$planner_root/terrain_variants/fastdds_large_shm.xml"
unset FASTDDS_DEFAULT_PROFILES_FILE
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=3 OMP_WAIT_POLICY=PASSIVE
planner_python=${SE2_GLOBAL_PYTHON:-$(dirname -- "$planner_root")/se2-terrain-venv/bin/python}
if [[ ! -f "$planner_dir/astar.so" || "$planner_dir/astar.cpp" -nt "$planner_dir/astar.so" ]]; then
  bash "$planner_dir/build_native.sh"
fi
planner_map=${1:-}
if [[ -z "$planner_map" ]]; then
  planner_map=$("$planner_python" - "$planner_root" <<'PY'
from pathlib import Path
import sys
files=list((Path(sys.argv[1])/'bags').glob('*/global_plans/*/navmesh.json'))
if not files:raise SystemExit('尚无全局通行图，请先运行 global_planner/reconstruct.py 和 build.py。')
print(max(files,key=lambda p:p.stat().st_mtime).parent)
PY
)
fi
mkdir -p -- "$planner_map/runtime"
"$planner_python" "$planner_dir/ros_node.py" "$planner_map" > "$planner_map/runtime/planner.log" 2>&1 &
planner_pid=$!
rviz_pid=
cleanup() {
  if [[ -n "$rviz_pid" ]]; then kill -TERM "$rviz_pid" 2>/dev/null || true; fi
  kill -INT "$planner_pid" 2>/dev/null || true
  wait "$planner_pid" 2>/dev/null || true
  if [[ -n "$rviz_pid" ]]; then wait "$rviz_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
rviz2 -d "$planner_dir/global.rviz" --ros-args -r __node:=se2_global_rviz > "$planner_map/runtime/rviz.log" 2>&1 &
rviz_pid=$!
wait -n "$planner_pid" "$rviz_pid"
