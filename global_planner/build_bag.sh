#!/usr/bin/env bash
# Existing Jetson ROS/nvblox deployment; all artifacts belong to this bag.
set -eo pipefail
planner_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
planner_root=$(dirname -- "$planner_dir")
source /opt/ros/humble/setup.bash
set -u
planner_python=${SE2_GLOBAL_PYTHON:-$(dirname -- "$planner_root")/se2-terrain-venv/bin/python}
planner_bag=${1:?Usage: build_bag.sh /absolute/path/to/bag_bundle}
planner_bag=$(realpath -- "$planner_bag")
case "$planner_bag" in
  "$planner_root"/bags/*) ;;
  *) echo 'Bag 必须位于当前项目的 bags 目录。' >&2; exit 1 ;;
esac
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=3 OMP_WAIT_POLICY=PASSIVE
planner_output="$planner_bag/global_plans/map_$(date +%Y%m%d_%H%M%S_%N)"
bash "$planner_dir/build_native.sh"
if [[ ! -f "$planner_root/terrain_variants/local_window/raster.so" ]]; then
  bash "$planner_root/terrain_variants/local_window/build.sh"
fi
"$planner_python" "$planner_dir/reconstruct.py" "$planner_bag" --output "$planner_output"
"$planner_python" "$planner_dir/build.py" "$planner_output/mesh.npz"
echo "全局通行图已保存：$planner_output"
