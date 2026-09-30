#!/usr/bin/env bash
set -eo pipefail
scan_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
scan_ws="$scan_root/scanplanner_ws"
scan_source="$scan_ws/src/SCAN-Planner"
scan_commit=d0b921c9b05a6d291d144d60882b2e0e88d2c0e0
scan_patch="$scan_root/scanplanner_integration/patches/0001-m20-se2-low-latency.patch"
mkdir -p "$scan_ws/src"
if [[ ! -d "$scan_source/.git" ]]; then
  if [[ -e "$scan_source" ]]; then echo "Refusing to overwrite $scan_source" >&2; exit 1; fi
  git clone --depth 1 --branch ros2-community --single-branch https://github.com/wuyi2121/SCAN-Planner.git "$scan_source"
fi
if [[ $(git -C "$scan_source" rev-parse HEAD) != "$scan_commit" ]]; then
  git -C "$scan_source" diff --quiet || { echo 'Upstream checkout has local edits; refusing to replace it.' >&2; exit 1; }
  git -C "$scan_source" fetch --depth 1 origin "$scan_commit"
  git -C "$scan_source" checkout --detach "$scan_commit"
fi
if git -C "$scan_source" apply --reverse --check "$scan_patch" 2>/dev/null; then
  echo 'Integration patch already applied.'
else
  git -C "$scan_source" diff --quiet || { echo 'Unexpected upstream edits; preserve them before installing.' >&2; exit 1; }
  git -C "$scan_source" apply --check "$scan_patch"
  git -C "$scan_source" apply "$scan_patch"
fi
if [[ ! -e "$scan_ws/src/se2_scan_bridge" ]]; then
  ln -s ../../scanplanner_integration "$scan_ws/src/se2_scan_bridge"
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
export MAKEFLAGS=-j${CMAKE_BUILD_PARALLEL_LEVEL}
cd "$scan_ws"
colcon build --executor sequential --packages-up-to scan_planner se2_scan_bridge --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
