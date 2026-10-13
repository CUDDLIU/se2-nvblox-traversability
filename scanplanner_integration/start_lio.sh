#!/usr/bin/env bash
# Isolated odometry for this navigation session; no robot commands or map loader.
set -eo pipefail
scan_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
scan_lio_prefix=${SE2_SCAN_LIO_PREFIX:-$(dirname -- "$scan_root")/Super-LIO-nx-scanplanner-port/install}
scan_lio_config="$scan_lio_prefix/super_lio/share/super_lio/config/lidar_points.yaml"
if [[ ! -f "$scan_lio_prefix/setup.bash" || ! -f "$scan_lio_config" ]]; then
  echo "SuperLIO installation missing: $scan_lio_prefix; set SE2_SCAN_LIO_PREFIX." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$scan_lio_prefix/setup.bash"
exec ros2 run super_lio super_lio_node --ros-args \
  --params-file "$scan_lio_config" -r __node:=se2_scan_superlio \
  -p lio.map.save_map:=false -p lio.output.robot:=false \
  -p lio.output.planner:=false -p lio.output.map:=false -p lio.output.dense:=false \
  -p lio.ros.map_frame:=nvblox_odom -p lio.ros.odom_frame:=nvblox_odom \
  -r /lio/odom:=/nvblox_lio/odom -r /lio/odom_lidar:=/nvblox_lio/odom_lidar \
  -r /lio/path:=/nvblox_lio/path -r /lio/body_cloud:=/nvblox_lio/body_cloud \
  -r /lio/cloud_world:=/nvblox_lio/cloud_world \
  -r /scan/diagnostics/event:=/nvblox_lio/diagnostics
