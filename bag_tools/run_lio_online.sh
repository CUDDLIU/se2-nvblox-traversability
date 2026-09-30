#!/usr/bin/env bash
# Online SuperLIO: starts with an empty in-memory map; no relocation/map loader.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/nvidia/scanplanner_test/Super-LIO-nx-scanplanner-port/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
export ROS_DOMAIN_ID=0
exec ros2 run super_lio super_lio_node --ros-args \
  --params-file /home/nvidia/scanplanner_test/Super-LIO-nx-scanplanner-port/install/super_lio/share/super_lio/config/lidar_points.yaml \
  -r __node:=se2_superlio_mapping \
  -p lio.map.save_map:=false -p lio.output.robot:=false \
  -p lio.output.map:=true -p lio.output.dense:=false \
  -p lio.ros.map_frame:=nvblox_odom -p lio.ros.odom_frame:=nvblox_odom \
  -r /lio/odom:=/nvblox_lio/odom -r /lio/odom_lidar:=/nvblox_lio/odom_lidar \
  -r /lio/path:=/nvblox_lio/path -r /lio/body_cloud:=/nvblox_lio/body_cloud \
  -r /lio/cloud_world:=/nvblox_lio/cloud_world \
  -r /scan/diagnostics/event:=/nvblox_lio/diagnostics
