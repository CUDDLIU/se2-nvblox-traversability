#!/usr/bin/env bash
# Owned recording mode; no edits to external launchers, services or calibration.
set -eo pipefail
record_root=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
source /home/nvidia/scanplanner_test/install-se2-nvblox/local_setup.bash
export AMENT_PREFIX_PATH="/home/nvidia/scanplanner_test/install-se2-nvblox:${AMENT_PREFIX_PATH:-}"
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
if [[ "${1:-}" == --camera ]]; then
  exec ros2 launch "$record_root/bag_tools/recording_inputs.launch.py"
fi
for unit in se2-terrain-rviz se2-terrain-check nvblox-d435i-shadow; do
  systemctl --user stop "$unit" 2>/dev/null || true
done
if [[ "${1:-}" == --new-map ]]; then
  systemctl --user stop nvblox-lio 2>/dev/null || true
fi
bash "$record_root/bag_tools/tune_network.sh" --prepare-record
systemctl --user start d435i-extrinsic-candidate d435i-depth-factory-tf
if ! systemctl --user is-active --quiet nvblox-lio; then
  systemd-run --user --unit=nvblox-lio --collect /bin/bash "$record_root/bag_tools/run_lio_online.sh"
fi
timeout 30 ros2 topic echo /nvblox_lio/odom_lidar nav_msgs/msg/Odometry --once --field header
systemd-run --user --unit=nvblox-d435i-shadow --collect /bin/bash "$0" --camera
