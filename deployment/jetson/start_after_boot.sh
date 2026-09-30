#!/usr/bin/env bash
# Run as nvidia on Jetson. Does not enable navigation or change boot services.
set -eo pipefail
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
export ROS_DOMAIN_ID=0
mode=${1:-normal}
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/bag_tools/start_diagnostics.sh
open_live_rviz() {
  # A replay may start while this script is waiting for the live sensors.
  if [[ -e /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/bag_tools/runtime/live_rviz_suspended ]]; then
    return
  fi
  if ! systemctl --user is-active --quiet se2-terrain-rviz; then
    systemd-run --user --unit=se2-terrain-rviz --collect \
      --setenv=DISPLAY=:0 --setenv=XAUTHORITY=/run/user/1000/gdm/Xauthority \
      --setenv=ROS_DOMAIN_ID=0 --setenv=ROS_LOCALHOST_ONLY=0 \
      --setenv=FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml \
      /bin/bash -c 'source /opt/ros/humble/setup.bash; exec rviz2 -d /home/nvidia/scanplanner_test/se2_terrain_check/paper.rviz'
  fi
}
if [[ "$mode" == --rviz-only ]]; then
  open_live_rviz
  exit 0
fi
# Keep saved bags accessible even when live sensors cannot start.
if [[ "$mode" == normal ]] && ! systemctl --user is-active --quiet se2-terrain-tuner; then
  systemd-run --user --unit=se2-terrain-tuner --collect \
    --property=KillMode=mixed --property=TimeoutStopSec=45s \
    --setenv=DISPLAY=:0 --setenv=XAUTHORITY=/run/user/1000/gdm/Xauthority \
    --setenv=FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml \
    /bin/bash /home/nvidia/scanplanner_test/se2_terrain_check/tune.sh
fi
if [[ "$mode" == --prepare-record ]]; then
  # A new odometry origin requires fresh depth mapping, checker history and RViz.
  for unit in se2-terrain-rviz se2-terrain-check nvblox-d435i-shadow nvblox-lio; do
    systemctl --user stop "$unit" 2>/dev/null || true
  done
fi
if [[ "$mode" == normal &&
      -e /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/bag_tools/runtime/live_mapping_suspended.json &&
      -e /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/bag_tools/runtime/live_rviz_suspended ]]; then
  echo '实时深度建图已为回放暂停，请在调参界面点击“恢复实时重建”。'
  exit 0
fi
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/bag_tools/tune_network.sh "$mode"
if [[ "$mode" == --restore-live ]] &&
   systemctl --user is-active --quiet nvblox-lio &&
   systemctl --user is-active --quiet nvblox-d435i-shadow &&
   systemctl --user is-active --quiet se2-terrain-check; then
  open_live_rviz
  exit 0
fi
# Boot-time TF services can have started before eno1 acquired an address.
# An active process is insufficient after DDS reported no whitelist interface.
ip -4 -o addr show dev eno1 scope global | grep -q 'inet '
systemctl --user restart d435i-extrinsic-candidate d435i-depth-factory-tf
if [[ "$mode" != normal ]]; then
  # GUI actions have no terminal for a sudo password prompt.
  if ! systemctl is-active --quiet m20-robot-model.service; then
    sudo -n systemctl start m20-robot-model.service
  fi
else
  sudo systemctl restart m20-robot-model.service
fi
if ! systemctl --user is-active --quiet nvblox-lio; then
  systemd-run --user --unit=nvblox-lio --collect \
    /bin/bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/bag_tools/run_lio_online.sh
fi
timeout 30 ros2 topic echo /nvblox_lio/odom_lidar nav_msgs/msg/Odometry --once --field header
if ! systemctl --user is-active --quiet nvblox-d435i-shadow; then
  systemd-run --user --unit=nvblox-d435i-shadow --collect \
    /home/nvidia/scanplanner_test/run_nvblox_d435i.sh \
    pose_mode:=odom odom_frame:=nvblox_odom odom_topic:=/nvblox_lio/odom \
    use_rviz:=false start_localization:=false use_legacy_surface_adapter:=false
fi
if ! systemctl --user is-active --quiet se2-terrain-check; then
  systemd-run --user --unit=se2-terrain-check --collect \
    /bin/bash /home/nvidia/scanplanner_test/se2_terrain_check/run.sh
fi
open_live_rviz
