#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
export ROS_DOMAIN_ID=0
# Tiny per-cell least-squares systems must not launch a BLAS thread pool.
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=3
export OMP_WAIT_POLICY=PASSIVE
# Same running nvblox process means the same odometry/map session. Never load
# a previous boot's history into a new odometry origin.
se2_map_invocation=$(systemctl --user show nvblox-d435i-shadow -p InvocationID --value)
export SE2_MAP_SESSION="$(cat /proc/sys/kernel/random/boot_id):${se2_map_invocation}"
exec /home/nvidia/scanplanner_test/se2-terrain-venv/bin/python \
  /home/nvidia/scanplanner_test/se2_terrain_check/realtime_node.py \
  --ros-args --params-file /home/nvidia/scanplanner_test/se2_terrain_check/paper_config.yaml
