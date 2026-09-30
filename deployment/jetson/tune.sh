#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml
export ROS_DOMAIN_ID=0
exec /usr/bin/python3 /home/nvidia/scanplanner_test/se2_terrain_check/terrain_tuning_gui.py
