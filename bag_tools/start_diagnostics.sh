#!/usr/bin/env bash
set -eo pipefail
if ! systemctl --user is-active --quiet se2-input-diagnostics; then
  systemd-run --user --unit=se2-input-diagnostics --collect \
    --property=Restart=on-failure --property=RestartSec=3 \
    --setenv=ROS_DOMAIN_ID=0 \
    --setenv=FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/.config/fastdds/eno1.xml \
    /bin/bash -c 'source /opt/ros/humble/setup.bash; exec python3 /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/bag_tools/input_diagnostics.py'
fi
