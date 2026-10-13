#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
set +u
source /opt/ros/humble/setup.bash
set -u
cmake -S "$HERE" -B "$HERE/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_ROS=ON
cmake --build "$HERE/build" -j2
ctest --test-dir "$HERE/build" --output-on-failure
