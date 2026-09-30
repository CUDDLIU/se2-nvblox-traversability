#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
incs=(-I/opt/ros/humble/include -I/usr/include/eigen3 -I/usr/local/cuda/include -I/usr/include/opencv4)
for package in /opt/ros/humble/include/*; do
  if [[ -d "$package" ]]; then incs+=(-idirafter "$package"); fi
done
for package in /opt/ros/humble/share/*/gxf/include; do
  if [[ -d "$package" ]]; then incs+=(-I "$package"); fi
done
g++ -std=c++17 -O2 test_split.cpp "${incs[@]}" -o test_split
./test_split
g++ -std=c++17 -O3 -fPIC -shared process_lidar.cpp "${incs[@]}" \
  -L/opt/ros/humble/lib -Wl,-rpath,/opt/ros/humble/lib \
  -lnvblox_ros_lib -lnvblox_lib -lrclcpp -lglog -lgflags -o libmulti_lidar.so
sha256sum /opt/ros/humble/lib/libnvblox_ros_lib.so > installed_library.sha256
