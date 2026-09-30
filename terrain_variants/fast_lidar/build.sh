#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
incs=(-I/opt/ros/humble/include -I/usr/include/eigen3)
for package in sensor_msgs std_msgs builtin_interfaces geometry_msgs visualization_msgs rosidl_runtime_cpp rosidl_runtime_c rosidl_typesupport_interface; do
  incs+=("-I/opt/ros/humble/include/$package")
done
flags=(-std=c++17 -O3 -arch=sm_87 -Xcompiler=-fPIC -Wno-deprecated-gpu-targets --expt-relaxed-constexpr -diag-suppress=20012)
libs=(-L/opt/ros/humble/lib -lnvblox_lib -lglog -lgflags -Xlinker=-rpath -Xlinker=/opt/ros/humble/lib)
/usr/local/cuda/bin/nvcc "${flags[@]}" "${incs[@]}" -shared converter.cu "${libs[@]}" -o libfast_lidar.so
g++ -std=c++17 -O3 -x c++ test_converter.cu "${incs[@]}" -I/usr/local/cuda/include \
  -L. -lfast_lidar -L/opt/ros/humble/lib -lnvblox_ros_lib -lnvblox_lib -lglog -lgflags \
  -L/usr/local/cuda/lib64 -lcudart -Wl,-rpath,/opt/ros/humble/lib -o test_converter
LD_LIBRARY_PATH="$PWD:${LD_LIBRARY_PATH:-}" ./test_converter
sha256sum /opt/ros/humble/lib/libnvblox_ros_lib.so > installed_library.sha256
