# Experimental TSDF weighting

This directory contains an optional LiDAR weighting adapter and a reference copy of the nvblox weighting implementation. It is not enabled in the default replay profile.

In the tested installation, the ROS weighting parameter configures camera TSDF/color but does not set the LiDAR integrator. The benchmark therefore distinguishes `--weighting` (camera) from `--lidar-weighting`. `lidar_weight.cpp` sets the LiDAR integrator explicitly and logs its effective getter value before delegating to the installed callback.

Build with `bash build.sh` on the configured Jetson. The experiment uses an isolated mapper process and checks the installed library digest. It cannot be combined with the dual-origin callback replacement. The reference header and adapted code retain NVIDIA Apache-2.0 notices.

## 中文

不要根据一个 ROS 参数推断两路积分器都已改变。当前安装版本的相机权重与雷达权重需要分别核对；实验输出实际生效的枚举值。惩罚权重没有被证明能改善本项目的路线连通性，未作为默认配置。相关历史 bag 与运行日志不随源码发布。
