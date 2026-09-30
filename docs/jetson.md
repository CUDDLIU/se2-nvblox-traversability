# Jetson integration / Jetson 集成

This guide documents the existing M20 deployment. The repository includes project source and deployment snapshots; it is not a complete robot image or a one-command installer for an unconfigured Jetson.

本说明描述现有 M20 集成。发布操作没有修改机器人上的运行配置。仓库提供项目源码和部署入口快照，传感器、里程计及系统服务仍需要独立配置。

## Tested system

| Component | Deployment |
| --- | --- |
| Compute | Jetson Orin NX, 16 GB |
| ROS | ROS 2 Humble |
| Mapping | Isaac ROS nvblox 3.2.x installation (tested 3.2.5) |
| LiDAR | Front and rear RoboSense AIRY, merged into the local ingress topic |
| Depth | Intel RealSense D435i |
| Pose source | SuperLIO online mapping; recorded poses for replay |
| Visualization | RViz 2 and a Tkinter tuning interface |

The CPU example uses Python 3.10+ independently of ROS. For Humble nodes, use the Python version matching the ROS installation, normally Python 3.10 on Ubuntu 22.04. Do not substitute a Python 3.12 virtual environment for Humble's binary `rclpy` packages.

## External prerequisites

- A working ROS 2 Humble / Isaac ROS nvblox installation with headers, CUDA, and `nvblox_msgs` available.
- Camera and LiDAR drivers, the deployment's local point-cloud bridge, calibrated sensor TF, and the SuperLIO workspace.
- The base robot TF/model service and the user services referenced by the startup scripts.
- `rosbag2`, `rviz2`, `robot_state_publisher`, Tkinter, NumPy, SciPy, Shapely 2, and PyYAML.
- The deployment-specific Fast DDS network profile and graphical session permissions.

SuperLIO, vendor drivers, the point-cloud bridge, and the private SDK/documentation are not bundled here. The existing launch files reference their installed locations. Review these references when porting the project.

## Existing paths and startup

| Existing path | Role | Repository source |
| --- | --- | --- |
| `/home/nvidia/scanplanner_test/se2_nvblox_surface_shadow` | Project, bags, maps, and replay output | Repository root |
| `/home/nvidia/scanplanner_test/se2_terrain_check` | Existing live checker and GUI integration | `deployment/jetson/` plus `bag_tools/gui_source/` |
| `/home/nvidia/scanplanner_test/se2-terrain-venv` | ROS-compatible checker environment | Environment created on deployment |
| `/home/nvidia/scanplanner_test/install-se2-nvblox` | Installed ROS package overlay | Built by the existing workspace |

The `deployment/jetson` files are snapshots for review and porting, not an instruction to overwrite a working installation. The GUI uses `SE2_BAG_TOOLS` when supplied; otherwise it expects the existing project path. Other service, helper-script, and workspace paths also need adaptation on a different robot.

在已经配置好的 Jetson 上运行：

```bash
bash /home/nvidia/scanplanner_test/se2_terrain_check/start_after_boot.sh
```

The command opens the tuner, prepares input diagnostics and network settings, starts the dedicated odometry/reconstruction chain, and opens RViz. It may request the local sudo password for existing robot/network services. It does not send motion commands.

新增的 bag、地图、回放结果和日志全部放在项目目录内；无需在项目同级新建数据目录。旧的 `se2_terrain_check` 是现有部署入口，并非本次发布新增目录。

## Build project-owned components

Run on the configured Jetson, from the project root, using the ROS-compatible environment:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/scanplanner_test/se2-terrain-venv/bin/activate

bash terrain_variants/local_window/build.sh
cmake -S terrain_variants/deskew -B terrain_variants/deskew/build
cmake --build terrain_variants/deskew/build -j2
ctest --test-dir terrain_variants/deskew/build --output-on-failure
bash terrain_variants/fast_lidar/build.sh
```

The CUDA converter targets Orin (`sm_87`) and links the installed nvblox libraries. Its build also runs a converter test and records the installed `libnvblox_ros_lib.so` digest. Replay refuses to load the adapter if this digest changes; rebuild against the new installation. Do not reuse an x86 or different-nvblox binary.

The root `package.xml` / `setup.py` describe the ROS adapter package for an existing colcon workspace. They do not install the entire bag-management deployment or its external services. Keep the terrain implementation and bag tools available as a source checkout.

## Replay and live switching

1. Open **Bag 录制 / 回放 / 管理** and select a saved bag.
2. Enable **重新建图并调参** and choose LiDAR, depth, or fusion. Start with the default 1× LiDAR profile for comparisons.
3. Click **在 RViz 回放**. Replay stops the live RViz, pauses the live reconstruction/checker, and runs in domain 73.
4. Pause input before comparing a parameter change at the same scene. Save comparison snapshots with the bag time and configuration.
5. Click **恢复实时重建** to close replay and restart live reconstruction/RViz. Live depth mapping starts from current observations; the previous GPU map is not restored.

回放重算 TSDF、Mesh 和通行性，但使用已录位姿，不重新运行 SuperLIO。融合模式将两路观测积分到同一 TSDF，目前没有实现经噪声标定的最优传感器融合。仅回放历史输出的模式不会重新建图。

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Replay does not start | `bag_tools/runtime/`, the selected bag's replay logs, deskew executable, CUDA library and ABI manifest |
| No upper-floor mesh | Mesh publication height clipping; reconstruction config disables the old fixed world-height exclusion |
| LiDAR stalls while recording | Input diagnostics and IP fragment counters; check the deployment network before attributing it to the sensor |
| Robot links have no transform | Recorded joint states, `/tf_static`, timed replay transforms, and the namespaced model publisher |
| Blue region after movement | Historical validity; inspect the current local window and status before interpreting old cells |
| Mesh updates quickly but cells lag | Check `distinct_result_sequence_hz` and computation latency; see the documented CPU bottleneck |

Manual `smoke_*.py` checks may start/stop services or create recordings. They are intended for controlled integration testing and are excluded from the CPU test script.
