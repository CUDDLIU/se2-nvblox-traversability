# M20 joint model / M20 实测关节模型

This adapter consumes M20 motion-state telemetry and publishes a robot model dedicated to the nvblox workflow. It does not replace unrelated robot TF publishers or issue movement commands.

| Interface | Purpose |
| --- | --- |
| `/nvblox_robot/joint_states` | Converted measured joint state |
| `/nvblox_robot/robot_description` | Namespaced URDF, transient-local |
| `nvblox_robot/*` child frames | Dedicated leg/wheel transforms rooted at `base_link_dog` |
| `/nvblox_robot/motion_status/{raw,joints,wheels}` | Recorded input telemetry |

The receiver sends only the telemetry heartbeat (`Type=100, Command=100`). Body pose and height from the status message are retained as telemetry; they do not replace the world pose supplied by odometry. Wheel angle is a relative visualization integral from startup, not an absolute encoder measurement. Gaps over 300 ms are not extrapolated.

关节采用 `q = direction × protocol_value + offset`，并处理等价圈数；实现和测试见 `kinematics.py`、`test_kinematics.py`。专用命名空间避免与原有腿部 TF 冲突。旧 bag 缺少实测关节时，不会伪造实测值。

The original URDF and STL files are retained in `upstream/` under their [BSD-3-Clause license](upstream/LICENSE.txt). Source: [DeepRoboticsLab/deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model), commit `75824b516ecc8fa3f8f7ce5d6577e1e86d6b614f`. File hashes are in [source_manifest.json](source_manifest.json). Runtime adaptation renames frames, resolves mesh paths, and removes simulation-only tags; it does not edit the vendored originals.

Private SDK headers and user-provided protocol guides are not redistributed. The adapter is deployment-specific and requires verification of units, conventions, and telemetry support on another robot. The model remains a visualization input, not a full gait/contact calibration.

Run `bash scripts/test_core.sh` from the root for the pure kinematics tests. The manual `smoke_bag.py` program requires the configured ROS environment.
