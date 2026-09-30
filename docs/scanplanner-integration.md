# ScanPlanner integration contract

The target local planner is the existing **SCAN-Planner** on the Jetson. The global planner currently publishes a complete `nav_msgs/Path` on `/se2_global/path`, retaining floor XYZ and body yaw at every state. This is a visualization and geometric planning output. **Cross-floor execution compatibility with the installed ScanPlanner has not yet been established.**

## Verified interface differences

Inspected on September 30, 2026: `SCAN-Planner/src/planner/plan_manage/src/scan_replan_fsm.cpp`, SHA-256 `7984cdaac6beda6a933d4717f5c7a7131b8833257b67e839fb49a02f3edf71f8`.

| Installed behavior | Required for this global route |
| --- | --- |
| Modes 3/4 subscribe to `initial_path`, reliable / transient local | Compatible ROS message and QoS are already used by `/se2_global/path` |
| `pathCallback()` replaces every waypoint Z and the goal Z with the current odometry Z | Preserve the route's floor separation and convert floor-reference height to the local planner's body-reference height using the calibrated frame/pose convention |
| Reference waypoints are sampled at a fixed 1.5 m distance | Preserve stair boundaries, turns, and constrained passage states; any simplification needs geometric validation |
| Only the final pose orientation reaches `storeGoalYaw()` | Preserve intermediate heading requirements, especially in orange narrow passages; validate the optimized local trajectory against the global route's permitted geometry and headings |
| Goal completion and several tracking decisions use XY distance | Require floor/height consistency and route progress, so the same XY on another level is not treated as arrival |
| Mode 4 has additional code that assigns local target Z from the current start | Audit the full local-target and optimization pipeline; changing only `pathCallback()` is insufficient |
| `pathCallback()` ignores an empty path | An empty global visualization path does not cancel ScanPlanner execution; use its explicit cancellation lifecycle |
| Existing Smac bridge flattens path Z to zero | Use a dedicated 3D bridge instead of routing this output through that planar bridge |

The existing reference-path input does not itself transform the incoming frame. A future bridge must transform positions **and yaw** into the frame used by `body_pose` and the local map, using an actual valid transform. Renaming `nvblox_odom` to `map` or `odom` is not a transform. A bag reconstruction origin is also not automatically aligned with the current live odometry origin.

## Handoff responsibilities

The intended chain is **SE(2) NavMesh global planning → 3D reference/heading adapter → ScanPlanner local optimization → existing execution layer**.

- Keep global paths on `/se2_global/path` while validating integration. The global viewer has no publisher to the deployed `/initial_path`, execution topics, or robot command topics.
- Preserve the full ordered XYZ/yaw route and the build-time body dimensions. Floor points are not automatically body-center odometry points; that conversion must use the robot's measured reference-frame convention.
- Use ScanPlanner's `/scan/global_replan_request` for a new plan from fresh, correctly aligned odometry. Match results to the current goal/request so old results cannot replace a newer task.
- Wire explicit goal replacement and `/scan/navigation_cancel` semantics. Publishing an empty `Path` currently clears RViz only.
- Retain the multiheight/yaw constraints through local smoothing and tracking. The global path's checked status does not certify a modified local trajectory.

Acceptance cases include a staircase returning to the start's XY on a different floor; a body-width corridor that requires a non-bin heading; an in-place heading change; a turn that a 1.5 m chord would cut; an unavailable frame transform; and cancellation or replan failure with an old route already active. Integration should first run with bag/simulation inputs and the execution layer disconnected.

## 中文说明

最终链路确定为：**SE(2) NavMesh 全局规划 → 三维路径与朝向适配 → 本地 ScanPlanner → 现有执行层**。当前已经输出标准 `nav_msgs/Path`，每个状态保留地面三维坐标与机身朝向。

已核查 Jetson 上的真实代码：ScanPlanner 的模式 3/4 接收 `/initial_path`，但会把整条路线压到当前里程计高度，按 1.5 米抽稀，仅使用终点朝向；部分到达判断只看 XY，模式 4 的局部目标处也存在高度覆盖。旧 Smac 桥接还会将 Z 归零。因此，**仅重映射话题无法完成跨楼层适配**。

后续接入需要保留楼层、楼梯拐点、受限朝向及路径进度，明确“地面路径点”到 ScanPlanner 机身参考点的转换，并对局部优化后的轨迹继续检查。坐标系必须通过有效 TF 对齐，不能只改 `frame_id`；bag 原点也不能直接当成当前实时定位原点。停止／换目标需要使用 ScanPlanner 的明确取消流程，因为当前它会忽略空路径。

本次实现保持完整的三维与朝向输出，并完成上述接口审查；现有 ScanPlanner 代码、配置和运动执行链路未修改。实机跨楼层闭环执行尚未验证。
