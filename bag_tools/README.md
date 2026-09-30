# Recording and replay / 录制与回放

This module extends the existing tuning interface with sensor recording, reconstruction from bags, parameter snapshots, and bag/map lifecycle management. See [Jetson integration](../docs/jetson.md) for the required ROS stack and deployment services. The GUI is currently in Chinese.

## Workflow / 操作流程

1. Start the configured deployment with `bash /home/nvidia/scanplanner_test/se2_terrain_check/start_after_boot.sh`.
2. Open **Bag 录制 / 回放 / 管理**. With **新建地图后录制** selected, recording restarts the dedicated SuperLIO mapping session with a fresh origin. Mesh reconstruction, terrain computation, and live RViz are suspended to save resources; sensor inputs, poses, TF, and measured joint state continue to be recorded.
3. Click **停止并保存**. The bag, accumulated map, trajectory, telemetry, and parameter snapshots remain one bundle. Live reconstruction is restored after recording.
4. Select the bag, enable **重新建图并调参**, choose LiDAR / depth / fusion, and click **在 RViz 回放**. The default profile is LiDAR with deskewing and the current `local_window` classifier.
5. Pause playback to compare parameters at a fixed scene. **保存对比快照** records parameters, bag time, result counts, and computation state. **保存本次回放参数** changes only that replay's configuration.
6. Click **恢复实时重建** to close replay and resume live reconstruction/RViz. **关闭回放** only closes playback, allowing live restoration later.

回放从空 TSDF 开始，使用包内传感器、位姿与 TF 重新建图，不重新估计 SuperLIO 位姿。录制开始之前的观测不会自动恢复。旧包若含录制时的 Mesh，可以打开对照层；新的纯输入录制需要重建才能生成 Mesh。

实时 GPU 重建与回放不会同时启动两套。恢复实时后从当前观测重新建立深度地图，不恢复暂停前的 GPU 地图。LIO 在重建回放期间保持运行，回放与实时使用独立 ROS domain。

## Storage / 存储

All generated files remain under the project root:

```text
bags/
  <session-id>/
    data/                 # rosbag2 SQLite data and metadata
    session.json          # name, state, requested topics, parameters
    maps/                 # accumulated SuperLIO PCD, trajectory, input events
    telemetry/            # recorded motion-status and diagnostics
    replays/              # replay configs, results, comparison snapshots
  .trash/                 # restorable deleted bundles
bag_tools/runtime/        # bounded diagnostics, locks, and process logs
```

Rename, move to trash, restore, and permanently delete operate on the complete bundle. Permanent deletion is available only for trashed recordings and requires confirmation in the GUI. Moving a bag to trash does not free disk space. Interrupted recordings can be reindexed, but unflushed sensor data cannot be recovered.

保存的 PCD 是录制窗口内 SuperLIO 世界坐标点云的体素累积结果，并非回环优化地图；它只作为回放对照层，不作为传感器重建输入。

## Input integrity

- Retain odometry, IMU, TF, camera intrinsics, and model state needed by reconstruction. Unpublished RGB/IR topics do not acquire data merely because they appear in a requested-topic list.
- Record the local `/LIDAR/POINTS_NX` ingress rather than adding redundant readers to the remote large-cloud topics.
- Detect stream gaps, preserve diagnostics with the bag, and stop/finalize when free disk space falls below the configured guard.
- Keep timed transforms and model description available during playback, including after the physical camera disconnects.
- Reject missing LiDAR prerequisites or a changed installed nvblox library before starting the replay adapter.

深度话题虽然名为 `image_rect_raw`，仍可能受当前相机驱动滤波影响。诊断能够记录缺失与断流，不能补回从未录到的数据。

## Tests

Run `bash scripts/test_core.sh` from the repository root for CPU tests. The `test_*.py` modules use temporary storage and mocks for external processes. `smoke_*.py` programs are separate manual checks that operate the real deployment. Historical field datasets used by diagnostic scripts are not included.

## 整包全局规划

选中已构建通行图的 bag 后，点击 **全局路径规划**，在 RViz 使用 **Publish Point** 点击目标楼层地面。起点默认是录制终点，可通过 **在 RViz 设起点** 和 **恢复 bag 终点** 修改。**关闭回放** 关闭规划窗口。静态图采用构建时参数，随 bag 绑定管理；详见[整包建图、论文对应关系与 ROS 接口](../docs/global-planning.md#简体中文)。
