# Validation / 验证范围

## Reproducible CPU checks

Run the [quick start](../README.md#quick-start), then `bash scripts/test_core.sh`. The focused geometry suite includes 21 tests across:

- Physical fit at 0.506 m width, rejection at 0.500 m, off-grid positions, and a 17.3° corridor.
- Actual obstacles, missing support, low headroom, and invalidation after geometry or body-size changes.
- Pose-graph connectivity through narrow corridors, including swept-footprint checks.
- Feasible-angle color behavior, comfort margins, and historical validity.
- Local-window changes and overlapping floors connected by stairs.

Bag-management and kinematics tests run separately within the same script. Archived test modules and manual ROS smoke programs remain available for development; the check script names the maintained regression scope explicitly. The CPU example uses the production classifier rather than a substitute implementation.

The September 30 publication checkout passed all **68 checks** locally (21 geometry, 44 bag-management, 3 kinematics), plus the CPU example. Exact Python/package versions and scope are recorded in [`validation/publication-checks.json`](validation/publication-checks.json). This release check did not rerun the field bag or operate robot services.

## Recorded field validation

The published aggregates come from the September 29 run `20260929_190422_lidar_3bad8`, using bag `20260929_155717_4a77a9`, on Jetson Orin NX 16 GB. The input window was 326.628 seconds at 1× replay speed. Raw bags, trajectories, and full scene meshes are not distributed.

[`validation/field-summary.json`](validation/field-summary.json) retains timing, ingestion, scope flags, configuration, doorway snapshot results, and hashes of the relevant deployed source files. It omits machine-local paths and raw trajectories. Source-file digests identify the implementation used in that run; they are not a hash of this publication's documentation.

| Measure | Value | Interpretation |
| --- | ---: | --- |
| Distinct geometry | 12.421 Hz | Newly observed mesh geometry, not checker throughput |
| Distinct result sequence | 1.926 Hz | Actual new traversability results |
| Oldest input → result P50 | 939.60 ms | Includes waiting and computation |
| Oldest input → result P95 | 1,425.42 ms | Latency tail for this replay |
| Checker callback P50 / P95 | 454.96 / 694.41 ms | Computation callback, a different interval |
| Raw / integrated LiDAR frames | 3,266 / 3,264 | Two startup drops, no additional mid-run losses in this run |
| Complete-input result frequency | 0 Hz | Inputs arrived faster than the checker completed; updates were coalesced |

`run_completed` was true. Computation, input, and export errors were empty at the end, with no pending slabs. **`performance_target_met`, `connectivity_verified`, and `stair_connectivity_verified` were false.** Completion of the replay must not be read as a successful real-time or whole-route acceptance result.

Separate checks on frozen mesh snapshots at 75, 85, and 90 seconds found connected doorway pose graphs, including a rescanned scene. These checks do not establish connectivity at every time or through all stairs. A default GUI replay probe observed 352 orange cells with 218 distinct colors; it verifies the actual marker stream rather than just a color helper function.

The full demo was recorded the following day and is provided for visual inspection. It is not the performance measurement source.

## 中文说明

构造测试可以在不连接机器人的情况下复现；实地回放结果来自已有验证记录。本仓库公开汇总指标和相关源码摘要，不分发原始 bag、轨迹或现场 Mesh。

整包回放完成、传感器没有中途新增丢帧，并不意味着实时性或全路线连通性已经通过。当前精细判定的通行性结果约 1.93 Hz，最旧输入至结果的 P95 约 1.43 秒；性能目标尚未达到。门口三个固定快照均连通，但整栋建筑及全部楼梯的连续路线仍未完成验证。

评价算法修改时，应在相同输入、参数、时间窗口和有效性范围下比较。历史区域的蓝色显示、Mesh 的发布频率或局部绿色数量，都不能单独代替有效通路和端到端延迟指标。
