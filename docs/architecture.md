# Architecture / 算法与接口

[English overview](../README.md) · [中文概览](../README.zh-CN.md)

## Geometry pipeline

1. **Reconstruct.** nvblox integrates depth or LiDAR observations into a TSDF and publishes incremental mesh blocks. The replay adapter aligns recorded poses and transforms; LiDAR deskew compensates point motion within a scan.
2. **Index.** The mesh index tracks changed and deleted blocks. A native BVH supports local triangle queries without rebuilding the entire world.
3. **Represent floors.** Each horizontal cell may contain several occupied height intervals and supported surfaces. Local surface connectivity separates overlapping floors even when stairs connect them elsewhere.
4. **Classify.** Check support, slope, step height, headroom, and the rectangular body envelope. Conservative grid-footprint checks provide a fast first pass. Refinement searches feasible positions within the cell and headings supported by observed geometry.
5. **Retain pose evidence.** A feasible state stores its position, heading, and conservatively estimated heading interval. Refined graph transitions check the footprint swept between states; a partially feasible heading bin does not imply that the whole bin is free for rotation.
6. **Publish.** Current cells, validity information, status, and archived display markers are separate outputs. New observations and parameter changes invalidate affected results.

当前默认回放实现位于 `terrain_variants/local_window`。算法保留每个格子内的多层表面，结合实际机身轮廓搜索可行位置与朝向；并非简单二维膨胀，也不会额外添加“门口补线”显示层。真实障碍、缺少支撑或顶部净空不足仍可使连接失效。

## Feasible heading angle and color

Each cell carries the sum of its feasible heading-interval widths, capped at `2π`. Color interpolation uses the square root of that fraction to make narrow intervals visible. A symmetric rectangular footprint counts both forward and reverse headings. An exact heading can form a zero-width feasible interval, so zero angular measure is not necessarily an empty feasible set.

Green requires the comfortable mask to cover every heading. Orange covers restricted headings or physical fit without the configured lateral comfort margin. Top clearance, support, slope, and step constraints remain active. Historical validity is handled separately from the color of a freshly computed result.

深橙对应较窄的可行朝向区间，浅橙对应更大的区间；不是简单统计有几个 yaw bit。机身刚好能放下时，即使侧向余量不足，也可能通过原分类结果保留连续橙格。绿色仍要求完整余量，不需要把余量手动改为零来表达受限通路。

## Configuration profiles

The shipped **live** configuration and **replay** preset intentionally differ. `Config()` alone uses the earlier conservative defaults; use an explicit configuration for comparisons.

| Parameter | Existing live snapshot | Default stair replay |
| --- | ---: | ---: |
| Body length × width × height | 0.82 × 0.506 × 0.90 m | Inherited from bag / explicit settings |
| Side / top margin | 0.08 / 0.10 m | Inherited from bag / explicit settings |
| Maximum adjacent step | 0.03 m | 0.20 m |
| Maximum slope | 15° | 40° |
| Horizontal / vertical resolution | 0.10 / 0.01 m | 0.10 / 0.01 m |
| Tile size | 16 cells | 8 cells |
| Heading bins | 40 | 40 |
| Surface normal / stair riser filters | Off | On |

The replay selection lives in [`replay_profiles.py`](../bag_tools/replay_profiles.py); explicit user values override the preset. The default LiDAR reconstruction uses 7 cm TSDF voxels, a four-voxel truncation band, mesh minimum weight 0.5, a 4 m integration range, deskewing, and body filtering. Optional dual-origin and weighting experiments are not enabled by this preset.

实时启动保留原有的平地判据；楼梯重建预设使用 20 cm / 40°。这些是分析阈值，不代表机器人已经启用相应步态或具备对应运动能力。

## Local validity

The replay implementation selects a robot-centered horizontal window of approximately ±1.6 m, snapped to tile boundaries. Candidate surfaces are selected around the current body height (approximately −1.2 m to +0.6 m), with a larger geometry query halo for footprint and headroom checks. The exact selected bounds are reported in status metrics.

Outside this window, cells are retained for historical display. They are not a continuously validated global navigation map. Support checking is finite-resolution and does not establish safety over unobserved space or arbitrarily small holes.

## ROS interfaces

| Topic | Use |
| --- | --- |
| `/nvblox_node/mesh` | Incremental reconstructed geometry |
| `/nvblox_lio/odom`, `/nvblox_lio/odom_lidar` | Recorded/live pose inputs |
| `/LIDAR/POINTS_NX` | Local merged LiDAR input in the M20 deployment |
| `/camera/d435i/depth/image_rect_raw` | D435i depth input; actual filtering follows the camera configuration |
| `/se2_navmesh/local_result` | Serialized local results, including heading masks and angular measure |
| `/se2_navmesh/local_display` | Current cell visualization |
| `/se2_navmesh/local_validity_display` | Result validity visualization |
| `/se2_navmesh/completed_markers` | Archived region display |
| `/se2_terrain/status` | Configuration, timing, validity, and errors |
| `/nvblox_robot/joint_states` | Measured joint model for this integration |

Schema 2 cell rows use `[ix, iy, iz, physical_mask, comfortable_mask, feasible_angle_rad]`. Geometry consumers should also check validity and status; a marker's presence alone is not sufficient. The normal reconstruction frame is `nvblox_odom`.

Live ROS domain: `0`; GUI replay: `73`; manual benchmark: `74`. The benchmark command is an integration tool, with its own earlier defaults. It does not implicitly select the current GUI profile.

## Source map

- `paper_pipeline.py`, `paper_bvh.py`, `native_bvh.cpp`: incremental geometry and height spans.
- `native_nav.cpp`, `pose_clearance.hpp`: classification and physical-clearance refinement.
- `pose_graph.py`: graph states and refined transitions.
- `angular_display.py`, `local_display.py`, `local_updates.py`: angular color, markers, and incremental publication.
- `realtime_node.py`: ROS orchestration and configuration updates.
- `bag_tools/rebuild_node.py`, `rebuild_config.py`: replay orchestration and generated configuration.

The historical `paper_*` filenames are implementation names, not a claim that this repository accompanies a published paper.
