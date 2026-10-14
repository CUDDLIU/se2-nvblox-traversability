# Official NOS local planning integration

**2026-10-14：默认后端改为 `source`（源码复现）。** 运行时只加载本项目编译的 `build/source/libnos_source.so` 与已核对哈希的轨迹数值表，不启动官方规划二进制，不加载 DrDDS SDK。`--local-backend official` 显式选择旧的二进制对照后端，`legacy` 选择更早的自定义版本。构建源码后端用 `bash official_first_integration/source/build.sh`。

**2026-10-14 楼梯中途停止与 ASA 居中修复：** 楼梯高度图改用完整去畸变的机身点云，避免原定位匹配点集的 2 m 盲区和 0.5 m 下采样造成近身踏板缺测。新机器启动前运行 `bash scanplanner_integration/build_stair_lio.sh`，默认从项目同级的 `Super-LIO-nx-scanplanner-port` 构建私有 LIO；也可将上游工作区路径作为第一个参数。补丁仅增加密集机身点云输出，不修改上游工作区或定位匹配参数。ASA 居中余量偏好为 0.45 m、权重 6，并对非绿色区域增加权重 4 的软代价；必要窄通道仍可通过，1.5 m 局部前瞻不变。51 项源码测试、23 项 ASA 测试及 12 级上下楼模拟通过，实机效果仍需复测。详见 [修复与验证记录](evidence/source_20261014/stair_midflight/README.md)。

**ASA 分段导航已接入（2026-10-14）：** 默认 `NavMode=1、ObsMode=0`。ASA 从原始地图的连续踏面生成平地／上楼／下楼段，路径与模式通过会话隔离的 `/se2_global/navigation_route` 一次发布。普通 `Path` 仅用于显示和参考后端，不触发源码执行。楼梯段使用 `Gait=0x3003` 与 AOS AUTO/STAIR/NORMAL 参数，平地使用 `0x3002` 与 AUTO/FLAT/NORMAL 参数；收到参数 ACK 和实际步态确认后执行。新目标、取消和重规划清理旧模式；取消同时终止 ASA 未完成的查询。

识别采用同一份 Mesh，不增加机身、支撑或碰撞验证。保留水平踏面检测（12°、至少两级同向 6 cm～`max_step+2.5 cm` 高差、踏面间距 12～80 cm、提前 0.6 m）；同时支持 TSDF 抹平踏面的持续爬升：从不超过 65° 的面采样实际地面高度，按路径高度选择楼层，0.48 m 窗口坡度至少 0.30、持续至少 0.6 m 且累计高差至少 0.30 m。只填补至多 12 cm 的短缺口，要求至少 80% 原始有效采样，并拒绝过大单跳和明显反向起伏。此分支提前 0.8 m 切换，结束后延续 0.5 m；准备／退出段重叠时中间平台保持楼梯步态。陡坡也使用爬升模式，不能把这个分类器理解为精确区分楼梯与陡坡。缓坡、单个低凸起、仅路径 Z 噪声不切换。参数属于 **ASA 接入算法，不是反汇编得到的官方识别算法**。局部规划不再增加末端命令扫掠复检，保留官方候选路径、旋转及直线避障检查。普通选点默认前向 NORMAL 速度，官方表对应平地 maxX=1、楼梯 maxX=1 m/s，maxY/maxYaw 均为 1；这是配置上限，不是恒定执行速度。


2026-10-14 时延与定位修复：NX 点云保留原有 ±4 m 裁剪、1 cm 体素、10 cm 半径四邻点判定及 5 cm 障碍计数规则；合并等价裁剪，使用空间哈希提前终止邻点计数，保留 PCL 参考入口用于逐点及规划输出对比。ROS 点云 payload 改用 `array('B')`，避免 Python 逐字节验证。每个导航会话从原网络配置生成独立 DDS 配置，保留网卡白名单和 UDP 缓冲，将共享内存区设为至少 32 MiB 以容纳完整 NX 点云；不改写系统原配置。状态新增 `flat_filter` 分阶段耗时、`planning_ms/cycle_ms`、原始点云接收/拒收计数，便于区分计算迟延和传输丢帧。`source/benchmark_cloud.py` 对实测 NPZ 做数值/速度结果一致性检查；`source/acceptance_latency.py` 在隔离 ROS 域回放完整点云，不连接执行器。

定位继续使用原始扫描和实测位姿去畸变。体素去重改用稳定字典序排序，代表点和顺序与原实现一致；稀疏配准点云输入实验未纳入运行默认。匹配器、初始化稳定检查及导航接入都改为在机器人当前位置衡量地图校正量，消除远离里程计原点时小角度校正引起的平移放大。15 cm/0.06 rad 匹配限制及质量阈值保持不变。连续失配超过 1.5 秒后使旧路线失效，并在里程计传播的最后地图位置附近重新执行严格初始化、候选歧义检查和五次稳定确认；先前全姿态只作为候选，不作为绕过匹配的可用定位。定位状态新增 JSONL 历史，记录实际校正量与匹配耗时。实测起点数据已离线恢复并稳定跟踪，楼梯运动中的定位效果仍需实机复测。

**自动楼梯执行链（2026-10-14）：** `ASA 模式 → 参数事务/步态确认 → 对应感知 → 源码 NOS 自主规划 → robot.py` 已连接。平地保持 NX 原始点云过滤与计数网格；楼梯段使用 `/nvblox_lio/body_cloud` + **同戳 `/nvblox_lio/odom_lidar`** + `/IMU`，调用源码 AOS 累积、重力校正、高度图和代价图。高频 `/nvblox_lio/odom` 仍用于跟踪，不能代替同帧激光更新位姿。切换、取消清除旧感知可用状态；楼梯模式必须收到切换后的同步扫描和 IMU 插值区间，缺失超过输入时效即停车，不退回普通点云模式。启动会检查这些流及激光位姿/点云同戳配对。

高度图输出已经是“一点一个障碍格”，直接进入规划器障碍表，**不再经过原始回波的双点计数、最大 Z 排除或离群过滤**。保留官方高度阈值 0.25 m，`STAIR_OA=false`；`STAIR_OA=true` 是用 0.08 m 阈值避开台阶，不能当作上楼开关。RViz 的 occupancy 显示实际输入，楼梯时使用重力坐标系及对应 TF。含楼梯路线的进度投影使用地面高度（机身位置减遥测高度），完成条件核对目标楼层，避免同一 XY 不同楼层提前到达；没有添加机身支撑/足端检查。

**复现与适配边界：** 已复现捕获配置的 NOS 轨迹规划、原始点云过滤、DWA/直线避障状态，以及 AOS 高度图主链、任务参数和步态事务。高度图分别通过 200 组射线、60 组 IMU、32 帧累积回调、40 组建图、120 组代价图原始指令对照。源码 handler 的候选预测、碰撞、代价选优、减速和定时器决策也有指令对照，但它属于 HANDLE_STEER 链路，**不叠加到自主 DWA 后面**。本项目把高度图语义障碍接到 NOS 自主规划器，属于明确的接入适配，不声称这就是已证实的官方自动上楼拓扑。官方整套系统所有配置、handler 的完整传输、`PointInfo=4` 跨地图动作仍不在等价验证范围内。

2026-10-14 实机失败修复：会话 `20261014_144859_1ea038c6` 的跨层路径被旧水平面检测器全部漏标，3,432 条活动记录均为平地步态、楼梯高度图 0 帧。已将实际地图的原始三角面走廊和 8 条路径纳入 `global_planner/fixtures/stair_scene_20261014` 回归；两条升高 3.5 m 的路径正确标注，其余 6 条平地路径保持平地，反向路径验证下楼。最后一条路线的楼梯模式区间为 2.909～10.868 m，早于实测停住的 3.283 m。`source/acceptance_captured_stairs.py` 使用这份真实地图/路线验证 ROS 模式、感知及模拟步态确认；位姿为路径回放，不是实际爬楼或速度闭环验证。

整链验收脚本 `source/acceptance_stair_execution.py` 从合成踏面 Mesh 生成 ASA 标段，提供带高度变化和倾角的传感输入，运行实际 `robot.py` 对 localhost UDP 模拟器闭环；检查上/下楼、平台恢复、延迟步态 ACK、IMU 中断停车、取消及 RViz 输入。它验证软件链，不模拟足地接触，不能称为实机上楼成功。原高度图的 10 cm 栅格及中值过滤会损失某些细薄障碍；复现保留了这一行为，不能把宽墙测试外推到所有障碍。证据见 `evidence/source_20261014/stair_*`。

真机手动测试使用默认源码后端，无需另启高度图/handler 节点：

```bash
cd /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow
bash official_first_integration/start_live.sh --execute --enter-navigation
```

默认定位在录包起点附近搜索；若已移动到其他位置，沿用 `--initial-map-pose X Y Z YAW`。在 RViz 点包含已建图楼梯的目标，`metrics/planner.jsonl` 可检查 `asa_mode.terrain`、`requested_gait/actual_gait`、`perception_mode=aos_height`、`height_frames`、`gravity_matched` 和 `hold_reason`。模式、点云、定位未就绪的原因都会记录；`global/status.jsonl` 和地图目录下的 `queries/*.json` 记录 `mode_annotation`，可检查实际网格覆盖率及两种识别分支；完全缺失的地形证据仍不会凭路径 Z 强制标楼梯。

以下原始二进制后端说明保留作对照历史（2026-10-13），不代表现在的默认后端。

The ASA global planner and localization remain in place. `native/adapter.py` converts their path into the official external `/local_goal` interface (1.5 m route lookahead, 10 Hz), aligns odometry, converts messages and applies input/command freshness gates. This integration code is not claimed to be the official global planner. The default no longer runs the custom rolling occupancy grid, trajectory continuity cost, route-heading score, stair gait switching or incremental retreat state machine.

The official cloud filter uses X/Y ±4 m, Z [−.30,.60], 1 cm downsampling, its body crop and radius outlier filter. The trajectory lookup is 90×80 cells at 5 cm (4.5×4.0 m, offset −1.475/−1.975 m). The backup's 4.2×2.2 m elevation map is a different component. RViz displays the actual `/NAV_POINTS` output at up to 5 Hz. Official limits are 1.5/.6/1.0 for X/Y/yaw; the sender preserves native float32 commands within those component limits.

Each run uses a private user/network/IPC namespace, a dummy eth0, isolated shared memory, private logs/configuration, and a Unix socket ROS adapter. It cannot publish native DDS commands onto the robot network. Only the existing explicitly enabled, watchdog-protected `robot.py` sender can move the robot. No NOS service is replaced. The session copy changes `local_try` to false to accept ASA local goals; the official parameter service selects autonomous obstacle avoidance (`directLine_mode=false`). Other supplied service values match the captured YAML. Binary and asset hashes are checked before startup. SDK assets are excluded from Git; `native/capture_sdk.py` is the read-only export helper, and `native/build.sh` builds the transport adapter on Jetson.

The original filter has a confirmed timestamp bug at 0xc6c78–0xc6d08: it sets output nanoseconds to zero. The adapter associates exactly one in-flight input frame with its filtered result and restores the acquisition stamp on ROS output only. Native cloud geometry and messages received by the original planner are unchanged. Freshness uses the validated final point timestamp for NX scans, matching the previous scan-end convention. Original trajectory loading takes about 6 seconds, followed by a 2-second initialization stage; goals wait for the official service to become ready.

Validation: isolated native binary service/velocity test; ROS closed-loop 90° corridor turn with zero sampled body/wall intersections; point-cloud timeout, cancellation/new goal and localization timeout/recovery; original-binary captured-cloud replay; sender unit and localhost UDP tests. This is not a completed physical corner or staircase acceptance test. Evidence is under `evidence/native_20261013/`. The previous launcher configuration is backed up on Jetson at `backups/native_20261013/before.tar.gz`.


```bash
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/official_first_integration/start_live.sh
# Explicit supervised execution; a new RViz goal is required after every start.
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/official_first_integration/start_live.sh --execute --enter-navigation
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/official_first_integration/stop.sh
```

The default map is explicitly pinned to `scanplanner_integration/maps/20261012_confirmed_scene`. Use `--map DIRECTORY` to select another built map. `--initial-map-pose X Y Z YAW` supplies a measured localization search seed; it never bypasses scan matching. RViz displays the local occupied cloud used by planning, measured M20 model and selected trajectory, plus ASA's global map and path. The default mode receives robot telemetry but sends no velocity, mode or gait changes. `--execute` enables the guarded sender; `--enter-navigation` explicitly requests navigation mode. The deployment uses a hash-verified same-recording SuperLIO reference for localization when `maps/localization_reference.json` matches the selected Mesh. Initial search covers ±2 m with 0.5 m seed spacing and full yaw search, stays within 3 m horizontally and 0.5 m vertically of the search anchor, and excludes quality-failing candidates before ambiguity checks. Startup defaults to searching near the recording start; the last matched pose is retained only as diagnostic output and is never automatically used as the next search center. `--recording-start` remains a compatible explicit spelling of this default. Use `--initial-map-pose X Y Z YAW` to deliberately choose a different search center. All starts still require fresh matching and stability confirmation. This remains local relocalization, not arbitrary whole-building place recognition. Use RViz Publish Point to select the goal after localization succeeds. `--duration SECONDS` bounds the session lifetime. Executing starts without a route; old retained paths and commands are isolated through unique session topics.

Stopping terminates the sender before the GUI/planners, sends three zero-velocity packets 33 ms apart, and requests restoration of the previous usage mode if this session changed it. `runtime/SESSION/telemetry/shutdown.json` records these sends; UDP delivery is not guaranteed by a successful send call. The session cleans up only processes it owns.

## Legacy implementation history (`--local-backend legacy`)

The implementation details below describe the previous C++ planner, not the default NOS binary chain.

## Implemented

- All 4,641 official constant body-velocity primitives and four correspondence tables, with strict parsing and captured hashes. `evidence/audit.md` distinguishes verified binary behavior from unfinished state-machine work.
- Hard collision occurrence counts; binary soft-layer flags; audited goal/yaw/lateral/proximity score and ordered candidate validation. Native candidates are selected under speed limits. Recovery selects official reverse primitives through the same tables and swept-body check.
- A bounded continuity preference in native trajectory ranking reduces near-tie switching, using only the most recently published valid command from the same safety generation (under 120 ms old). It adds at most 0.08 to the native score, with x/y/yaw delta weights 0.10/0.16/0.16. Candidates still pass the official tables, current limits and full collision checks. This is a project extension, not a recovered official binary score term.
- Trajectory, direction and terminal-phase changes receive a 0.22 s command blend from the last published valid command. Stable tracking passes through directly; there is no acceleration limit. Retargeting an active transition preserves its origin, start time and deadline, so frequent primitive changes cannot restart the blend at zero. Holds and input loss publish zero immediately and discard blending history. Gait speed limits still apply. At the operator's request there is **no additional collision check of the blended command**; the checked target primitive does not certify every intermediate command. RViz shows the blended command's nominal sweep while switching.
- When no usable forward primitive exists for 0.65 s, recovery tries FIRST's reverse library toward a point 0.30 m behind the body on the first executed attempt and 0.50 m on the second (clipped to the remaining .85 m cumulative budget), with limits .4/.3/.4 and planar .4 m/s. Reverse selection preserves body heading and checks the live rear cloud, official collision tables and full sweep. Rear data must be under 0.35 s old; a front-only frame preserves the last actual rear timestamp, while geometry/posture invalidation clears it. Stairs disable retreat. Retreat ends upon reaching its requested distance, after 4 s, when rear freshness/stair gating disables it, or when forward motion remains available for .3 s after at least .15 m measured retreat. The operator explicitly retained this early-release rule, so .30/.50 m are targets rather than guaranteed distances. Reverse collisions still stop commands immediately. A 0.3 s recheck precedes forward replanning. An attempt consumes the two-attempt budget only after at least 5 cm of measured displacement; sensor interruptions and entirely rejected reverse candidates do not consume executed attempts. Cumulative retreat is capped at .85 m. After exhausting the budget, recovery holds until forward planning becomes feasible or a new route arrives; .5 m of forward progress resets the budget. These are project recovery transitions, not a claim of equivalence to the complete official state machine.
- Audited native terminal controller: approach inside 0.5 m, position tolerance 0.1 m, then yaw alignment. Its actual command is independently swept and checked; route arc/floor guards prevent early arrival on overlapping routes.
- New obstacle scans are accepted in stable standing (gait 0x1001, no pending gait change) or a confirmed navigation gait after reported height remains stable for at least 0.3 s (within 1.5 cm). Queued scans acquired before stabilization are discarded; existing occupancy is retained. Stable standing populates the obstacle preview before any goal is selected; other unsupported postures do not. Motion still requires a route and confirmed navigation gait. MotionStatus source timestamps prevent callback delays from refreshing stale height.
- Gait confirmation allows 6 s for the physical transition and telemetry reporting; advance stair requests include that horizon. Missing confirmation still latches zero output with an explicit timeout diagnostic. A late confirmation does not release that latch or resume an old route. Global status history preserves failed clicked goals and their measured starts in `status.jsonl`.
- M20/AIRY near-body filtering rejects raw rays that cross the strict interior of the central chassis bottom face, using the URDF box and decoder optical-origin offsets. Endpoints remain unchanged; rejected returns create neither hits nor clearing rays. This targets the captured low-channel cluster without a blanket minimum-range increase. The collision geometry is an approximation of the opaque housing; this is a limited geometric hypothesis, not a calibrated multipath detector. Close-object fixtures pass; physical acceptance is recorded in the execution evidence.
- Before voxel deduplication, deskewed returns within 1.5 m of their physical sensor require two distinct neighboring points within 4 cm. Distant sparse returns bypass this filter. Rejected returns create neither occupied cells nor clearing rays. Thin-object fixtures pass, but single-return real objects can be suppressed; hardware acceptance remains required. This uses the [radius-neighbor filtering principle](https://pointclouds.org/documentation/tutorials/remove_outliers.html).
- A flat-route command that produces neither 8 cm of translation nor .20 rad of net rotation for 2.5 s triggers the same bounded reverse recovery. Small alternating yaw commands do not count as progress; missing-input intervals do not count as failed motion. Entry joining, stairs and final approach retain their separate handling.
- Lookahead starts from `clamp(1.0 + measured_speed, 1.0, 2.5)` metres, using a measured 200 ms translation baseline. It is divided by `1 + 1.5 * preview_angle` (radians), then bounded to .45–2.5 m; the preview angle is the largest heading error over .4 m route chords sampled every .2 m within the speed-based horizon, avoiding false slowdown from individual 5 cm mesh zigzags. The geometric corner guard can shorten the resulting target further. Local lookahead bounds deviation from the ordered route to .10 m across intermediate vertices, including duplicate yaw samples and distributed bends. The body-to-target chord must converge toward the route. Cross-track error above .35 m enters reduced-speed rejoining, released below .15 m. These guidance bounds do not replace live collision checks or prove the saved route traversable.
- Local cloud evidence uses a rolling **5 × 5 m** horizontal window (100 × 100 cells at 5 cm), retaining the existing 6 m vertical span. The window follows measured body position in 0.1 m horizontal steps (0.5 m vertically). Rays are clipped to this rectangular box; the previous 4 m radial cutoff is removed. Only real endpoints inside the box create occupied cells; clipped boundary endpoints remain free, and silence never clears obstacles. The initial expansion showed material processing overhead and freshness holds. Subsequent latency fixes and paired replay measurements are recorded in the execution evidence; uninterrupted physical navigation still requires field verification.
- Route-following yaw ranking uses the stored route heading up to .25 m ahead of current progress (bounded by the accepted positional preview), independently of bearing to the preview point. The forward route-heading term uses weight 1.0; reverse retains .5. This anticipates turns without changing native velocity samples or applying an output multiplier. This avoids a short preview and lateral tracking error continually rotating the body away from its route heading. Duplicate yaw samples within that bounded heading preview select the outgoing heading. Map entry uses its entry heading; reverse retains its recovery heading. The native default score remains available unchanged; this route-heading override is a project extension. `route_heading_rad` exposes the actual world-frame heading used for forward ranking.
- The perception thread exclusively owns the dense evidence grid and publishes immutable occupied-point snapshots, avoiding a dense grid copy every scan. Scratch hash tables grow with actual unique cells. Collision classification uses an exact 4 m broad phase: every official table cell is within 3.65 m and every currently allowed swept body fits within 2.1 m. Distant points remain in the 5 m evidence and displayed cloud; this does not reinstate the former 4 m ray cutoff. Tests cover table/sweep bounds and hash-growth data preservation.
- Near-neighbor filtering reuses separated witnesses in the existing FIFO spatial bin to prove acceptance, with the original exact fallback and first-neighbor order. This removes the second fine-grid hash pass while preserving point acceptance. Ray clearing uses three private worker bitsets merged with same-scan hit precedence, with serial/parallel and dense differential tests. Map-aligned odometry publication is limited to 30 Hz for visualization and clicked-start consumers; raw pose history, deskew and internal body updates retain their input rate.
- The RViz odometry arrow is replaced by the M20 URDF in an isolated `first_robot/*` TF tree. Map-aligned odometry drives the body pose; telemetry drives calibrated leg joints. Wheel rotation is a relative integral of measured wheel speed, not an absolute measured angle. These visualization nodes send no robot commands.
- RViz displays `/first/occupancy`: the full processed 5 × 5 m occupied point set used as the source for local planning, including ground and distant returns. Display publication is 5 Hz; evidence updates retain every processed scan. `/first/planning_obstacles` remains available for inspecting the nearby collision classification.
- Every launcher session now records newly occupied near-body cells to a bounded `near_body_hits.jsonl` trace (three files of up to about 64 MiB each). It includes raw return, sensor/ring, acquisition time, transformed hit/voxel and body position; established occupied cells are not repeatedly serialized. `--trace-near-body` enables the heavier full near-body outcome trace. Recording is diagnostic only and changes no occupancy or collision decisions.
- Independent per-return LiDAR deskew using measured odometry, front/rear physical ray origins, occupied/free/unknown voxels, direct-ray clearing, and hit precedence. Silence does not clear obstacles.
- Saved surface heights are optional references for obstacle/ground separation and trajectory height, not support certificates. Missing center or footprint support never rejects a local candidate. Body odometry minus reported body height supplies the current ground datum. Saved heights are shifted to that datum and contribute only relative relief; they never replace the measured ground height. Where a reference is absent, candidate height continues from the previous sample. Real-time occupied-voxel collision checks retain the physical footprint. This does not implement live hole/drop-off detection.
- ASA route progress follows arc order with a speed/angle-adaptive target and corner preservation. A separate route revision resets progress only on route replacement/cancellation; localization/posture changes still invalidate commands and cloud evidence while retaining progress in the fixed map. Floor height gates prevent jumping to the nearest XY route on another floor.
- Live ASA queries may choose a nearby valid map entry when the measured start/heading fails saved-map checks. Entry candidates are ordered by 3D distance then heading change, bounded to 0.75 m horizontally and 0.20 m vertically. Snapped graph entries retain all owning regions and use graph headings; an arbitrary measured heading can fit in place yet have no valid exit to the graph. FIRST first approaches/aligns with that entry at the reduced .4/.3/.4 limits using live obstacle checks, then follows the ASA route. The query records both actual and entry poses; it does not invent a saved-map certificate for the connection. Manual/offline queries retain strict behavior unless `snap_start=True` is requested.
- Stair gait request ahead of a height transition (a five-sample, 5 cm metric median is used only for gait classification to reject isolated mesh-height spikes); no routine pause while waiting on flat terrain. A single outstanding gait request, RL-state restriction and telemetry confirmation are implemented. Unconfirmed gait blocks entry. Full-body clearance from height transitions controls return to flat gait.
- Perception reuses epoch-tagged flat key tables for neighbor bins and per-sensor voxel deduplication. FIFO neighbor order, distinct-point tests and first-return selection are unchanged; rejected returns still generate no clearing rays.
- Exact collision checks use a spatial broad phase, static triangle heights at voxel centers are cached without caching live occupancy or floor choice, and an unchanged selected primitive is validated once. Per-return acquisition logging is opt-in with `--trace-near-body`; timing and rejection counters remain available without it.
- Separate perception, planning and 30 Hz command publisher threads; stale route, localization, cloud, pose or robot state suppress commands. Existing UDP command-age and robot-status watchdogs are retained.
- Explicit session ownership lock shared with the legacy launcher; only owned subprocesses stop. Sources, assets, builds, map derivatives and runtime records stay inside the original project.

Hardware telemetry defaults to `status_relay.py`: `AOS 10.21.33.103:30000 → NOS 10.21.33.106 → SSH → Jetson`. The transient NOS process only sends heartbeat 100/100, writes no files, and exits with the owned SSH connection. The original UDP socket still sends control commands and receives their acknowledgements/errors; direct UDP status cannot override the relay’s state. No official robot service or system network configuration is changed.

The relay retains NOS kernel receive timestamps. Expired, future-dated and reordered packets do not refresh watchdogs; disconnect immediately invalidates state and retries automatically without blocking the sender. Jetson and NOS clocks must remain synchronized. Authentication uses existing SSH keys, `FIRST_NOS_SSH_PASSWORD`, or the ignored mode-600 file `runtime/private/nos_ssh_password`. The deployed machine is configured; credentials are not part of the repository. `robot.py --status-source udp` is available for diagnostics, and localhost tests use UDP by default.

The external AOS-to-Jetson return path still loses packets: a simultaneous trace saw 395 outgoing MotionStatus frames but only 259 received. This fix routes this project’s telemetry around that path; it does not claim to repair the USB adapter/cable/switch or to fix UDP command acknowledgements. See `evidence/telemetry-fix.md`.

Robot status is still required: odometry provides body pose, but does not carry clearance above the ground, confirmed gait, navigation/manual mode or emergency-stop state. BasicStatus and MotionStatus freshness gates are unchanged. ASA still validates the chosen map entry and every global route segment against the saved map.

Configured native candidate limits: normal vx ≤ 1.5 m/s, |vy| ≤ .4 m/s, |yaw| ≤ .8 rad/s, planar speed ≤ 1.5 m/s; stairs .4/.3/.4, planar .4 m/s. All native candidate increments remain available under these caps, including gentle turns. Terminal position/yaw minimum handling follows the audited native controller; no post-planner multiplier is used. Robot deadband calibration remains unverified.

## Build and tests

```bash
bash official_first_integration/build.sh
python3 -m unittest discover -s official_first_integration/tests -p 'test_*.py'
# ROS synthetic tests require an isolated domain; they never start a robot sender.
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --stairs
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --no-support
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/udp_loopback.py --gait --normal-speed
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/udp_loopback.py --gait --gait-delay 4 --normal-speed
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/udp_loopback.py --gait-timeout --normal-speed
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --no-support --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --height-offset --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --posture --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --route-progress --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --map-entry --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --recovery --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --terminal-switch --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --corner --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/ros_chain.py --stalled --session
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 python3 official_first_integration/tests/udp_loopback.py --gait --normal-speed --enter-navigation --session
```

Use the project's ROS-enabled Python environment on Jetson. Vendor assets are not included in Git. `import_assets.py official_planner.tgz` verifies the captured archive against `evidence/manifest.json`. Core tests also build on a workstation with `cmake -S official_first_integration -B official_first_integration/build`, without any ScanPlanner or ROS dependency.

## Acceptance still required

1. Compare the recovered DWA score and terminal controller against the running original binary, and finish line-mode/state transitions. The entire original state machine is **not** behavior-equivalent or fully recovered yet.
2. Validate recorded stairs/open risers/landings and gait timing. Saved-map support rejection was removed at the user’s request; capture14 now produces a native candidate in the offline probe. This is not physical stair acceptance or a replacement live negative-obstacle detector.
3. Physically validate FIRST's new map-entry connection, switching blend and reverse recovery. Offline checks and isolated ROS fixtures are not supervised driving acceptance.
4. Complete supervised driving acceptance. Session `20261012_150133_1fcbf8c9` confirmed navigation mode, flat gait and 0.96 m measured displacement toward the existing ASA goal. The planner then rejected all speed-admissible candidates through the hard collision table. Stop packets and restoration of manual mode were exercised; the subsequent read-only capture confirmed mode 0. After correcting the saved-ground height offset, a further trial advanced 0.575 m but stopped again on accumulated low voxels; this remains partial execution acceptance. See [execution trials](evidence/execution-trials.md) for evidence and unresolved posture-transition behavior. The October 13 flat-route trial reached the original goal and the operator reported essentially smooth motion. Rare collision holds and cross-floor execution remain unaccepted; the subsequent continuity preference has separate synthetic validation and awaits field observation.
5. Measure timing under sustained navigation, including stairs. The first hardware trial averaged 30.003 Hz UDP sending, but the maximum send interval was 111.4 ms, so an uninterrupted 20 Hz minimum is not yet established. Motion/basic status receive gaps remained below 0.140/0.546 s; the seconds-long telemetry loss did not recur.

Legacy start scripts remain unchanged until these acceptance conditions pass. Do not interpret this directory as a completed ScanPlanner replacement.


## 2026-10-14 AOS 保存点与源码接入

用户确认官方自动上楼使用预先保存的点/任务。AOS `basic_server` 的 `NavWorker::handle_task`（0x1a1450）依次设置任务参数、读取任务 `Gait`、调用 `SwitchGait`、发送路径点和目标；点属性 4 是“楼梯准备点”，到达后调用 `CallMapSwitch`。这是任务驱动的切换证据，不能称为已经找到“临时点任意目标后的实时楼梯分类器”。NOS `motionInfoHandler` 只读取速度，不读取步态。

`source/task_profile.py` 根据捕获的 AOS 导航配置选择参数：`NavMode=0` 直线，`1` 自主；`Gait=12290 (0x3002)` 平地、`12291 (0x3003)` 楼梯；**`Speed=0` 正常、`1` 低速、`2` 高速**。`Manner=1` 后退。`ObsMode!=0` 写入 `judge_close=true`，原直线规划器据此跳过障碍检查；不能从楼梯步态自行推导这个值。配置中的 ORIENTEER 表虽被加载，但捕获的参数选择分支没有为其它步态赋索引 2，不能按名字猜测选择逻辑。

源码保存点入口是每个会话隔离的 `/first/saved_goal`（`std_msgs/String`，实际 remap 见 `runtime/SESSION/session.json`），一次请求必须同时提供当前 ASA `se2_scan_map` 坐标与 `PosX/PosY/PosZ/AngleYaw`（米/弧度）、`Gait/Speed/Manner/ObsMode/NavMode`。不提供自动执行的示例目标；请求本身表示选点。`/first/cancel`（std_msgs/Empty，同样按会话隔离）取消保存点，RViz 新点击也取消原任务。该桥接并非官方任务文件导入器。它只接普通点 `PointInfo=0`；`PointInfo=4`、跨地图 `MapID`、额外 Terrain/Posture 动作会明确拒绝，不能静默略过。普通 RViz 点击重置成独立的默认规划参数，保存点的楼梯参数不会遗留。

`source/gait.py` 复现 AOS `VelControl::SwitchGait`（0x1adbf0）的三次发送、每次最多 3000 ms 等待确认。源码发送器只接已用于本机导航的 0x3002/0x3003；等待期间发零速度，状态过期或操作模式改变时不重发。取消/换目标不会抹掉已发出的物理步态切换：先保持零速度等待旧切换确认或锁定故障，随后才能发新请求，避免重叠。三次失败锁定发送故障；晚到的确认不恢复旧目标。官方任务代码里未看到对 SwitchGait 返回值的有效拦截，源码的确认后放行属于明确的接入保护。

AOS `height_map_nav` 提供 `/HEIGHT_MAP_STAIR_OA_ENABLE` 服务端。源码 `heightmap.py` 复现回调及 `computeCostMap` 有限高度图分支：四邻域相对高差、保留原始点计数与低邻域判定。**OA=1 选约 0.08 m 阈值，OA=0、已知地形选约 0.25 m；开启此开关使 13 cm 台阶被挡住，不是开启爬楼。** `height_frontend.cpp/.py` 已补齐捕获配置的上游链，`height_adapter.py` 提供独立 ROS 接口。默认 NOS `pcl_pass_grid → /NAV_POINTS → localPlanner` 输入链不是这个分支；不能把单点障碍格直接送回需要每格至少两个原始回波的计数器。

源码局部状态保留自主规划的跟踪、无可用轨迹、目标占障、终点调整，以及直线规划的初始转向、接近、终点对齐、停障与十周期清障等待。没有在原 `runDWA` 中找到逐次加长回退距离的独立恢复机；后退候选属于同一轨迹选择。不同于官方的接入保护包括数据时效、会话隔离、路线剩余长度约束；显式关闭直线 OA 的保存点不额外重新开启碰撞检查。

任务层另接入 `supervision.py`：停障、连续规划异常、定位丢失和指令未刷新按 AOS 共用的“最后健康时间”计时，默认超过 30 秒锁定本次任务；异常类别变化不会重置计时。新目标才清除任务故障。Go2Point 可选逻辑按捕获配置关闭。

### 对照和验证

- 原始 AArch64 指令离线仿真：步态事务 704 组、任务参数 48 组、OA/地形回调 66 组、预构建高度图 120 组、持续异常计时 512 组均通过。仿真只替换日志、消息存取与发送/等待接口，未连接硬件；各脚本说明了具体替身范围。
- DWA 9 个静态场景、直线 6 个静态场景与原二进制对照见 `evidence/source_20261014/`；不据此宣称所有状态/输入均已等价。
- 源码行为单测 27 项、发送器单测 13 项；本机独立 ROS 域的 90° 拐角无采样机身穿墙、输入超时/取消恢复、保存点直线和自主两种模式的步态等待/参数复位均通过，自主分支另外验证拒绝同 XY、不同楼层的错误路径。保存点测试覆盖真实空 PointCloud2，修复了空数据字段偏移导致工作线程终止的问题。
- UDP 发送器另外通过 127.0.0.1 模拟器：4 秒后确认触发两次步态请求，始终不确认触发三次请求后锁定，等待期间零速度，退出发送三个零速度包。
- 实机楼梯和跨地图任务未测试；代码中的 `stair_reconstruction_complete=false` 保留，避免将内核对照通过误报成完整上楼能力。

### 开源来源判断

ELF 动态符号直接证明使用 PCL 的 PassThrough/CropBox/VoxelGrid/RadiusOutlierRemoval、Eigen、OpenCV；独立 `passable_area` 还依赖 ANYbotics `grid_map` 的数据结构和转换。NOS `DwaPlanner` 实际使用 4641 条预计算全向定速轨迹和四层碰撞对应表，不能仅凭类名断言是 ROS `dwa_local_planner`，也未见标准完整加速度动态窗口。它与 CMU 轨迹库/对应表方案的思路接近，但 CMU 样本数及非均匀索引与这里不同，没有证据确认直接 fork。AOS 高差/计数判障与任务状态机按本机指令复现，未找到可确认的同名开源实现。RL 中出现 HiMLoco 名称不等于已证明策略来源；底层关节控制仍由机器人原控制器完成。

参考：[PCL filters](https://pointclouds.org/documentation/group__filters.html)、[grid_map](https://github.com/anybotics/grid_map)、[CMU path generator](https://github.com/jizhang-cmu/autonomy_stack_go2/blob/foxy-humble/src/base_autonomy/local_planner/paths/path_generator.m)、[原始 DWA 论文](https://publications.ri.cmu.edu/storage/publications/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)。其中库依赖是确定证据，算法血缘尚无直接证明。

### AOS 高度图源码组件（2026-10-14）

`source/height_adapter.py --output DIR` 接收 `/nvblox_lio/body_cloud`、同一时间戳的完整 `/nvblox_lio/odom_lidar` 和 `/IMU`；这是 ROS 传输适配，计算模块不依赖 ROS/DDS。输出 `/first/source_height/cloud_gravity`、`cloud_nav`、`status` 及重力帧 TF。输入必须是已经完成 LIO 配准的机身点云，不能以原始 NX 点云冒充。该独立工具没有运动指令或步态请求发布者，不由默认自主导航启动器额外启动；默认源码执行器在楼梯段直接调用同一计算模块。

`/first/source_height/terrain`（Int32）对应已复现的地形回调；`/first/source_height/stair_oa`（SetBool）对应楼梯避障开关。开关只改变是否把台阶当作障碍，不负责切换楼梯步态。此独立工具未找到包围点云时刻的 IMU 时，保留原程序未校正点云的分支，并通过 `gravity_matched=false` 报告；实际楼梯执行输入则等待有效 IMU 区间，过期停车。空观测、累计清除、机身转动、时间未同步与开关语义均有回归覆盖。

配置按捕获 YAML：外窗 X/Y ±4 m、Z −1.5～0.5 m，内框 X ±0.4/Y ±0.2/Z −0.5～0.4 m，两次 5 cm 体素，10 cm 八叉树和高度格、81×81、每格最后 30 个样本。并行射线采用 X=±0.4 m 起点、端点缩短 0.1 m、最多四个相交体素；串行分支另保留近身/侧带点。源码串行遍历并行分支的射线并求清除集合，几何结果与原多线程集合一致；耗时单独记录，不增加控制循环线程。

离线验证：`test_height_frontend.py`；原始指令对照：`differential_height_frontend.py`、`differential_height_sync.py`；隔离 ROS：`acceptance_height.py`。后者必须在隔离 ROS 域运行。对照中 PCL 查询/滤波委托给安装的 PCL，不能据此宣称所有 PCL 版本完全等价；本机和 Jetson 分别构建验证。没有启用原配置关闭的插值、深度/图像调试或 change-height 分支。

### NOS handler 源码组件与验证范围

`source/handler_core.cpp` 和 `source/handler.py` 实现 `HANDLE_STEER` 速度避障计算及单周期状态决策。它与默认 `NAV_POINTS → localPlanner` 分开；不会在 ASA 输出后再叠加一层避障或机身验证，当前也没有新增真实运动发布器。

捕获配置的点云前端是外裁剪 X/Y ±3 m、Z [-0.6,0.3] m，去掉机身框 X ±0.4、Y ±0.2 m，再按 0.1×0.1×3 m 做 PCL 体素过滤。这里保留单个障碍返回，不使用自主规划那条链路的“每格至少两个原始点”条件。

速度候选在当前速度正/负方向上分别结合 0.8/0.6/0.8 的搜索范围，边界四舍五入到 0.1，按 0.2 采样。每个候选以 0.15 秒积分 6 步；默认 Euler，可选 Simpson 仅作用于第一步。按照末端到起点的顺序检查官方矩形轮廓，结合 360 项边界距离表计算余量；以速度偏差和倒数余量代价选优。保留原始 float/double 转换、FMA 顺序及同分比较，不另外添加恢复轨迹或扫掠校验。

官方减速分两段：余量 0～0.25 m 对应 slow2～slow1；0.25～1 m 对应 slow1～最大速度。前后速度再乘俯仰角的 cos²。`SetOoaParam` 只改变硬上限，不重算初始化时的分段系数，此处保留捕获版本行为，不将它描述为全区间硬限速保证。选优后还会按本周期更新的边界重新限幅，并转换为 float。

状态输出为关闭 0、空闲 1、一般运行 2、余量不大于 0.3 m 时 3。输入未就绪或遥控输入从有到无的首个检查周期会清空旧速度；无有效候选输出零并清空旧速度。楼梯避障开关在后处理阶段下发，包括无有效候选的分支；原程序即使服务调用返回 false，也记录本次请求值。这里的 StairOA 仍表示“把楼梯视为障碍”，不是上楼步态开关。

`differential_handler.py` 在离线 Unicorn 中运行捕获 handler ELF 的完整 `CalAvoidance`（`is_transform_cloud=false`），计算函数使用原始指令，替换内存分配、容器存储和 libc/libm 外部调用；24 组场景的选中速度、余量及有效候选数一致，另覆盖 600 次积分、300 次路径判定、55 组限速。`differential_handler_state.py` 对照 144 组 RunTimer/PostProcess 分支和 100 组归一化速度输入；候选结果由测试注入，传输/定时器及长度为 1 的输出缓冲使用测试替身。证据见 `evidence/source_20261014/handler*_comparison.json`。Jetson 的固定原始指令样例无需 Unicorn 即可回归。

AOS 高度图前端在 Jetson 以 3 万点合成输入、16 帧连续处理测得，稳定帧 P50 64.84 ms、P95 67.68 ms、最大 69.11 ms，包含建图与代价图，不含 ROS 传输或运动控制；这不是现场性能保证。源码与测试已同步，程序保持由操作者手动启动。

Jetson 的 handler 单次候选选优离线测量：空点云 P50 0.96 ms，含 60 个障碍点的合成门口场景 P50 40.67 ms、P95 40.84 ms（每组 12 帧，去掉前两帧）；详见 `handler_jetson_benchmark.json`。这些数值不能与不同输入/线程的高度图耗时直接相加当作现场周期。加入楼梯输入同步和语义障碍接入回归后，源码单元测试在本机与 Jetson 均为 48 项通过。

handler 的算法属于采样速度、短时运动预测、碰撞筛选和加权代价选优这一类局部避障；有明确的 Eigen/PCL 调用证据，没有证据把它标成某个 ROS DWA/TEB 开源包的直接移植。反汇编中确认的规则与项目新增的 ASA 地形分类、ROS 传输适配在本文中分别说明。
