# Live navigation / 实机导航

当前默认链路为 **ASA 全局规划 + `source` 局部规划 + M20 执行接口**。跨楼层导航已完成接入：全局路线携带平地／上楼／下楼模式，局部执行器按段切换感知、规划参数和步态，使用实时点云完成避障与路线跟踪。运行时使用本项目编译的 `build/source/libnos_source.so` 和经过哈希校验的轨迹／碰撞数值表。

The default backend integrates cross-floor ASA routes, terrain-dependent perception, local trajectory selection, gait transactions and M20 command output. Initialization searches the selected recorded map; continuous-odometry recovery uses bounded pose corrections. This guide describes the current runtime. Dated test records are linked under validation.

## 启动与停止 / Start and stop

在已配置的导航机上运行：

```bash
cd /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow
bash official_first_integration/start_live.sh --execute --enter-navigation
```

等待定位进入 `tracking`、全局规划进入 `ready`，在 RViz 使用 **Publish Point** 点击目标楼层的地面。每次启动使用独立会话话题，初始没有路线；选点后才执行。

```bash
bash official_first_integration/stop.sh
```

停止时先关闭指令发送器，发送零速度并请求恢复此前使用模式，再关闭所属进程。发送结果保存在 `runtime/SESSION/telemetry/shutdown.json`。

不带 `--execute --enter-navigation` 启动时只接收遥测和显示，不发送运动或模式切换指令。`--duration SECONDS` 可限制会话时长。

## 部署与构建 / Build

依赖 ROS 2 Humble、项目 ROS Python 环境、C++17/CMake、Eigen3、PCL、NumPy、SciPy、Shapely 2，以及已配置的雷达、IMU、SuperLIO 和 M20 通信接口。

```bash
# 在项目根目录执行，轨迹资源归档需另外提供：
python3 official_first_integration/import_assets.py /path/to/official_planner.tgz
bash official_first_integration/build.sh
bash scanplanner_integration/build_stair_lio.sh
```

`build.sh` 编译导航组件、运行 CTest 并构建 `source` 库。仅重建局部计算库可运行 `bash official_first_integration/source/build.sh`。密集机身点云构建脚本默认使用项目同级的 `Super-LIO-nx-scanplanner-port`，也可通过第一个参数指定源码工作区；构建输出位于项目内，不修改上游工作区。

轨迹归档、原始 bag、生成地图、私有凭据和预编译二进制不随 Git 分发。部署路径及外部服务须与目标设备匹配，见 [Jetson 部署](../docs/jetson.md)。

## 定位 / Localization

默认地图为 `scanplanner_integration/maps/20261012_confirmed_scene`，通过 `--map DIRECTORY` 选择其他已构建地图。同录制参考由地图哈希绑定；地图缺少该参考时，匹配能力以实际加载的参考类型为准。

默认首次定位使用 8 米范围的去畸变扫描，在地图通行区域和录制轨迹产生的位置候选上搜索多楼层、全朝向。距离场粗排采用 48 个朝向，最多精配准 64 个分散候选；距离场本身不授权位姿。独立留出点检查表面匹配质量，结构内点比例至少 78%，并检查六自由度几何约束和竞争候选。通过五次新鲜稳定匹配后才发布有效定位。

无需回到录包起点，也不把上次终点当作可信初值。`--recording-start` 显式限制到录包起点附近；`--initial-map-pose X Y Z YAW`（米、弧度）指定已知搜索中心。两种指定方式仍须通过匹配。

跟踪使用近身 4 米参考匹配。连续里程计会话内失配时优先在最近有效变换附近恢复，修正上限为 15 cm / 0.06 rad，并保留有限表面质量、候选歧义和五次稳定确认。里程计跳变或发布者变化会清除恢复先验。定位失效会清除路线，恢复定位后需要重新选目标。

定位的 8 米观察范围不改变局部避障点云范围。重复楼层、观测不足或地图变化仍可能使首次定位等待。

## 跨楼层路线与模式 / Cross-floor execution

ASA 执行 A* → 三维路径拉直 → 朝向 A*，保留地图几何检查。居中偏好目标为 0.45 m、权重 6，非绿色区域软代价权重 4；优先选择净空更充分的路径，必要窄通道仍可通过。局部前瞻为 1.5 m。

ASA 从同一份 Mesh 的踏面及持续爬升结构生成地形分段，经会话隔离的 `/se2_global/navigation_route` 发布路径和导航属性。普通 `Path` 用于显示。平地／楼梯模式如下：

| 路段 | 局部参数 | 请求步态 |
| --- | --- | --- |
| 平地 | AUTO / FLAT / NORMAL | `0x3002` |
| 上楼、下楼及其准备／退出段 | AUTO / STAIR / NORMAL | `0x3003` |

普通 RViz 选点默认 `NavMode=1、ObsMode=0`，即自主避障。执行端收到参数 ACK 和实际步态确认后放行；新目标、取消或重规划清理旧模式。机身步态转换由底层机器人控制器执行。

路线进度使用地面高度（机身位置减实测高度），到达判断同时核对目标楼层，避免同一 XY 不同高度提前完成。模式标注不额外加入机身支撑或足端验证。

## 局部感知与避障 / Local perception and avoidance

局部规划从 4,641 条预计算速度轨迹中筛选和评分，结合碰撞对应表完成自主或直线模式避障。默认点云裁剪为 X/Y ±4 m；楼梯高度图使用同一扫描时刻的密集去畸变机身点云、里程计及 IMU，输入来自 `/nvblox_lio/body_cloud`、`/nvblox_lio/odom_lidar` 和 `/IMU`。

高度图前端采用 5 cm 体素过滤与 10 cm 高度格，81×81 格、每格保留最后 30 个样本。姿态对齐后输出语义障碍格，直接进入规划器障碍表。`STAIR_OA=false` 配合 0.25 m 高差阈值用于楼梯通行；`STAIR_OA=true` 使用约 0.08 m 阈值避开台阶，该开关不是上楼步态请求。

自主状态包括跟踪、无可用轨迹、目标占障和终点调整；直线模式包括初始转向、接近、终点对齐、停障和清障等待。后退候选参与轨迹选择，当前自主状态机没有逐次增加回退距离的独立恢复流程。

点云、IMU、里程计、定位和机器人状态的时效决定是否执行；无有效轨迹或输入过期时保持零输出。持续停障、规划异常、定位丢失或指令未刷新由共用健康计时器监控，默认超过 30 秒锁定任务，新目标才清除任务故障。

`source/handler_core.cpp` 提供单独的速度避障计算与定时器状态组件；该模块不叠加到默认自主局部规划链中。

## 保存任务 / Saved goals

保存点通过会话隔离的 `/first/saved_goal` 提供位置、朝向、`Gait/Speed/Manner/ObsMode/NavMode`。`NavMode=0` 为直线、`1` 为自主；`Speed=0/1/2` 为正常／低速／高速；`Manner=1` 为后退。保存任务使用显式属性，普通 RViz 选点使用 ASA 地形模式。

步态事务最多请求三次，每次等待 3000 ms；等待期间发零速度。取消或换目标不会并发发送相互冲突的物理切换请求。失败锁定后，晚到确认不会恢复旧路线。

支持同一地图的普通保存点 `PointInfo=0`。`PointInfo=4`、跨地图 `MapID` 和额外 Terrain/Posture 动作会明确拒绝。同一地图内的跨楼层导航已接入，不依赖跨地图任务导入。

## 显示与诊断 / Display and diagnostics

RViz 显示 ASA 地图和路径、实际参与局部避障的点云、局部轨迹及实测 M20 模型。楼梯点云使用对应的重力坐标系与 TF。

| 文件或字段 | 用途 |
| --- | --- |
| `runtime/SESSION/session.json` | 会话配置、进程、话题映射及退出状态 |
| `localization/` | 初始化搜索、匹配质量、跟踪和恢复状态 |
| `global/status.jsonl` 与地图 `queries/` | ASA 查询、地形标注和失败原因 |
| `metrics/planner.jsonl` | `asa_mode.terrain`、`perception_mode`、`height_frames`、`gravity_matched`、`hold_reason` |
| `requested_gait/actual_gait` | 请求步态与实际步态的对应状态 |
| `telemetry/shutdown.json` | 停止指令与模式恢复记录 |

## 验证范围 / Validation

- [首次定位](evidence/initial_localization_20261015/README.md)：导航机实时输入、无初值，约 30.65 秒进入跟踪，随后 66 次有效跟踪；工作站和 Jetson 均通过 9 项初始化／参考匹配和 3 项传输测试。
- [定位恢复](evidence/localization_recovery_20261015/README.md)：静止失定位位置约 3.80 秒恢复，随后 60 次有效跟踪；验证只发布诊断定位。
- [楼梯执行与路线居中](evidence/source_20261014/stair_midflight/README.md)：51 项源码测试、23 项 ASA 测试及 12 级上／下楼软件执行测试通过，覆盖 IMU 中断和取消停车。

跨楼层执行链已完成接入。软件闭环不模拟真实足端接触，静止定位测量也不代表运动中全部恢复场景；实机效果按场景与对应记录评估。历史测量仅描述其记录日期和代码版本，不作为当前默认参数说明。
