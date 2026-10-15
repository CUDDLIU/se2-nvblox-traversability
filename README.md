<div align="center">

# SE(2) Nvblox Traversability

**From 3D reconstruction to terrain you can reason about.**

Mesh-based traversability, ASA global routing, and FIRST-based local navigation for the M20 quadruped.

[English](README.md) · [简体中文](README.zh-CN.md)

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E)](https://docs.ros.org/en/humble/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

[Demo](#demo) · [Quick start](#quick-start) · [Live navigation](official_first_integration/README.md) · [Global planning](docs/global-planning.md) · [Architecture](docs/architecture.md) · [Jetson integration](docs/jetson.md) · [Validation](docs/validation.md)

</div>

SE(2) Nvblox Traversability turns an incremental [nvblox](https://github.com/nvidia-isaac/nvblox) mesh into a local map of feasible robot positions and headings. It checks the robot's physical envelope, floor support, step height, slope, and headroom, then shows how much heading freedom remains in each cell. A ROS 2 interface supports sensor recording, reconstruction from bags, parameter comparison, and RViz inspection.

The current live navigation chain uses **ASA for global planning** and a **trajectory-library local planner** with collision correspondence tables, candidate scoring, and straight-line/autonomous obstacle avoidance. The default `source` backend is built from this repository. **Cross-floor navigation is integrated end to end:** ASA terrain segments drive local perception, navigation parameters, gait requests and route execution.

Developed on an M20 robot with a Jetson Orin NX, dual LiDAR, and a RealSense D435i. The geometry core also builds and runs on a Linux CPU without ROS, CUDA, or robot hardware.

## Demo

[![RViz reconstruction and traversability demo](docs/media/demo-preview.gif)](docs/media/demo-full.mp4)

**[Watch or download the full demo · 3:56 · MP4](docs/media/demo-full.mp4)**

Recorded September 30, 2026. The preview contains two short excerpts at original speed; the full video retains the complete recording. This video demonstrates reconstruction and traversability. Rainbow colors encode mesh height. Green and orange cells encode current traversability; blue areas retain historical results.

<details>
<summary>Staircase view</summary>

![Multiple floors and staircase geometry in RViz](docs/media/stairs-frame.png)

</details>

## Highlights

- **Physical fit in narrow passages.** Feasible poses can lie inside a grid cell and between sampled headings. A passage just wide enough for the body can retain a chain of orange cells through the existing classifier.
- **Heading freedom at a glance.** Deep orange indicates a very narrow feasible heading interval; lighter orange indicates more freedom. Green requires feasibility across all headings with the configured margins.
- **Floors stay distinct.** A multilevel heightfield and local surface connectivity prevent an upper-floor edge from becoming a spurious obstacle on the floor below.
- **Incremental geometry.** A native C++/OpenMP core updates a robot-centered window from changed mesh blocks. Historical areas remain visible with separate validity semantics.
- **Record once, compare parameters.** Record sensor inputs without mesh reconstruction, keep a bag with its map and telemetry, rebuild from LiDAR, depth, or both, and tune while replay is paused.
- **Cross-floor goal planning.** Build a static all-floor map from a bag and select a 3D goal in RViz. A* → string pulling → heading A* produces a checked route through stairs, with flat/upstairs/downstairs annotations consumed by the live execution chain. [Workflow and relation to SE(2) NavMesh →](docs/global-planning.md)
- **Live map localization and local avoidance.** Match measured LiDAR surfaces to the saved reference, follow the ASA route using 4,641 native velocity primitives, and check live occupancy before sending M20 commands. Whole-map initialization and bounded recovery within a continuous odometry session are integrated.
- **RViz and robot state.** Height-colored meshes, traversability cells, and an M20 joint model share the reconstruction frame. Replay suspends the live reconstruction view; a button restores it.

This is a research and integration project. Geometry analysis and replay do not issue motion commands. The optional M20 integration enables hardware output only with `--execute`. Cross-floor planning and execution are connected in the default backend. See [current validation](docs/validation.md) for the scope of software tests and recorded field measurements, and [current limits](#current-limits) for deployment boundaries.

## Quick start

### Live navigation on the configured M20 / Jetson

Use the existing project directory on the robot. **Start with hardware command output enabled:**

```bash
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/official_first_integration/start_live.sh --execute --enter-navigation
```

**Stop the owned navigation session:**

```bash
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/official_first_integration/stop.sh
```

For telemetry and visualization without velocity or mode changes, run `start_live.sh` without either flag. After localization succeeds, use RViz **Publish Point** to choose a green/orange surface on the destination floor. The yellow path is the ASA route; FIRST displays its local trajectory and the actual classified obstacle points, alongside the measured M20 URDF.

Wait for the global planner's `ready` status: the static graph is prepared once at startup. Validated map geometry is cached across restarts. Two measured routes took 0.56–0.91 s with that cache; first validation took 1.52–2.93 s. See [ASA latency measurements and limits](docs/asa-latency.md).

The `source` backend uses X/Y ±4 m cloud cropping and autonomous avoidance by default. ASA supplies flat/upstairs/downstairs segments; the executor applies terrain-specific perception, navigation parameters and gait transactions. Local lookahead is 1.5 m. ASA prefers 0.45 m clearance and penalizes non-green regions to favor centered routes. See the [live navigation guide](official_first_integration/README.md).

Startup searches **the selected recorded map by default**, using measured scans within 8 m and position/heading hypotheses across mapped floors. It does not trust the previous session's endpoint. Refined matching, independent structural constraints, competing-pose checks, and five fresh stable confirmations precede localization output. Indistinguishable floors or insufficient observations remain unresolved. Use `--recording-start` to restrict the search to the recording origin, or `--initial-map-pose X Y Z YAW` (metres, radians) for a known search center; neither bypasses matching. The local obstacle-planning cloud range is unchanged.

The default saved map is `scanplanner_integration/maps/20261012_confirmed_scene`; select another prepared map with `--map DIRECTORY`. Live navigation uses the current measured pose, while the separate bag-planning viewer can use a recorded endpoint. Stopping sends zero-velocity packets before shutting down the other owned processes and requests restoration of the prior usage mode; sender logs record the outcome.

### Prepare a new Jetson deployment

The CPU example below does not install the robot stack. A new hardware deployment needs:

| Component | Requirement |
| --- | --- |
| Runtime | ROS 2 Humble, C++17/CMake, Eigen3, and the project's ROS-enabled Python environment |
| Live inputs | M20 LiDAR and IMU drivers, SuperLIO body odometry, calibrated frame conventions and clock synchronization |
| Saved map | A built `navmesh.json` bundle and source mesh; optional verified LiDAR localization reference |
| FIRST assets | Locally supplied official trajectory/collision archive, verified against the included manifest |
| Robot link | Reachable M20 command/status interfaces and private NOS relay authentication |
| Reconstruction | Installed nvblox/Isaac ROS and project CUDA adapters when building maps or replaying sensors |

```bash
# From the repository root, with the separately supplied asset archive:
python3 official_first_integration/import_assets.py /path/to/official_planner.tgz
bash official_first_integration/build.sh
bash scanplanner_integration/build_stair_lio.sh
```

The archive, private credentials, field bags, generated maps and prebuilt binaries are **not shipped in Git**. Deployment paths and network addresses must match the target system. See the [FIRST integration guide](official_first_integration/README.md), [Jetson setup](docs/jetson.md), and [map-building workflow](docs/global-planning.md).


### Run the geometry core on a CPU

Requirements: Linux, Python 3.10+, a C++17 compiler with OpenMP, and the packages in [requirements-core.txt](requirements-core.txt).

```bash
git clone https://arcusgit.vip.cpolar.cn/liuzhenghao/se2-nvblox-traversability.git
cd se2-nvblox-traversability

# Ubuntu / Debian build prerequisites
sudo apt-get install build-essential python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-core.txt

bash scripts/test_core.sh
python examples/narrow_passage.py
```

The example runs the production geometry classifier on synthetic corridors. A 0.506 m body fits the 0.506 m corridor with restricted headings; a 0.500 m corridor is rejected. Wider corridors admit larger heading intervals. These are geometric fixtures, not measurements from the demo.

The check script builds the native library and runs the focused geometry, graph, color, local-window, and floor-separation regressions, followed by the bag-management and robot-model unit tests. Hardware smoke scripts are separate and are not started by this command.

### Reconstruction and bag tools

On the existing M20 deployment, the tuning interface starts with:

```bash
bash /home/nvidia/scanplanner_test/se2_terrain_check/start_after_boot.sh
```

Select a bag, enable reconstruction, choose the sensor input, and open replay in RViz. The default reconstruction profile uses LiDAR with deskewing; replay uses the **recorded poses**, rather than rerunning odometry estimation. Use **Restore live reconstruction / 恢复实时重建** to return to live input.

For a bag with a built global map, click **全局路径规划**, then use RViz **Publish Point** on the destination floor. The start defaults to the recorded bag endpoint. See the [global planning guide](docs/global-planning.md).

A fresh machine also needs the ROS 2/nvblox stack, sensor drivers, odometry, calibrated transforms, and deployment services. Those external components are not installed by the CPU quick start. Follow the [Jetson integration guide](docs/jetson.md) and [bag workflow](bag_tools/README.md).

## How it works

```mermaid
flowchart LR
    S[Live sensors or bag] --> D[Deskew and pose / TF alignment]
    D --> N[nvblox TSDF and mesh]
    N --> B[Incremental BVH and multilevel heightfield]
    B --> C[Body fit, support, slope, step and headroom]
    C --> P[Feasible positions and heading intervals]
    P --> R[RViz cells and local pose graph]
    T[Parameter tuning] --> C
    P --> G[Saved multilevel map and ASA route]
    L[Live LiDAR and SuperLIO] --> A[Map localization]
    A --> F[Source local trajectory selection]
    G --> TERRA[Flat / upstairs / downstairs mode]
    TERRA --> F
    TERRA --> O
    L --> O[Deskew and live obstacle evidence]
    O --> F
    M[M20 measured height and gait] --> F
    F --> V[Freshness and command gates]
    V --> U[M20 command sender]
```

The classifier first uses conservative grid footprints to establish feasibility efficiently. Unresolved states are refined against the current mesh using the physical body footprint. Local graph nodes retain the actual feasible poses; refined edges check the swept footprint between poses. Changed geometry and parameters invalidate affected results. See [architecture and interfaces](docs/architecture.md).

| Display | Meaning |
| --- | --- |
| Green | All headings feasible with configured clearance margins |
| Deep → light orange | Increasing total feasible heading angle; physical fit may have insufficient lateral comfort margin |
| Blue | Historical geometry results, not current traversability permission |
| Rainbow mesh | Surface height, independent of traversability color |

## Validation

The current integrated chain covers cross-floor ASA routes, terrain-dependent local avoidance, gait transactions and command output. Component tests, software execution tests and field measurements are reported separately.

| Component | Latest recorded verification |
| --- | --- |
| Whole-map initialization | No initial pose supplied; tracking after **30.65 s**, followed by **66 valid tracking updates** on live Jetson inputs |
| Continuous-odometry recovery | **3.80 s** to recover, followed by **60 valid tracking updates** on live Jetson inputs |
| Localization regressions | 9 initialization/reference tests and 3 transport tests passed on both workstation and Jetson |
| Stair navigation | 51 source tests, 23 ASA tests and a 12-step ascent/descent software execution test passed |

The live localization measurements were stationary diagnostic runs with no motion output. Cross-floor integration is complete; these measurements do not establish reliability for every staircase or moving recovery case. [Validation scope and dated evidence →](docs/validation.md)

## Repository layout

| Path | Purpose |
| --- | --- |
| [`terrain_variants/local_window/`](terrain_variants/local_window/) | Current replay geometry core, pose graph, and display |
| [`terrain_variants/deskew/`](terrain_variants/deskew/) · [`fast_lidar/`](terrain_variants/fast_lidar/) | LiDAR motion compensation and Jetson CUDA conversion |
| [`bag_tools/`](bag_tools/) | Recording, reconstruction, tuning integration, lifecycle management |
| [`global_planner/`](global_planner/) | Complete-bag map building, yaw-layered ASA planning, and RViz 3D goals |
| [`official_first_integration/`](official_first_integration/) | Current FIRST-based execution, localization, telemetry, collision checks, tests and evidence |
| [`scanplanner_integration/`](scanplanner_integration/) | Shared localization, odometry and dense body-cloud integration |
| [`robot_model/`](robot_model/) | M20 joint-state adapter, namespaced TF, and upstream model |
| [`launch/`](launch/) · [`config/`](config/) | ROS 2 package launch and configuration |
| [`deployment/jetson/`](deployment/jetson/) | Existing deployment entry points and configuration snapshots |
| [`examples/`](examples/) · [`docs/`](docs/) | CPU example, integration notes, validation, and demo |

## Current limits

- Cross-floor navigation operates within a connected saved map. Saved-task cross-map actions (`PointInfo=4` / `MapID`) are outside the supported task interface.
- Repeated floors, insufficient observations, occlusion or changed geometry can prevent unambiguous localization. The system waits for valid matching and stable confirmation.
- Geometry depends on sensing, calibration, odometry and TSDF integration. Finite-resolution body/support checks do not model foot contacts or guarantee live hole/drop-off detection.
- Global paths use a saved mesh; live local avoidance handles observed obstacles. Changes to the environment can require rebuilding the saved map.
- Runtime latency depends on scene complexity and hardware load; command scheduling at 30 Hz is not a guaranteed minimum delivery rate.
- Raw field bags, generated maps, private credentials and prebuilt binaries are not distributed. Deployment paths, external services and CUDA dependencies must match the target system.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for the core checks and useful information to include in a bug report. Changes to collision or support behavior should include a geometry fixture that demonstrates the intended result.

## License and acknowledgments

Project code is licensed under [Apache 2.0](LICENSE). Third-party model assets and reference sources retain their own licenses; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

Built with [NVIDIA nvblox](https://github.com/nvidia-isaac/nvblox), [Isaac ROS nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox), and [ROS 2](https://github.com/ros2). M20 model assets come from [DeepRoboticsLab](https://github.com/DeepRoboticsLab/deep_robotics_model). This is an independent project and is not an official release by these organizations.
