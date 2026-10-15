# Third-party notices

The root Apache-2.0 license covers this project's original code. The following included assets and reference sources retain their existing copyright notices and license terms.

| Component | Included location | Source and license |
| --- | --- | --- |
| DeepRobotics M20 URDF and meshes | `robot_model/upstream/` | [deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model), commit `75824b516ecc8fa3f8f7ce5d6577e1e86d6b614f`; [BSD-3-Clause](robot_model/upstream/LICENSE.txt), © 2024 DeepRoboticsLab |
| NVIDIA Isaac ROS nvblox reference files and adapted LiDAR integration | `terrain_variants/multi_lidar/` | [isaac_ros_nvblox, release-3.2](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/tree/release-3.2); [Apache-2.0](terrain_variants/multi_lidar/reference/LICENSE), NVIDIA copyright notices retained in source |
| NVIDIA nvblox weighting implementation reference | `terrain_variants/weighting/weighting_function_impl.h` | [nvblox](https://github.com/nvidia-isaac/nvblox); Apache-2.0, © 2022–2023 NVIDIA CORPORATION |
| RoboSense AIRY / mechanical decoder reference headers | `terrain_variants/multi_lidar/reference/decoder_*.hpp` | [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver); BSD-3-Clause terms and RoboSense copyright retained in each header |

The model's source manifest includes file hashes in [`robot_model/source_manifest.json`](robot_model/source_manifest.json). The Apache license stored in the multi-LiDAR reference directory applies to NVIDIA files; it does not replace the BSD license embedded in the RoboSense headers.

ROS 2, nvblox/Isaac ROS, RealSense drivers, SuperLIO, and vendor hardware interfaces are separate dependencies governed by their own upstream terms. No private vendor SDK headers, user-provided protocol documents, or prebuilt deployment libraries are included in this publication.

README organization was informed by the official [Meta SAM 2](https://github.com/facebookresearch/sam2), [NVIDIA Warp](https://github.com/NVIDIA/warp), and [Google DeepMind MuJoCo](https://github.com/google-deepmind/mujoco) repositories: concise purpose, visible demos, executable quick starts, and explicit technical scope. Their text, branding, and project claims were not copied. These references do not imply affiliation or endorsement.

The global planner independently implements concepts from [SE(2) NavMesh](https://se2-navmesh.github.io/) and its [paper](https://se2-navmesh.github.io/static/SE2_anonymous.pdf). No author source code or paper assets are bundled. See [algorithm correspondence and adaptations](docs/global-planning.md).

The optional ScanPlanner workspace is downloaded from [wuyi2121/SCAN-Planner, ros2-community](https://github.com/wuyi2121/SCAN-Planner/tree/ros2-community) at `d0b921c9b05a6d291d144d60882b2e0e88d2c0e0` (Apache-2.0). This is a community ROS 2 port. Its license and notices remain in the downloaded checkout. The project ships a small integration patch, not the full upstream workspace.

The `official_first_integration/` provides the source local-navigation backend, terrain-dependent perception, task/gait transactions and M20 integration. Separately supplied proprietary trajectory/collision archives, vendor executables and generated reference maps are not distributed in this repository. The asset manifest and import utility verify a supplied local archive; their presence does not grant redistribution rights to that archive. See the [runtime and validation scope](official_first_integration/README.md).
