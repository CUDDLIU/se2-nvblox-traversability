# Experimental dual-origin LiDAR integration

This optional adapter splits the M20 merged point cloud into front/rear sensor groups and integrates rays from separate physical origins. It is **not enabled by the default GUI replay profile**.

`split_cloud.hpp` preserves world measurement endpoints while transforming each group into its sensor projection frame. The M20 dataset convention uses 96 rings per sensor. The native-axis and optical-origin options are separate experiments; they are specific to the verified input format, not generic RoboSense driver replacements.

Build with `bash build.sh` on the configured Jetson. The adapter links installed nvblox headers/libraries and records their digest. `benchmark.py --multi-lidar` requires explicit LiDAR input, `--deskew`, and `--fast-lidar`; inspect `--help` before using the integration benchmark. It operates a real ROS deployment, not the CPU example.

The benchmark and diagnostic tools were used with private field bags and saved sensor samples, which are not shipped. These experiments did not establish improved route connectivity and were not promoted to the default. The scan still approximates ray origins at a reference frame time; this is not per-point continuous-time ray integration.

## 中文

本目录是独立双雷达原点实验，不是当前默认方案。它只在 nvblox 内部修改射线投射坐标，不修改原始 bag、公共 TF、厂商驱动或 LIO。双原点、物理扫描轴和轴向光学偏移分别改变不同条件，应独立比较。旧实验的速度不能代表当前精细通行性判定性能。

NVIDIA reference files retain Apache-2.0 notices; RoboSense decoder headers retain their embedded BSD-3-Clause notices. See [third-party notices](../../THIRD_PARTY_NOTICES.md).
