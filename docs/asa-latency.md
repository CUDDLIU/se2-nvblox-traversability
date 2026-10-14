# ASA query latency / ASA 规划延迟

Measured on the deployed Jetson, 2026-10-13, using
`scanplanner_integration/maps/20261012_confirmed_scene`. These are offline
`Planner.query()` wall times with the live navigation processes running;
they include cleanup/cache writes but exclude startup and RViz delivery.

| Case / 场景 | Before / 优化前 | First validation / 首次验证 | After process restart with certificates / 重启后复用验证结果 |
|---|---:|---:|---:|
| Previously successful 10.23 m route / 原成功路线 | 27.6 s (profiled) | 1.52 s | 0.56 s |
| Current goal, 27.86 m route from a valid reference start / 当前目标、有效参考起点 | — | 2.93 s | 0.91 s |
| Actual rejected start / 实际被拒绝的起点 | 32.6 s (live log) | 0.04 s | 0.04 s |

The valid reference start is `[-0.0281905, -0.2933251, -0.57, -0.0740523]`.
The actual rejected start was `[-0.0339666, -0.4082134, -0.5642628, -0.0363769]`.
The long-route benchmark does **not** assert that the actual robot start is valid.
No movement commands were published by these benchmarks. Original route costs
were preserved in the measured comparisons.

## Changes

- Prepare the immutable forward/reverse graph before reporting `ready`.
- Search the static graph plus a small endpoint overlay directly in C++, avoiding
  repeated sorting, copying and transposing millions of edges.
- Stop reverse Dijkstra when the start is settled; cap remaining distances at
  this bound. The heuristic remains admissible after collision rejections.
- Validate usable route sections in the same lazy-check pass; reuse identical
  pose/edge certificates across the two ASA stages and final verification.
- Store only static-map certificates in `<map>/runtime/asa_static_checks.npz`.
  Mesh identity, configuration, pose coordinates and checker code/binary are
  fingerprinted; changed inputs invalidate the cache. Query endpoints and the
  robot's previous parking position are not persisted in this cache.
- Reject invalid exact endpoint poses before searching, and cancel obsolete
  queries when a newer goal arrives.

Tests cover flat ground, physical-width passages, separate floors, stairs,
invalid-start rejection, cancellation cleanup, cache reload/invalidation and
native multi-graph shortest-path equivalence against SciPy on 20 directed graphs
with collision rejections.

## Limits / 限制

Startup still loads the map and precomputes the graph (about 12 s each in this
measurement). Wait for `ready` before assessing click latency. A new, unvalidated
route can still take several seconds; subsecond response is not guaranteed for
all goals or hardware loads. RViz delivery adds scheduling/rendering latency.
The ASA stages, physical body footprint and support/collision constraints remain
enabled. Local FIRST collision checking continues to use live sensor data.

固定地图搜索图在启动时准备，点选时只接入起终点；已验证的固定地图几何结果会保存，
重启后仍可复用，地图、配置或检查算法改变时自动失效。新目标会取消旧查询。
当时实际起点的朝向未通过保存地图的机身轮廓/支撑检查，严格查询约 0.04 秒即可明确反馈。
后续实时查询已增加就近接入：选择同层附近通过检查的地图位姿，由 FIRST 使用实时点云先接入，
再跟踪 ASA 路线。全局检查仍作用于接入点及其后的路线，不覆盖地图外的接入过程。
首次遇到的复杂路线仍可能耗时数秒，不能据此宣称任意路线都可瞬时生成。
