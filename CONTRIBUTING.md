# Contributing

Start with the CPU quick start in [README.md](README.md) or [中文说明](README.zh-CN.md). Run `bash scripts/test_core.sh` before proposing changes to the maintained core. This command is independent of ROS and hardware.

For a geometry bug, include the body dimensions, margins, resolution, expected feasible poses, and a small synthetic mesh fixture. For replay issues, include the input mode, package versions, relevant topic rates, transform availability, and sanitized error output. Field bags are optional; avoid including credentials or unrelated recordings.

Keep `terrain_variants/local_window` changes distinct from optional reconstruction experiments. A visual connection is not sufficient evidence of graph connectivity: test actual feasible poses and transitions. Performance reports should distinguish input, mesh, and traversability rates and state which latency interval is measured.

The `smoke_*.py` scripts operate the configured robot deployment and may start or stop services and create test recordings. They are manual integration checks, not CPU unit tests. Use them only on a deployment you control.

Update both root READMEs when changing user-visible behavior. Open a pull request with the concrete behavior change and the checks you ran.

## 中文

提交前运行 `bash scripts/test_core.sh`。几何问题请附带机身尺寸、余量、分辨率和最小构造场景；回放问题请说明输入模式、依赖版本、话题频率、TF 状态和必要日志。

碰撞与连通性修改应验证实际可行位姿及过渡过程。性能报告应区分输入、Mesh 和通行性频率，并说明延迟的起止点。`smoke_*.py` 会操作已配置机器上的服务与测试录制，请勿作为普通单元测试批量执行。
