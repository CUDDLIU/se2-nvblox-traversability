# RViz 黑屏排查与恢复（2026-10-08）

本次现象是菜单和显示列表正常、Global Status 为 OK，但整个三维区域全黑，连参考网格和配置的背景色都没有显示。

## 排查结果

- 实时地图定位状态持续为 `tracking`；使用同一套话题和显示配置新开的 RViz 能显示地图和点云。
- Jetson 本机截图也能复现黑屏，因此不只是远程客户端画面传输的问题。
- 当前 X11 桌面为 1600×900；原配置的窗口高度为 900，超过扣除桌面面板、标题栏后的可用高度。
- 以原尺寸重新启动仍出现黑屏；将初始窗口改成 1000×700 并新建窗口后恢复。最终采用的 1280×720 也已实际验证。
- 对已经黑屏的窗口，仅调整大小或重新加载配置没有恢复，必须重建 RViz 窗口。

以上对照将故障缩小到与初始窗口尺寸相关的 RViz 渲染窗口初始化问题。尚未确认 Qt、OGRE 或 NVIDIA 驱动内部的具体缺陷；本次没有更换驱动，也没有切换到 CPU 软件渲染。

## 已部署的处理

1. `scanplanner_integration/scan.rviz` 的默认窗口改为 1280×720，位置为 X=100、Y=70。启动时生成的 `live.rviz` 会继承该设置。
2. `run.py` 支持通过 SIGUSR1 重建其管理的 RViz 子进程，保持 SuperLIO、定位、规划及控制进程运行，继续使用本会话的 RViz 配置。
3. 增加恢复命令：

```bash
bash /home/nvidia/scanplanner_test/se2_nvblox_surface_shadow/scanplanner_integration/restart_rviz.sh
```

这是显示窗口的恢复命令，不是停止机器人命令。原有启动、停止命令不变，项目根目录 README 仍只保留这两个命令。

## 实机验证

验证会话：`scanplanner_integration/runtime/20261008_121215`。

- 恢复后已截图确认网格、可通行图、实时雷达点云显示正常。
- 两次通过新命令重建 RViz，均仅 RViz PID 改变，其余本会话子进程 PID 保持不变。
- 地图定位持续为 `tracking`，验证期间未发送非零机器人速度指令；这次检查不代表完成行走测试。
- 黑屏、对照实验、恢复后的截图及 `validation.json` 保存在 Jetson 项目内的 `scanplanner_integration/runtime/rviz_black_20261008/`。
