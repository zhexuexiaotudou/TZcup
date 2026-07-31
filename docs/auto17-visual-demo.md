# AUTO-17：从“有仿真结果”到“看得见清扫过程”

## 目标

AUTO-17 解决的是展示与可观察性缺口：以前可以从 JSON、日志和 MCAP 判断任务是否成功，但普通观众难以看到车辆何时启动、正在清扫哪一条带、刷盘是否开启、当前位置和覆盖进度。该层直接复用已通过的 AUTO-02 正式导航与 Coverage 链，不制造动画替代仿真。

## 一条命令启动

前提是本机已有 `TZcup-Ubuntu-24.04`、ROS 2 Jazzy、Gazebo Harmonic、WSLg 以及 `$HOME/sanitation_ws` 基础工作空间。Windows Python 需要 Pillow；本机已经满足。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_visual_demo.ps1 -Video on
```

常用参数：`-SkipBuild` 复用已构建 overlay，`-Video off` 关闭 MP4，`-NoRviz` 关闭 RViz，`-KeepOpen` 在任务结束后保留窗口，`-OutputDirectory <path>` 指定证据目录。默认看板地址是 `http://127.0.0.1:8877`。

## 屏幕上会看到什么

1. Gazebo 展示三维场景与清扫车，`/gui/track` 请求让相机跟随 `sanitation_vehicle`。
2. RViz 以 `base_footprint` 为跟随目标，叠加地图、机器人模型、激光、当前覆盖路径、Nav2 路径、局部/全局代价地图与 footprint。
3. 浏览器看板显示任务状态、17 个组件进度、融合位姿、速度、刷盘、急停、当前规划、evaluation-only 实际轨迹、刷盘开启轨迹和事件时间线。
4. 专用视频渲染器只读取 `/api/v1/telemetry`，生成与看板相同语义的 MP4；它不录制整个 Windows 桌面，也不接触执行器。

## 数据流和安全边界

```text
Gazebo -> ROS-Gazebo bridge -> Stage4V mixed localization -> Nav2 -> Coverage
                                      |                    |         |
                                      +------ ROS topics --+---------+
                                                            |
                                             live dashboard (read-only)
                                               |       |        |
                                             browser  JSON     MP4
```

`/ground_truth/*` 和 `/coverage/evaluation_sample` 只用于轨迹绘制、经验覆盖率与定位误差评价。规划、控制、碰撞监测和安全门只使用可部署传感器与融合估计。网页只提供 GET 接口，不发布 ROS 命令。

## 证据目录

每次运行产生独立目录，主要文件如下：

- `acceptance_summary.json`：fail-closed 总门；任一必要证据缺失即 FAIL。
- `coverage_report.json`、`coverage_path.json`、`coverage_trajectory.csv`：正式 Coverage 结果。
- `dashboard_telemetry.json`：看板终态快照与 claim boundary。
- `visual_demo_bag/`：MCAP 与元数据。
- `visual_demo.mp4`、`visual_demo_frame.png`：专用可视化录像与代表帧。
- `localization.log`、`navigation.log`、`coverage_server.log`、`rviz.log`：组件日志。

2026-07-31 本机验收结果为 17/17、经验覆盖率 93.67%、0 碰撞、0 禁行区违规、定位 XY RMSE 3.59 cm、205528 条 MCAP 消息/18 个话题，终态 `COMPLETED`；单命令启动器完整返回 0。

仓库内紧凑证据见 [`artifacts/auto17_visual_demo_20260731_evidence/`](../artifacts/auto17_visual_demo_20260731_evidence/)；原始 MCAP、MP4、逐帧图像和完整日志不提交 Git。

## 结论边界

AUTO-17 证明真实 Gazebo/Nav2/Coverage 过程可以一键启动、现场观察和审计。它不证明学习感知泛化、真实道路数据、Horizon J6 板端运行或综合竞赛矩阵通过；对应状态必须继续保持 false，直到各自正式门有独立证据。
