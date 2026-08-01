# Gazebo 原生清扫控制与三档场景

## 从哪里控制清扫

运行下面的 Windows PowerShell 命令后，Gazebo 右侧固定显示“清扫任务控制”卡片：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_gazebo_cleaning_demo.ps1 -MapSize small
```

| 按钮 | 作用 | 安全行为 |
|---|---|---|
| 开始 | 从 `READY` 进入规划、转场和清扫 | 只调用 `/coverage/control/start`，不直接控制底盘 |
| 暂停 | 暂停当前清扫任务 | 关闭刷盘、取消当前 Nav2 goal，状态进入 `PAUSED` |
| 继续 | 从暂停位置续接当前任务段 | 重新提交当前 Nav2 goal，不把暂停计为导航重试 |
| 停止任务 | 结束本次任务，保留 Gazebo 供检查 | 关闭刷盘并生成带 `stopped_by_operator` 的报告 |
| 关闭 Gazebo | 先停止任务，再关闭仿真窗口 | 不留下仍在运动的任务 |

Gazebo 自带的播放/暂停属于“仿真物理控制”，它会冻结整个世界，并不是清扫任务暂停。任务完整状态链为：

```text
READY → STARTING → PLANNING → TRANSIT_PREFLIGHT → TRANSIT → ALIGNING
→ EXECUTING_SWATH / EXECUTING_TURN → COMPLETED
```

## 三档地图

| 参数 | 场景尺寸 | 用途 | 主要内容 |
|---|---:|---|---|
| `small` | 30 m × 20 m | 快速演示完整清扫 | 服务中心、道路、人行道、绿化、斑马线、公交站、停车位、树木、路灯、垃圾桶和五类清扫目标 |
| `medium` | 80 m × 50 m | 中等规模联调和展示 | 园区建筑、服务建筑、道路设施、停车区、绿化和更密集街具 |
| `large` | 200 m × 100 m = 20,000 m² | 赛题正式建图尺度 | 精确 20,000 m² 场地、分区建筑、尺度标尺、长道路和完整园区要素 |

```powershell
.\scripts\run_gazebo_cleaning_demo.ps1 -MapSize small
.\scripts\run_gazebo_cleaning_demo.ps1 -MapSize medium
.\scripts\run_gazebo_cleaning_demo.ps1 -MapSize large
```

三张 SDF 只使用本地基础几何，不在线下载模型。`large` 的物理地面尺寸严格为
200 m × 100 m。普通 `-MapSize large` 仍只改变物理场景；需要把完整大图栅格、
AUTO-12 车辆参数和代表性分区任务接入同一条运行链时，使用：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_visual_demo.ps1 `
  -CompetitionProfile -GazeboOnly -ManualControl -KeepOpen
```

该配置加载 20,000 m² 地图和 20 分区，但现场只派发一个 108 m² 分区，不能写成
“20,000 m² 全场清扫已经通过”。正式全场多分区调度、耐久和效率仍须另行验收，
详细边界见 [`competition-gazebo-profile.md`](competition-gazebo-profile.md)。

场景由 `scripts/generate_gazebo_world_variants.py` 确定性生成，结果保存在 `sanitation_worlds/worlds/sanitation_campus_{small,medium,large}.sdf`。

## WSLg 的 3D Scene 黑屏

WSLg 中 Mesa D3D12 能通过 `glxinfo -B` 并不等于 Ogre2 视口一定可见：实测故障会保留
完整 Qt 外壳、World Control 和清扫控制卡，但 `3D Scene` 像素全黑。三档任务配置现在补齐
`GzSceneManager`、交互视图、相机跟踪、Marker 和实体选择插件；Windows 默认参数
`-GazeboGuiRenderer auto` 在 WSLg 上让 Gazebo 服务端与传感器继续使用 D3D12/NVIDIA，
只让 GUI 使用 X11/llvmpipe。可用 `d3d12` 或 `software` 显式覆盖，仅建议用于诊断。

GUI 原生控制节点加载后，`gazebo_viewport_probe.py` 会通过 X11 捕获 Gazebo 窗口，只分析
左侧中央 3D 区域并记录 `gazebo_viewport.png`、`gazebo_viewport_probe.json`。若该区域仍为
纯黑，启动器以退出码 `8` 停止整条运行链，不把窗口响应、ROS READY 或 GPU 名称当作画面验收。

## WSLg 的 COPY MODE

本机使用的 WSL 2.7.3 / WSLg 1.0.73 存在 RemoteApp 共享内存初始化缺陷：
`/mnt/shared_memory` 缺失时，应用仍会产生任务栏图标，但窗口没有可交互表面并显示
`[WARN:COPY MODE]`。`run_visual_demo.ps1` 现在先以 root 幂等执行
`prepare_wslg_runtime.sh`，挂载 `tmpfs`、写入专用发行版的 `/etc/fstab` 并验证可写性，随后等待
两秒再启动 Gazebo，避免触发该故障。
若当前 WSLg 日志已经出现共享内存分配错误，且没有其他发行版正在运行，启动器会执行一次
完整 `wsl --shutdown` 并重新预检；若检测到其他活动发行版则明确停止，避免中断无关工作。

启动器还会同时运行 `wslg_window_guard.ps1`。守护器只匹配唯一的 Gazebo RemoteApp 窗口，
恢复异常最小化或隐藏的窗口；若 COPY MODE 持续出现或同时存在多个 Gazebo 窗口，则写入
`wslg_window_guard.failed` 并让 Linux 任务监督器停止整条运行链，避免留下不可交互的后台会话。

关闭 Gazebo GUI 后，Linux 侧任务监督器会在任务运行期间检测精确 GUI PID，给 Coverage
一个短暂的安全停止窗口，然后终止本次任务和全部子进程并释放 `8877`。证据目录中的
`wslg_window_guard.jsonl` 记录窗口恢复事件；任务未完成时关闭 GUI 会写入
`launcher_termination.json`，状态为 `OPERATOR_GUI_CLOSED`，不得误写成清扫完成。
Windows 守护器同时保留已经确认过的 Gazebo 窗口句柄；若 Linux GUI 已退出而 WSLg
RemoteApp 只剩无标题黑色外壳，守护器会仅向该已跟踪句柄发送 `WM_CLOSE`，避免任务栏留下
可点击但没有渲染内容的残留窗口，不会终止整个 `msrdc` 或其他 WSLg 应用。

COPY MODE 本身来自 WSLg RemoteApp，不是 ROS 或 Gazebo 任务故障；它不再被解释成需要
人工按 `Esc` 的普通状态。若预检后仍出现 COPY MODE，启动器会明确失败并保留诊断证据。
若 WSLg 重启后的 Gazebo GUI 在原生任务控制加载前提前退出，Windows 启动器会将它与
COPY MODE 一样视为可恢复的 WSLg 冷启动故障，最多再执行一次安全重启和同参数重试；第二次
仍失败时立即停止并返回错误，不进行无限重启。
