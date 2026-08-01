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

三张 SDF 只使用本地基础几何，不在线下载模型。`large` 的物理地面尺寸严格为 200 m × 100 m；当前自动清扫仍使用已验证的小区域或 17 段 Coverage 任务，不能把“场地尺寸正确”写成“20,000 m² 全场清扫已经通过”。正式全场建图、分区调度和全场效率仍须另行验收。

场景由 `scripts/generate_gazebo_world_variants.py` 确定性生成，结果保存在 `sanitation_worlds/worlds/sanitation_campus_{small,medium,large}.sdf`。

## WSLg 的 COPY MODE

若标题出现 `[WARN:COPY MODE]`，单击 Gazebo 窗口并按一次 `Esc` 退出 WSLg 复制模式。它不是 ROS 或 Gazebo 任务故障；退出后右侧按钮即可正常接收鼠标操作。
