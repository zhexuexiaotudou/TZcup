# 20,000 m² Gazebo 地图档位

## 用途与限制

`CompetitionProfile` 提供 `200 m × 100 m = 20,000 m²` 的完整仿真地图、20 个可审计分区和代表性现场任务。它同时提供产品默认 Ackermann 与显式 legacy 驱动入口，但单区运行只验证大地图上的车辆、地图和任务链兼容性，不能作为全场 V1 验收证据。

Ackermann 大地图兼容性运行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_visual_demo.ps1 `
  -CompetitionProfile -DriveModel ackermann -CoverageProfile ackermann `
  -NoGui -NoRviz -NoMcap -Video off
```

legacy 资产演示：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_visual_demo.ps1 `
  -CompetitionProfile -DriveModel skid_steer_legacy -CoverageProfile optimized `
  -GazeboOnly -ManualControl -KeepOpen
```

生成器为两个入口输出独立任务配置。Ackermann 配置冻结 `1.32 m` 刷宽、`1.429352 m` 物理最小半径、真实 footprint 与外部 turning apron；legacy 配置不进入产品裁决。正式 V1 仍需连续多区调度、保存/加载/重定位、Nav2 规划闭环和全场运行证据。

## 代表性分区

现场任务限定在 `Z01_00` 的 `12 m × 9 m = 108 m²` 分区，以便验证规划、转场、清扫、转弯、暂停续扫和完成过程。Ackermann 报告必须标记为 `representative_ackermann_zone_on_full_competition_map`，不得描述为全场耐久、完整建图或全地图任务通过。

当前冻结的兼容性配置使用 `1.12 m` 中心线间距、`3.0 m` 刷盘关闭前导/出口段、`0.60 m/s` 清扫速度和 `0.25 m/s` Dubins 曲率段速度。单区实测 coverage `1.0`、repeat `0.1403`、直线度 P95 `0.0205 m`、横向误差 P95 `0.0572 m`，碰撞与禁区侵入为 `0`；定位 P95 `0.0618 m` 和净效率 `273.9 m²/h` 未过产品门，因此总体状态保持失败。

## 产品准入仍需补齐

- Ackermann 在完整 20,000 m² 地图上的建图、存图、加载、重定位和导航闭环。
- 多分区连续调度、30 种子导航/覆盖/动态障碍矩阵及 2 小时耐久。
- 覆盖率不低于 95%、重复覆盖率不高于 20%、零碰撞与零越界。
- 包含规划、转场、避障、回充、暂停、点清扫等全部时间的净效率不低于 3500 m²/h。
- 五类垃圾感知、点清扫、清后复检、故障注入和确定性回放的完整证据。

在这些证据齐备前，该档位只能证明场景资产和可视化链路存在。
