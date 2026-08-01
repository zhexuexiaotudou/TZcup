# 竞赛仿真配置与剩余差距

## 当前交付边界

`CompetitionProfile` 把原先分离的比赛尺度地图、AUTO-12 清扫机构参数和
Gazebo 原生任务控制串成同一条可运行链：

- Gazebo 世界和 Nav2 栅格均为 `200 m × 100 m = 20,000 m²`；
- 地图按 `10 × 2` 划分为 20 个可审计分区；
- 车辆使用 `1.32 m` 展开清扫宽度、`0.52 m` 刷盘中心和 `1.0 m/s`
  候选上限；
- Gazebo 页面显示完整地图面积、当前现场演示范围、任务状态、任务段进度和
  刷盘状态，并保留开始、暂停、继续、停止和关闭按钮；
- 现场运行 `Z01_00` 内的 `12 m × 9 m = 108 m²` 代表性分区，便于在评审时间内
  看完规划、转场、清扫、转弯、暂停续扫和完成过程。

启动命令：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_visual_demo.ps1 `
  -CompetitionProfile -GazeboOnly -ManualControl -KeepOpen
```

运行时会在证据目录生成 `competition_profile_manifest.json`、完整占据栅格、
keepout/speed mask 和现场任务配置。2 MB 原始栅格不提交 Git。

## 为什么现场只跑一个分区

以 AUTO-12 离线均值 `4205.8 m²/h` 估算，纯清扫完整 20,000 m² 仍需约
4.75 小时，现场从启动到结束全程观看不现实。代表性分区不是“缩小比赛地图”：
完整地图仍由 map server 加载，车辆也在该地图坐标系中定位；只把本次派发任务
限定为其中一个可重复验证的分区。任何报告都必须使用
`LIVE_REPRESENTATIVE_ZONE_ON_FULL_SCALE_MAP`，不得写成全场耐久通过。

## 距离最终比赛作品的差距

| 能力 | 当前状态 | 还需完成 |
|---|---|---|
| 比赛尺度地图与分区 | 仿真完整接入 | 增加全场多分区连续调度与约 5 小时耐久证据 |
| 清扫效率候选 | 离线通过，现场参数接入 | 在当前 Gazebo 动力学中做多种子有效面积/小时实测 |
| 动态避障与安全 | 现有 Nav2/碰撞监控链 | 在竞赛大图内注入行人/车辆并形成零碰撞回归矩阵 |
| 五类垃圾与点清扫 | 场景有五类目标外观 | 学习检测、决策和真实点清扫闭环仍未集成 |
| 定位精度 | 小场景有仿真证据 | 大图多区域、多种子和失锁恢复的在线统计 |
| 实车与 J6 | 未通过 | 真实数据、实车安全试验、J6 工具链和目标设备运行 |

因此本配置提升的是“比赛尺度的可信现场演示”和“人类可监督性”，不会将
`SIMULATION_COMPETITION_MATRIX_PASS`、`REAL_DOMAIN_PASS`、`J6_TOOLCHAIN_PASS`
或 `FINAL_COMPETITION_EVIDENCE_COMPLETE` 改为 `true`。
