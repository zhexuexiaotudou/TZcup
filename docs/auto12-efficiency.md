# AUTO-12：3500 m²/h 清扫效率重构

## 结论

AUTO-12 的软件与离线动力学机器门通过。自动设计搜索选择 `1.32 m`
展开清扫宽度、`1.0 m/s` 清扫速度、`1.0 m/s²` 加速度和
`1.2 m/s²` 紧急减速度。理论上限为 `4752 m²/h`，仅用于 smoke
筛选；正式结论来自 10 次带加减速、转弯、正常避障等待、任务内
staging 和栅格扫掠验证的时间步进任务。

正式矩阵的平均有效效率为 `4205.81 m²/h`，95% 置信区间下界为
`4193.52 m²/h`，单次最低为 `4181.12 m²/h`。最低经验覆盖率为
`1.0`，最大漏扫率为 `0`，最大重复率为 `0.05303`，最大轨迹 XY
RMSE 为 `0.03749 m`；碰撞和 keepout 违规均为 0，10/10 任务均以
刷盘关闭结束。

证据等级是
`OFFLINE_TIME_STEP_DYNAMICS_AND_RASTER_SIMULATION`。这些数字不是
Gazebo 或实车实测，不得用于声称真实车辆已经达到竞赛效率。

## 同步设计

候选是 opt-in profile，既有 `0.65 m` 生产默认不变。

- 物理模型：xacro 暴露 `cleaning_width` 和 `brush_center_y`，候选值为
  `1.32/0.52 m`，刷盘的 visual 与 collision 同步移动。
- 清扫 footprint：宽 `1.32 m`，中心在底盘前方 `0.55 m`。
- 碰撞与 costmap：local/global footprint 均为
  `[[0.72,0.66],[0.72,-0.66],[-0.58,-0.66],[-0.58,0.66]]`；
  Collision Monitor 消费 local costmap 发布的同一 footprint。
- 动力学：清扫速度 `1.0 m/s`，加/减速度 `1.0/1.2 m/s²`，最小转弯
  半径 `0.75 m`；计算制动距离 `0.5667 m`，小于 `0.65 m` 安全包络。
- Coverage：作业宽度 `1.32 m`、swath spacing `1.2276 m`、headland
  `1.80 m`、Boustrophedon 连续路径。
- 能耗：显式记录驱动、刷组、计算/辅机、速度和加速储备模型，每次
  正式运行输出能量和峰值功率。
- 安全：扩展状态未知时 fail-closed，停止、超时和退出均要求刷盘关闭；
  该静态审计对应 AUTO-02 等价安全回归，不替代新的 Gazebo 动态回归。

## 复现

```powershell
py -3 scripts/auto12_efficiency_formal.py `
  --output artifacts/autonomous_auto12_20260730_evidence `
  --implementation-commit 41ae6e0a4e59c785a7f9a397eab939ca1ee9246d
py -3 scripts/ci_fast.py
```

部署候选时必须同时选择：

```text
sim.launch.py cleaning_width:=1.32 brush_center_y:=0.52
navigation.launch.py params_file:=.../nav2_auto12.yaml
coverage.launch.py footprint_profile:=auto12_efficiency_v1
                   params_file:=.../coverage_auto12.yaml
coverage_probe -- mission config demo_area_auto12.yaml
```

机器证据位于
[`artifacts/autonomous_auto12_20260730_evidence/`](../artifacts/autonomous_auto12_20260730_evidence/)。
