# TZcup 三项比赛硬指标：仿真最小闭环

2026-09-13，范围 `COMPETITION_HARD_MINIMUM`。本轮最终运行 `steady-02` 三项通过；这是受控直线 Gazebo 试验，不是实车、整任务平均效率或完整比赛验收。

| 硬指标 | 实测结果 | 判定 |
|---|---:|---|
| 有效刷宽 ≥600 mm | 最大连续清除宽度 600 mm | PASS |
| 稳态清扫效率 ≥3500 m²/h | 4320 m²/h | PASS |
| 行驶中完整急停 ≤1 s | 0.489 s（仿真时间） | PASS |

## 根因与最小修复

左右侧刷升降方向、100 mm 行程、关节名称与转速控制均正常。实际侧刷碰撞圆柱中心位于刷轴下方 65 mm，厚度 26 mm，因此底端到刷轴距离为 78 mm。GroundDirtCleaningSystem 和 Xacro 却使用 83 mm，导致真实工作姿态被计算为 −5 mm clearance，低于保留不变的 −4 mm 下限，错误地报告两侧刷 not ready。

将这两处参数改为 78 mm；没有改变几何、升降行程、接触容差或栅格。插件增加仿真时间和已清除单元坐标遥测，以便按真实状态复算面积并集。

接触采集另有两项试验配置缺口：继承的校园 world 未加载 `gz-sim-contact-system`；接触桥接需要完整 scoped sensor source，再 remap 到 ROS 短话题。新试验 world 补齐 Contact 系统后，三刷均得到与 `ground_plane::link::collision` 的真实接触消息。没有修改定位、感知或全局场景生成器。

## 固定窗口、面积与宽度

试验污渍带为 22 m × 2 m，4400 个互不重叠的 100 mm × 100 mm 单元。原始单元坐标在 `fixture.json`，未调整网格以追求通过。控制器仅在三刷 ready 且三个接触源均新鲜非空后开始运动。

运动开始：39.051 s；预先约定前 5 s 为加速段，此后只计量 10 s。实际计量窗口 **44.051–54.051 s**，清除计数从 **606 到 1806**，新增单元集合恰为 **1200 格**，并集面积 **12.00 m²**。效率为 `12.00 / 10.00 × 3600 = 4320 m²/h`，没有使用速度乘配置刷宽替代栅格计量。窗内 GT 线速度范围 0.983746–1.000974 m/s。

中央连续清除带为 600 mm；两侧各有分离的清除带，总横向跨度为 1400 mm，但跨度包含未清除空隙，不能宣称 1400 mm 连续刷宽。宽度证据分辨率为 100 mm，没有亚栅格测量精度声明。

## 完整急停

触发前 GT 线速度 **1.000026 m/s**，角速度 **0 rad/s**。触发时刻 **54.051 s**；GT 源时间戳 **54.540 s** 起，三维线速度模与三维角速度模均不超过 **0.01 m/s、0.01 rad/s**，并持续保持 **1.5 s**。仿真延迟 **0.489 s**，PASS。

同段墙钟延迟 **4.809955 s**，单独列示，不作为仿真时间通过值。触发使用最近的物理仿真状态时间戳，停止使用 GT 消息源时间戳；触发时间戳的小幅滞后只会使延迟估计偏保守。

## 运行、失败留存与复核

- `probe-01`：完整观察 60 个仿真秒，清除 0.92 m²，连续宽度 600 mm；低速完整急停 0.279 s。退出码 0、MCAP 正常封口，但接触遥测缺失，不能独立作为三刷真实接触通过证据。
- `steady-01`：三刷接触预检被阻止，始终未开始运动和稳态窗口。定位到 world 缺少 Contact 系统后停止；退出码 1，MCAP 正常封口。中断时 rclpy context 已失效，finally 中发布失败，未生成 timeline；原始 bag、错误和缺失状态全部保留。
- `steady-02`：真实接触预检后执行唯一一次稳态窗口，三项 PASS；退出码 0，MCAP 和 timeline 正常封口。
- 独立读取最终 MCAP 到末尾，复算新增清除集合仍为 1200 格。bag 中左右侧刷/主滚刷非空接触消息分别为 18823、18280、18365 条，均记录到真实接触。
- 聚焦验证：地污/几何/升降 43 项通过，计量与 fixture 3 项通过，共 46 项；Python 编译和实际 Bash runner 语法检查通过。只重建 `sanitation_gazebo_control`、`sanitation_vehicle_description` 两包，构建成功。

## 提交、证据与范围

物理计量修复 `86286fb`；初始记录器 `fb82010`；完整接触端点及运动前接触门 `e4deec4`；可复现 Contact fixture 与回归测试 `40a24e3`。所有代码位于用户指定隔离 worktree `F:\Project\TZcup\.workspace\worktrees\TZcup-simulation-only-completion`，未修改根脏工作区。

云卡新证据根：
```
/workspace/tzcup-competition-sim-only-20260912/evidence/width-efficiency-estop-20260913-01
```
其宿主位置为 `/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/width-efficiency-estop-20260913-01`。本地副本位于隔离 worktree 的 `.work/width-efficiency-estop-20260913-01/evidence`。`source_binding.json` 绑定完整提交与受影响包源码归档；`bag_verification.json` 绑定原始 MCAP 复算；`run_integrity.json` 保留成功/失败状态。

最终资源检查时间：2026-09-13 07:10:51 UTC（北京时间 15:10:51），无本任务 partition 残留进程，Gazebo 锁探针 rc=0，**SIM_RESOURCE_RELEASED=true**。实例未销毁，全部旧证据保留。

按本任务明确范围未做 PR、CI 全套、合并、部署、三次 Golden、长稳、正式 HBM、20000 m² 地图或定位/感知修改。neat-freak 已核对 README 和相关专题边界，结果写入本专题文档；不修改全局记忆。工作区、分支与证据保留，等待用户确认后才考虑清理。回退参考为本轮前的 `d967ddd`；旧 runtime 与旧证据未覆盖。

下一轮提示（来自并行离线诊断，非本轮验证结论）：补录 raw wheel/IMU 与 map→odom 的 TF 所有者，核查可能的 AMCL/global_ekf 双所有者及局部里程计偏移；不要以只筛 CONFIRMED 替代感知误检修复。
