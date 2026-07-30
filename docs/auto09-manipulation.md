# AUTO-09：抓取、运输与入箱

## 结论

AUTO-09 软件与离线运动学机器门通过。交付包括 opt-in 四自由度机械臂、
双指夹爪、`ros2_control` mock hardware、MoveIt2 SRDF、KDL、OMPL、
controller/joint-limit 配置，以及感知到抓取坐标变换、抓取候选、规划场景、
40 L bin 状态、失败恢复和急停逻辑。`leaf_pile` 与 `puddle` 只进入刷扫，
不会生成抓取候选。

三种离散类别各执行 20 次 pose-known micro 和 30 次带独立感知噪声的正式
闭环。micro 抓取/抬升、正式 pick/transport/bin placement 的逐类最低成功率
均为 `1.0`；wrong-object、safe-zone 外掉落、碰撞与关节越界均为 0。另执行
90 个不可达目标，全部 fail-closed。40 L bin fill state 可观测，过量命令
拒绝率 `1.0`，bin-full routing 与急停拒绝均通过。

证据等级严格限定为
`OFFLINE_KINEMATIC_PERCEPTION_LOOP_SIMULATION`。MoveIt2 与 ros2_control
交付物在此阶段只做静态合同审计；没有把这些结果声称为 Gazebo 动态抓取或
实体机械臂成绩。

## 安全链

- `enable_manipulator:=false` 是车辆默认值，旧导航/感知几何不被默认改变。
- 目标类别、协方差、TF、IK 和规划场景任一不合格即拒绝执行。
- 不可达目标不会产生轨迹；急停会取消持有状态并进入安全终态。
- 夹爪只允许在 safe drop zone 释放；bin 容量预留失败时转入 bin-full route。
- truth world-state 与带噪感知 estimate 分开，truth 不进入控制。

## 复现

```powershell
py -3 scripts/auto09_manipulation_formal.py `
  --output artifacts/autonomous_auto09_20260730_evidence `
  --implementation-commit f094257b8fa3391f5c7c93c79132a425f096c46a
py -3 scripts/ci_fast.py
```

ROS 2 环境中的 opt-in 入口是
`ros2 launch sanitation_manipulation manipulation.launch.py`。该入口会以
`enable_manipulator:=true` 物化 URDF，启动 controller manager 与 MoveIt2
move_group；真实 Gazebo/硬件控制器仍需后续接入和动态验收。
