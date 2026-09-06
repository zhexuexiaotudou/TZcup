# 无精确URDF的园区主动清扫研发闭环

## 已实现范围

本闭环用于在真实小车与机械臂URDF到达前，提前验证不会依赖精确几何的任务语义：程序化园区与随机任务、评测真值隔离、有限感知主动清扫、虚拟Ackermann轨迹、安全审计、Q-learning训练接口、3 cm方块几何候选、单目标抓取、最多两次尝试、投箱验证和统一结果报告。

场景提供`research=106 m × 53 m`和`formal=200 m × 100 m`两个面积基准。每个split的map0保留基准尺寸，其余地图在面积不变的前提下由layout seed改变长宽比。冻结预算为32张训练地图×200任务、8张验证地图×100任务、12张hidden地图×100任务，共52张地图、8,400个任务。生成一个任务不等于已经执行整个预算。

校园生成器中的 `puddle` 与其他 dirt patch 一样，是供感知、覆盖规划和评测使用的无碰撞地表语义/可视表面。它不携带水深、水量、水动力、自由液面或回收质量守恒，因此不能作为积水物理回收通过证据。水深、地面体积扣减、污水箱增量、满箱闭锁和刮吸接触只能由独立的正式 water runtime 验收。

生成目录严格分为：

- `public/`：world和不含种子/真值路径的controller清单；
- `environment/`：仅供Gazebo环境进程使用的行人运动计划；
- `evaluator/`：种子、哈希和ground truth，只能由独立评测器读取。

## 一条命令运行

在ROS 2 Jazzy工作区构建并source后：

```bash
ros2 run sanitation_research_demo urdf_independent_research_demo \
  --config $(ros2 pkg prefix sanitation_campus_scenario)/share/sanitation_campus_scenario/config/default_scenario.yaml \
  --profile research --split train --map-index 0 --mission-index 5 \
  --output /tmp/urdf-independent-research-demo
```

命令拒绝覆盖已有目录。`scenario/`保存三域场景包，`report.json`保存episode身份、public/world哈希、任务指标、投箱证据哈希和权威边界。策略只收到`AgentObservation`；统一harness读取真值来实例化和评分环境，但不会把真值传入策略或抓取回调。

单独验证占位描述和抓取语义：

```bash
xacro $(ros2 pkg prefix sanitation_manipulation)/share/sanitation_manipulation/urdf/placeholder_mobile_manipulator.urdf.xacro > /tmp/placeholder.urdf
check_urdf /tmp/placeholder.urdf
gz sdf -k /tmp/placeholder.urdf
ros2 run sanitation_manipulation placeholder_cube_demo
```

## 当前验收证据

- Windows快速门禁：303项通过；
- WSL/ROS Jazzy：23个包构建成功，6个变更包共87项测试、0失败；
- placeholder Xacro：`check_urdf`成功，Gazebo SDF校验`Valid`；
- 场景：Gazebo Harmonic加载代理底盘和8个行人，`set_pose`服务存在，driver状态`ACTIVE`，实体位置发生变化；
- 统一研发任务：62 steps，观测率`0.9743`，已观测地污清除率`1.0`，离散垃圾清除率`0.95`，19/19次投箱请求验证成功，任务轨迹`413.38 m`，碰撞/越界/非法动作均为0；WSL安装后CLI约7秒完成。

上述数字只对应确定性的`research/train/map0/mission5`软件占位任务，不是统计结论。另一个任务可能因80-step技术上限被截断，证明后续必须报告完整任务分布，不能只展示成功seed。

## 明确不声明

- 没有真实URDF、动力学、传感器插件、MoveIt 2、IK、自碰撞、夹爪力/开度或手眼标定；
- 没有运行DOSOD或EdgeSAM权重；虽然锁定上游已有两者的S100参考链，本项目仍因外部模型、正式话题适配和实板运行证据缺失而fail-closed；
- belief传感器、圆形地污栅格和近距方块点云仍是软件模拟，未证明零样本感知精度；
- 场景中的 `puddle` 只是无碰撞的感知/覆盖地表语义，不证明水深、回收量、流体运动或刮吸物理；
- 新园区尚未接入真实SLAM、固图定位、Nav2和首次建图流程；
- 未执行8,400任务、阶段A 10,000/500/1,000预算或正式200 m×100 m综合矩阵；
- 不得将本闭环改写成实车、RDK S100、Journey 6或赛题最终通过证据。
