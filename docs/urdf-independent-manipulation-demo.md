# 无精确URDF的3 cm立方体抓取占位闭环

## 状态与证据边界

本模块用于真实小车URDF到达前验证软件接口和任务语义。它不是实车模型，也不提供机械臂可达性、自碰撞、载荷、夹持力、相机可见性、麦克纳姆接触或sim-to-real证据。占位profile的`placeholder=true`且`evidence_authority=false`；后续不得通过调换名称将其升级为真实证据。

旧AUTO-09的瓶、罐、纸类四轴离线运动学接口保持不变。新链路独立面向边长`30 mm`、单层、不堆叠的随机颜色立方体：

```text
标准XYZ点云
→ 近水平地面RANSAC
→ 18–45 mm离地高度筛选
→ 空间哈希欧氏聚类
→ 最小XY包围盒与30 mm三维尺寸门
→ 两个对称垂直顶抓Pose
→ mock导航/规划/夹爪
→ 两类以上独立抓取证据
→ 20 cm × 20 cm × 10 cm后箱
→ CLEARED或有界DEFERRED
```

算法核心只接收普通XYZ序列并返回Python dataclass。ROS适配器负责在边界转换`PointCloud2`、TF和`Pose`，因此几何单元测试不需要ROS、PCL、NumPy或具体机器人模型。地面、目标和抓取决策不得读取Gazebo真值。

## 任务合同

- 每个episode最多20个方块，后箱任务上限同为20个；该上限不是随机堆积容量证明。
- 每个目标最多两次抓取尝试；规划失败、空抓或验证不足都会触发回观测位恢复。
- 抓取验证分为三类：夹爪开度/力矩、目标随末端抬升、原位置目标消失；至少两类成立才可投箱。
- 行人或障碍进入机械臂安全区时进入`SAFETY_PAUSED`，不消耗抓取次数；安全恢复后重新规划或继续投箱。
- 只有投箱动作和投箱状态都通过，实例才进入`CLEARED`并计入清除率。
- mock后端仅证明状态机闭合。底盘对位、IK、全场景碰撞和实际关节轨迹必须由收到真实URDF后的Nav2、MoveIt 2和`ros2_control`替换。

ROS合同位于`sanitation_perception_interfaces`：

- `CubeGraspCandidate.msg`
- `CubeTargetState.msg`
- `GraspVerification.msg`
- `PickCube.action`

## 占位模型

`sanitation_manipulation/urdf/placeholder_mobile_manipulator.urdf.xacro`提供参数化的`0.60 m × 0.40 m`车体、四个麦克纳姆外形轮、通用六轴臂、平行夹爪、单线LiDAR、Mid-360、前向RGB-D、两个侧后鱼眼、腕部双目frame和开放式后箱。配套SRDF和YAML均带有`placeholder`标识。

该Xacro不接入正式场景，也不替换当前生产车辆。它只用于接口、TF命名、MoveIt配置准备和静态合同检查。

## ROS无关运行与测试

在仓库根目录运行：

```powershell
$env:PYTHONPATH='starter_ws/src/sanitation_manipulation'
py -3 -m sanitation_manipulation.placeholder_demo
py -3 -m pytest -q starter_ws/src/sanitation_manipulation/test
```

演示输出必须同时满足`success=true`、`task_state=CLEARED`、`placeholder_evidence_only=true`、`real_robot_evidence=false`和`gazebo_truth_used_for_control=false`。

主动清扫规划器与本占位闭环之间使用独立的单目标适配器；它不修改或导入主动清扫环境主体。输入示例为`starter_ws/src/sanitation_manipulation/config/active_cleaning_grasp_request.example.json`，运行命令为：

```powershell
$env:PYTHONPATH='starter_ws/src/sanitation_manipulation'
py -3 -m sanitation_manipulation.active_cleaning_adapter `
  --request starter_ws/src/sanitation_manipulation/config/active_cleaning_grasp_request.example.json
```

输出只有在`task_state=CLEARED`、控制器确认投箱且目标身份已登记到后箱三项同时成立时才设置`decision.cleared=true`。每份输出都固定标记`evidence_level=MOCK_TASK_SEMANTICS_ONLY`、`evidence_authority=false`、`real_robot_evidence=false`。场景对接草案位于`scenario_adapter_contract.py`，只接受controller-facing公开任务身份和感知几何，不接受评估真值或行人驱动计划。

## 真实URDF替换门

进入真实MoveIt/Gazebo闭环前至少需要：

1. 带准确visual/collision/inertial、机械臂安装位姿、六关节限制和夹爪结构的实测URDF；
2. SRDF、IK插件、自碰撞矩阵、`ros2_control`控制器和关节反馈合同；
3. 夹爪开度、指宽、允许力矩/电流与30 mm方块可夹性证据；
4. 前向RGB-D、腕部双目和手眼标定，含内参、畸变、baseline、最近工作距离和精确TF；
5. 机械臂触地、收拢运输和后箱投放的可达性及碰撞验证；
6. 麦克纳姆轮几何、驱动接口和虚拟Ackermann轨迹跟踪验证。

任一项缺失时，只能继续报告占位闭环，不得宣称真实整车抓取或板端部署完成。
