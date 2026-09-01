# 清扫执行器电机现实性闭环

正式整车保留 `gz_ros2_control` 作为五个清扫关节的唯一物理命令写入者。新增的 `CleaningActuatorMotorSystem` 只读取 Gazebo 中的关节位置/速度和安全管理器之后的控制器参考，不写 `JointForceCmd`、`JointVelocityCmd` 或 `JointPositionCmd`。因此它不会与现有控制器争夺关节，也不会改变 `GroundDirtCleaningSystem` 和 `WaterRecoverySystem` 继续按真实关节速度判定清扫/回收的机制。

## 型号与边界

- 左右侧刷和中央滚刷使用 Pololu 4694。模型采用厂家公开的 24 V、140 rpm、0.10 A 空载、3 A 外推堵转电流和 31 kg·cm 外推堵转转矩；连续电流采用厂家“通常不超过堵转电流 25%”建议，即 0.75 A。堵转数值不是连续额定值。
- 升降采用 Actuonix P16-100-256-12-P。关节速度/推力已经收紧到厂家公开的 4.8 mm/s 和 300 N，堵转电流为 1 A；热模型同时保留厂家 20% 最大占空比和 -10–50 °C 工作边界。
- 回收泵采用 Jabsco Q402J-118S-3A。厂家公开 24 V、60 psi 下最大 6 A、10 A 保险丝、间歇工作和内置 TCO。厂家未公开隔膜偏心轮转速/转矩，所以 600 rpm 与 2 N·m 明确标为工程标定值，不伪装成数据手册参数。

电机核心按命令、实测速度误差和电压计算输出负载、电流与铜耗，以一阶热网络积分温度；持续低速大误差触发堵转锁存，达到温度阈值触发过热锁存。显式复位只在全局禁能、所有命令归零且所有电机低于复位温度时接受。Gazebo 暂停时 `PreUpdate` 不积分，热量和堵转计时均冻结。

## ROS/Gazebo 接口与安全链

`cleaning_actuator_command_mirror` 读取 `/brush_controller/commands`、`/recovery_controller/commands`、`/cleaning_controller/controller_state` 与 `/safety/actuators_enabled`，只镜像已经过安全门的参考。`cleaning_actuator_motor_bridge` 将这些参考单向送入 Gazebo，并把五电机电流、温度、估算负载、合计功耗、JSON 状态和故障单向送回 ROS。

故障话题 `/model/tzcup_formal_sanitation_vehicle/cleaning_motors/fault_active` 是 `whole_vehicle_safety_manager` 的必需新鲜输入。缺失、超时或为真都会全局 fail-closed：底盘归零、刷/泵控制器停用、位置控制器取消目标并保持安全位置。机械臂展开时原有语义不变，仍只禁止底盘运动；只有真实电机故障才触发全局禁能。

完整话题、参数来源、单位顺序与待跑运行验收见 `config/high_fidelity_vehicle/cleaning_actuator_motor_realism_contract.yaml`。静态验证命令为：

```powershell
py -3 scripts/validate_cleaning_actuator_motor_contract.py
py -3 -m pytest -q scripts/test_cleaning_actuator_motor_contract.py scripts/test_validate_formal_cleaning_actuator_motor_runtime.py scripts/test_run_formal_cleaning_actuator_motor_runtime.py
```

## 正式运行门

本轮没有启动 Gazebo，因此当前只完成了 runner 与 fail-closed 验收逻辑，不能把 live runtime 写成通过。最终冻结整车源码并完成全新 colcon overlay 后，在正式 acceptance session 的 `RUNNING` 窗口内执行：

```bash
export FORMAL_VEHICLE_RUNTIME_WS=/absolute/path/to/fresh_frozen_workspace
export FORMAL_ACCEPTANCE_SESSION=/absolute/path/to/formal_final_acceptance_session.json
bash scripts/run_formal_cleaning_actuator_motor_runtime.sh
```

runner 会启动同一个 `formal_vehicle_sim.launch.py` 正式整车，通过安全管理器入口给左右侧刷、中央滚刷和回收泵下发正常非零负载，同时由 `cleaning_controller` 驱动 P16 升降。堵转不通过伪造 JSON 或修改关节状态产生：控制器给升降机构下发 `0.125 m` 参考，URDF 的 `0.100 m` 真实行程限位阻止实体关节继续运动，采集器同时记录 controller reference、`/joint_states` 实际位置/速度、1 A 堵转边界、故障锁存以及 `whole_vehicle_safety_manager` 的全局禁能。随后所有命令保持 idle，观察温度下降，再通过正式 reset topic 显式复位并验证五电机健康和整车 permit 恢复。

原始采集默认写入 `artifacts/formal_cleaning_actuator_motor_runtime.capture.json`；runner 会在启动 Gazebo 前对 snapshot、运行中 session 和 frozen closure 生成 `artifacts/formal_cleaning_actuator_motor_runtime.json.runtime_binding.json`，并把旧的报告、原始采集、日志和 sidecar 逐一轮换保留。validator 强制接收该 sidecar，重新核验采集 source binding、session 哈希与起始时间、closure 和当前 `AMENT_PREFIX_PATH` 中实际解析到的冻结 overlay，最终在 `formal_cleaning_actuator_motor_runtime.json` 完整嵌入 `runtime_gate_binding`。该正式报告仍通过 `source_binding.expanded_urdf_sha256` 受冻结 snapshot 约束，并在 `formal_functional_acceptance_contract.yaml` 中作为 `session_bound` gate 约束 `side_sweeping`、`main_sweeping`、`cleaning_head_lift` 和 `water_pumping`。

生产热时间常数不为缩短验收而修改。短时 live 门不声称触发过热；过热阈值、热态拒绝 reset、idle 冷却到复位阈值和显式 reset 由 `test_cleaning_actuator_motor_core.cc` 在保持生产参数的情况下推进仿真时间验证。live 门只证明正常负载、实体行程限位堵转、全局禁能、idle 冷却和复位链。
