# 正式整车静态功能链审计

`scripts/generate_static_functional_chain_audit.py` 是一个只读、低内存的
机器可读审计器。它不启动 WSL、Gazebo、Docker 或 ROS，也不采集数据；它只沿真实
Xacro/URDF、`ros2_control`、Gazebo 插件、launch、运行时校验脚本和传感器桥接配置
检查静态依赖链。

```powershell
py -3 .\scripts\generate_static_functional_chain_audit.py `
  --output .\reports\engineering\static_functional_chain_audit.json
py -3 .\scripts\validate_static_functional_chain_audit.py `
  --report .\reports\engineering\static_functional_chain_audit.json
py -3 -m pytest .\scripts\test_static_functional_chain_audit.py -q
```

报告的 13 项核心闭合链固定覆盖：四轮前进/制动、UR5e 六轴与 Robotiq 夹爪、带
密度/碰撞/惯量的 30 mm 材料立方体到后部干垃圾箱、干垃圾独立刚体增重、地面脏污
覆盖、侧刷/滚刷/刮吸/真实可回收积水至污水箱、单线 UTM、MID-360、前向 RGB-D、
后部双鱼眼、腕部 RGB-D（红外双目通道）、GNSS 和轮速。

另有两个不混入历史 13 项门禁的扩展项：S100P 安装及低压供电、以及 UTM+MID-360
输入到 Nav2 costmap / collision monitor / velocity gate 的避障链。每项都携带
`physical_semantics` 与 `placeholder_indicators`，因此报告明确区分“仅命名/接口存在”、
“有碰撞、惯量、接触或状态耦合的物理语义”与“暂定参考”。

## 状态语义

- `STATIC_CLOSED`：每一个所需静态环节都有具体源文件、插件/关节/topic 或 launch
  证据；它不表示 Gazebo 已运行，也不表示指标通过。
- `BLOCKED`：缺少可追溯的源级接口，或代码明确隔离了该接口。审计器不会因组件、
  topic 或报告名称存在而把它写成通过。

顶层 `status` 保持历史 13 项核心链的兼容门禁语义；扩展范围必须读取
`status_scope` 和 `expanded_scope_status`。`expanded_scope_runtime_accepted` 固定为
`false`。当前权威 Xacro 和组件台账都明确说明 S100P 使用旧 RDK S100
包络作暂定碰撞参考，且自有板卡的尺寸、孔位、质量、热、连接器与上电实机验证仍待
完成。因此 `s100p_installation_and_low_voltage_power` 的静态状态必为 `BLOCKED`，而
`expanded_scope_status` 也为 `BLOCKED`；这并不否定其机柜安装板、外壳、PDU/DC-DC
和软件启动图在源级存在。

干垃圾记账由
[`dry_payload_accounting_contract.yaml`](../config/high_fidelity_vehicle/dry_payload_accounting_contract.yaml)
定义为显式互斥模式。正式 20 立方体场景固定使用 `physical_resident`：20 个物体保持
为独立 Gazebo 刚体，入后部干箱后由真实接触向车体传递载荷；`DryBinMonitorSystem`
发布驻留刚体的实际数量和惯性质量。此时 `DynamicPayloadSystem` 的 aggregate dry mass
强制为 `0` 并拒绝非零输入，避免双计数。历史上没有实体刚体的 bulk 场景可显式选择
`aggregate`，但不得同时存在驻留刚体。静态闭合不表示 Gazebo 已运行或运行指标通过。

`--require-static-closed` 适合强制门：只要存在任何 `BLOCKED` 项就会失败。正常的
`validate` 只检查 JSON schema、证据结构和 fail-closed 状态一致性，从而允许报告
诚实地记录当前缺口。

静态审计输出还固定包含 `required_item_count: 13`、
`static_closed_count`、`runtime_accepted: false` 与
`fresh_gazebo_runtime_required: true`。独立校验器拒绝项目数不为 13、项目顺序/标识漂移、
静态闭合计数不一致，扩展项状态/计数不一致，或任何把静态检查误标为运行时已接受的
报告。该审计已被
`run_formal_vehicle_static_engineering_preflight.py` 和 `ci_fast.py` 调用；不写入或伪造
Gazebo 运行时报告，也不修改 central final acceptance 门。
