# 正式功能需求覆盖与运行缺口登记

`config/high_fidelity_vehicle/formal_requirement_coverage_gap_register.json`
是逐项、机器可读的输入表；它将竞赛方案、部件架构、正式 Xacro/URDF、控制源码和
`formal_functional_acceptance_contract.yaml` 的运行时门统一交叉到十六项：八类外感知、
轮速观测、A300 底盘、UR5e、2F-85、干箱动态载荷、刷地、刮吸/回收、污水载荷及安全联锁。

用纯 Windows/Python 生成并验证受控报告：

```powershell
py -3 scripts/audit_formal_requirement_coverage.py
py -3 scripts/validate_formal_requirement_coverage_gap_register.py --report reports/engineering/formal_requirement_coverage_gap_register.json
```

该程序不启动 WSL、`bash.exe`、Gazebo、ROS、CadQuery 或 FreeCAD。它分别输出
`documentation`、`model`、`control`、`formal_runtime_gates` 四类证据；
`STATIC_MODEL_CONTROL_GATE_DECLARED` 只表示这些源码和合同声明同时存在。

当前报告必须保持 `runtime_accepted: false`。正式会话即使处于 `RUNNING`，只要其
`evidence` 未绑定相应 gate，条目就是
`MISSING_CURRENT_SESSION_GATE_EVIDENCE`；不能因旧静态审计、旧 Gazebo 产物或
合同中出现 gate 名称而宣称实车或仿真运行通过。真正闭环仍须在当前冻结快照的正式
运行会话中产生、绑定并按合同独立验证的运行时证据。
