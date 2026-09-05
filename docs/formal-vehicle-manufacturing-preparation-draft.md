# 正式整车机械制造准备草案

## 结论

当前可以诚实完成的是“制造设计输入准备”和缺口审计，而不是制造放行。新增的机器可读草案
[`mechanical_manufacturing_preparation_draft.yaml`](../config/high_fidelity_vehicle/mechanical_manufacturing_preparation_draft.yaml)
把已有 URDF、包装布局、部件台账和名义 BOM 的可复用信息编入材料、紧固件、连接/焊接、
表面处理、装配、检验和维护图输入。它固定为 `DRAFT_DESIGN_INPUT_NOT_RELEASED`，并由
`validate_formal_vehicle_mechanical_manufacturing_preparation_draft.py` fail-closed 校验；该校验通过
只表示草案结构完整、源输入未漂移、放行主清单仍为未就绪，绝不表示可下单、可加工或可验收。

## 输入与可完成范围

| 现有输入 | 本草案可用的事实 | 不能推导的事实 |
| --- | --- | --- |
| 名义 URDF/Xacro | 安装链、关节、服务接口和包装对象 | 制造尺寸、公差、结构强度或真实质量 |
| 布局 YAML | 名义坐标、外包络和服务区域 | 加工基准、装配公差或干涉合格 |
| 组件台账 | 18 个机械子总成的连接语义与产品功能 | 螺纹规格、预紧力、焊缝尺寸或供应商批次 |
| 名义制造 BOM | make/buy 范围、33 行名义质量分配 | 可采购物料号、材料证书、实际质量或成本 |

草案因此仅给出候选材料族、待确认的紧固件位置与数量依据、连接设计输入、表面/防腐所需
环境输入、概念装配顺序、检验记录字段及维护图所需视图。所有扭矩、预紧、最终材料牌号、
焊接/粘接/密封工艺、GD&T 和接受准则都明确使用 `pending://` 占位，不能被解释为技术条件。

## 已建立的制造准备项

- 材料：结构、湿垃圾/回收路径、外饰与传感器/计算安装的候选材料族及必须确认的强度、
  腐蚀、化学兼容、振动/热/EMC 输入。
- 紧固件：传感器塔四个底座螺栓、机械臂六螺栓转接、计算盒、检修门和清扫头的初步表；
  螺纹、等级、表面、锁固、扭矩/预紧和检验仍待受控图纸与载荷计算。
- 连接：塔体、臂座、湿路和车身检修接口分别列出承载/密封设计输入，禁止用 URDF joint、
  STL 或 Gazebo 流量模型替代接头、泄漏或工艺证据。
- 表面：外露金属、湿区与外饰的防腐/耐候需求被分开，未选定任何最终涂层系统。
- 装配与检验：按来料、主承力、储存回收、清扫、电气传感、车身服务、最终构型建立顺序和
  hold point；量检基准、扭矩、泄漏、涂层、质量/重心/惯量记录都在受控文件放行后才可执行。
- 维护图：检修门/隔离、干湿箱、清扫头、传感器/机械臂/计算盒均指定所需视图，但没有把
  当前包装模型或视觉 STL 误标为维护图。

## 仍然阻断制造放行的项目

草案保留 13 个不可跳过的 release hold point：原生 STEP/STP、受控二维图与 GD&T、材料证书、
紧固件扭矩/预紧、合格的连接/焊接/粘接/密封工艺、表面处理、装配工艺、检验计划与记录、
实物称重/重心/惯量、FEA/疲劳/稳定性、防水试验及维护图。它同时要求
[`mechanical_release_readiness.yaml`](../config/high_fidelity_vehicle/mechanical_release_readiness.yaml)
继续保持 `NOT_READY_FOR_MECHANICAL_MANUFACTURING_RELEASE` 和 `ready: false`。

## 复核命令

```powershell
py -3 scripts/validate_formal_vehicle_mechanical_manufacturing_preparation_draft.py
py -3 -m pytest -q scripts/test_validate_formal_vehicle_mechanical_manufacturing_preparation_draft.py
```

这两条命令仅解析本地 YAML/CSV/XML 并运行纯 Python 单元测试；不会启动 WSL、Docker、Gazebo
或访问任何物理硬件。
