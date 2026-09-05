# 正式整车机械制造放行包

## 当前状态

当前状态固定为 `NOT_READY_FOR_MECHANICAL_MANUFACTURING_RELEASE`。URDF、OpenSCAD 包装布局、
STL 网格、部件台账和 Gazebo 关节模型可复用于制造设计输入，但它们都不是 STEP、二维工程图或
制造放行证据。尤其视觉 STL 不得被当作 STEP 或工程图。

机器可读清单为
[`manufacturing_bom.csv`](../config/high_fidelity_vehicle/manufacturing_bom.csv) 与
[`mechanical_release_readiness.yaml`](../config/high_fidelity_vehicle/mechanical_release_readiness.yaml)。
其中 7 条 `baseline_allocated` 行仅重现最后展开 URDF 的名义 `160.007583 kg`；采购件参考质量和
未知实物质量不重复相加。当前冻结数字快照已经逐项复核：展开 URDF 的 SHA-256 为
`2b399eaba8de34fd55ca663a3b6b49437070c72969f5e22faf2a64a12ba27db8`，快照清单文件自身的
 SHA-256 为 `f4a19f0361a7fe6f899a904ecebf2db5ba7b41c23ec2e0a5ae31d3400e0c7ba1`，source inventory
摘要为 `11123081a300b9c448fa09d7053318fafa4df1ea2f1d6de5d008ce62340521e5`，output inventory
摘要为 `199b895465d9215d42aff3e2efad69171d57b96bd2bd844495785af4e88587b6`。质量声明仍为
`nominal_model_allocation_not_actual_weighed`。这些绑定只表明名义数字基线未漂移；该数字不是
实物称重，也不是制造证据。

校验器同时绑定 BOM 的 SHA-256（当前为
`5de1fb2197870f937c5145bcc96f242adc428a772a14ba8d8779ee1b38294d97`）。任何源、展开产物或 BOM
漂移都会令该“数字基线可复算”声明失效，不能通过编辑质量数值、更新 STL 或更新配置来获得放行 credit。


## 可复用与必须重新设计

可复用：A300、UR5e、2F-85 与公开传感器的型号/安装接口约束；项目 URDF 的机构链、关节限位、
40 L 干箱、8.3 kg 污水容量、清扫宽度和服务位置；以及现有项目自研的包装布局和网格外观。

必须重新设计并形成放行件：臂座/转接盘、传感器塔、车身与服务门、干湿箱、清扫头支架与护罩、
线束/防水/热/接地结构、充排接口和所有项目自研连接件。每项必须拥有原生 STEP、二维图纸与
GD&T、材料与表面处理、紧固/焊接或连接规范、装配工艺、检验计划、实物称重与惯量、结构 FEA、
防水试验和维护图。缺一项都不得将 `ready` 设为 true。

## 质量与底盘边界

5 kg 与 10 kg 制造余量仍是未验证目标。保守口径分别需减去 `4.969583 kg` 和 `9.969583 kg`；
候选只限项目自研臂座、车身、箱架、塔体和清扫支架，统一是
`design_required_not_credited`。不得通过直接改小 URDF mass/inertia 或 reserve 获得 credit；只有
实物改型、称重、惯量/重心复算、重新冻结后才可计入。

若保留赛题功能和水容量后无法实现 5 kg 余量，底盘额定 payload 至少应为 `107.02 kg`；10 kg
目标至少 `112.58 kg`。更高 payload 不代替满水、伸臂、制动、转弯和坡道下的独立稳定性验证。
