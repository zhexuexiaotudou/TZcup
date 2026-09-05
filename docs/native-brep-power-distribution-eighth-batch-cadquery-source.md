# 第八批：PDU 箱体逐件 CadQuery 源码

[`native_brep_power_distribution_eighth_batch.py`](../starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_power_distribution_eighth_batch.py) 为 reconstruction manifest 中唯一的 `power_distribution_box` 项提供独立、可编辑的项目自研 B-rep 源。它以当前生成器、Xacro 安装坐标、layout 坐标约定和 BOM 的 make-buy 边界为输入，构成六个可区分的服务结构：空心带圆角箱体壳、可拆盖、DIN rail 安装界面、五个熔断器支撑界面、三个端子排支撑界面和五个线缆入口 boss 包络。

这不是融合包装盒，也不重建熔断器、端子、线缆、继电器、接触器、隔离器或 DC-DC 等供应商硬件。线缆入口是外部 boss 包络，不是受控孔或 gland 定义。

合同 [`native_brep_power_distribution_eighth_batch_contract.json`](../config/high_fidelity_vehicle/native_brep_power_distribution_eighth_batch_contract.json) 明确保持制造孔位、材料/公差、密封/IP、爬电间距、额定电流、短路/保护协调、热、EMC、接地、线束和服务空间为 pending。当前状态为 `design_input_pending_native_export`，因此 `--export` 在导入 CadQuery 前即拒绝，且不会写出 FCStd 或 STEP。

[`native_brep_power_distribution_eighth_batch_source_manifest.json`](../starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_power_distribution_eighth_batch_source_manifest.json) 以 SHA-256 同时绑定源和合同。低内存检查只解析 Python AST、JSON 和哈希：

```powershell
py -3 scripts/test_native_brep_power_distribution_eighth_batch_sources.py
py -3 starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_power_distribution_eighth_batch.py --summary
```
