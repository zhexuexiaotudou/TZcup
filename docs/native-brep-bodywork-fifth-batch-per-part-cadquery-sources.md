# 第五批：47 件车身逐件 CadQuery 参数化源码

第五批源码 [`native_brep_bodywork_fifth_batch.py`](../starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_bodywork_fifth_batch.py) 与合同 [`native_brep_bodywork_fifth_batch_contract.json`](../config/high_fidelity_vehicle/native_brep_bodywork_fifth_batch_contract.json) 将当前 `native_brep_source_coverage_audit.json` 中全部 47 个 `bodywork` 行逐件显式建模：每个 `source_mesh` 有唯一 `part_id`、唯一函数名和一个由生成器参数直接转录的特征族。

这不是把整车或一套车身熔合为包装盒。源码包含分段 loft 车身/护罩、带圆角的厚板和饰条、轮拱切除、带门缝与铰链耳的四类检修门、铰链桶和锁舌支座、环形检修饰框，以及带透镜凹位的灯座。它不读取、导入、转换或反推任何网格。

尺寸的主要权威源是 `generate_product_bodywork_meshes.py`；合同逐件标明对应 `generator_locator`，并把 `bodywork.xacro`、`formal_vehicle_layout.yaml` 和 BOM 限定为安装/包装和 make-buy 边界参考。孔图、螺纹、材料、成形、公差、门缝最终间隙、密封、热/EMC、灯具或锁具供应商接口、载荷与制造验证均是 pending。当前合同状态 `design_input_pending_native_export` 会在导入 CadQuery 前拒绝导出，且不创建 STEP/FCStd。

哈希清单 [`native_brep_bodywork_fifth_batch_source_manifest.json`](../starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_bodywork_fifth_batch_source_manifest.json) 同时绑定源码和合同。低内存验证只解析 Python AST、JSON 和 SHA-256：

```powershell
py -3 scripts/test_native_brep_bodywork_fifth_batch_sources.py
py -3 starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_bodywork_fifth_batch.py --summary
```

哈希或静态覆盖通过不构成原生 CAD、制造或安全放行；未来只能由单独审查的孔图、材料/GD&T、门密封与供应商接口、热/EMC/结构验证和非 tessellated STEP 预检解除门禁。
