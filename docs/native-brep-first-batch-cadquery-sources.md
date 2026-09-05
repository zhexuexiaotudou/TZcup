# 首批四件 CadQuery 原生参数化源码

[`native_brep_first_batch.py`](../starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_first_batch.py)
是 `arm_pedestal_adapter`、`sensor_tower`、`cleaning_head_brackets` 和
`storage_frame` 的项目自研 CadQuery 源码。它直接读取
[`native_brep_first_batch_contract.json`](../config/high_fidelity_vehicle/native_brep_first_batch_contract.json)：米制合同参数只在构造边界转换为 CadQuery 的毫米制数值。

[`native_brep_first_batch_source_manifest.json`](../starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_first_batch_source_manifest.json)
列出唯一可编辑源码和合同输入的 SHA-256。每次源码或合同输入变化，都必须在同一受控变更中更新该清单和静态测试；哈希更新不等同于 export 授权。

`audit_native_cad_readiness.py` 会把该目录内同时满足“惰性 CadQuery
导入、真实实体特征构造、无 STL/三角网格写出标记、被该 source manifest
精确 SHA-256 绑定”的源码列为 `editable_native_brep_sources`。这仅说明项目
拥有可编辑 B-rep 源输入；本文件当前的 `design_input_pending_native_export`
状态不会让审计放行。整车仍须另有完整组件 assembly manifest、非
tessellated STEP、受 Windows 原生 exporter 生成的 receipt，以及 receipt
中的源哈希后才可能达到 `ready`。

| 工作包 | 原生特征构造 | 合同中未放行的接口特征 |
| --- | --- | --- |
| `arm_pedestal_adapter` | 甲板背板、加强台座、外法兰、内轮毂和四个三角加强筋 | 六点圆周位置只有 datum；不能生成 UR5e 孔、螺纹、沉孔或定位销 |
| `sensor_tower` | 底座、双立柱、服务脊、三横撑、头板和双三角加强筋 | 四个底座位置以及各传感器安装接口仍需受控孔图 |
| `cleaning_head_brackets` | 主轨、两侧支架和端块、四根导柱、升降板、导向 boss、两移动滑板 | 底盘、导轨/轴承、Actuonix/clevis 的孔、配合与销轴仍未放行 |
| `storage_frame` | 托盘、两纵梁和两横梁 | 六个甲板 datum 不是托盘孔，箱体/罐体接口也尚未放行 |

当前合同状态是 `design_input_pending_native_export`。因此模块的普通入口只读取合同并输出摘要：

```powershell
py -3 starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_first_batch.py --summary
```

它不会导入 CadQuery、启动 CAD 内核、创建输出目录、导出 STEP，或读取/转换任何网格。以下低内存回归检查也只执行 JSON 和 Python 源码路径：

```powershell
py -3 scripts/test_native_brep_first_batch_sources.py
```

## 未来受控导出路径

真正的独立 part STEP 和一个 part-local assembly STEP 只能在以下条件全部满足后使用：合同经评审改为 `native_export_released`；一份新的、版本控制且评审过的 release-evidence JSON 精确绑定该合同 `document_id`；每件显式关闭其 `export_preconditions`；每件提供等于合同 datum 数量的 `released_holes`。每个孔都必须以 `datum_xy_m` 一对一绑定合同位置，并写出受控的 `start_m`、`axis`、`diameter_m` 与 `depth_m`，绝不能从视觉紧固件包络推测。

届时才可在通过 Windows 原生 CadQuery 预检、并锁定 CadQuery 环境中显式调用：

```powershell
py -3 starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_first_batch.py `
  --export --release-evidence <reviewed-release.json> --output-directory <empty-output-directory>
```

程序首先验证合同和 release evidence；任一状态、预条件、孔记录或 datum 数量不一致都会在导入 CadQuery 和创建任何文件之前退出。通过后才生成四个独立 `.step` 及 `formal_vehicle_first_native_brep_batch_assembly.step`。这不是 FCStd 交付、制造图、材料/载荷/DFM/GD&T 放行、实测质量惯量或整车 readiness 证据。

源码不含网格导入、网格重建、mesh-to-STEP、faceted/tessellated STEP 或占位 FCStd/STEP 路径。现有合同规定的计划 FCStd/STEP 输出也继续保持不存在，直到将来由单独、受控的变更更新合同和验证器。
