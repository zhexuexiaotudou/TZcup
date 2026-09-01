# 正式整车静态工程预检

这是一个专门给低内存 Windows 主机使用的纯 Python 聚合预检。它把十四个已有的
fail-closed 检查汇成一份机器可读报告：

- [`native-cad-preflight.md`](native-cad-preflight.md) 的原生 CAD 证据审计；
- [`formal-vehicle-manufacturing-preparation-draft.md`](formal-vehicle-manufacturing-preparation-draft.md)
  的制造设计输入草案校验；
- [`native-brep-reconstruction-manifest.md`](native-brep-reconstruction-manifest.md)
  的逐件原生 B-rep 重建覆盖校验。
- [`native-brep-first-batch-contract.md`](native-brep-first-batch-contract.md)
  的首批四件原生 B-rep 参数合同校验；
- [`formal-mechanical-interface-datums.md`](formal-mechanical-interface-datums.md)
  的 SHA-256 绑定零关节机械接口 datum crosswalk 校验。
- 清洗/回收第二批七个 CadQuery 设计输入的独立 source/contract/manifest 校验：精确
  SHA-256、惰性 CadQuery、无 mesh import，且 FCStd/STEP 均必须仍不存在；
- 存储/服务第三批六个及车身/传感器/电源第四批四个 CadQuery 设计输入的独立
  source/contract/manifest 校验：精确 SHA-256、惰性 CadQuery、无 mesh import，且
  FCStd/STEP 均必须仍不存在；
- [`native-brep-source-coverage-audit.md`](native-brep-source-coverage-audit.md)
  的逐件 source-mesh 覆盖交叉表。八批参数化源码现已为 105/105 个项目件建立精确
  builder 映射，但这仍只是静态源码覆盖，不是内核构建或制造放行；
- [`component-addressable-native-cad-assembly-draft.md`](component-addressable-native-cad-assembly-draft.md)
  的逐件 native source/builder assembly 草案校验：105 个项目件与 21 个供应商排除项
  必须完整，但草案必须仍是 pending，且原生装配、STEP 和 export receipt 仍未交付；
- [`per-part-native-cad-release-gap-register.md`](per-part-native-cad-release-gap-register.md)
  的逐件制造输入缺口 register：105 个项目件均须有合同来源的未决门，64 个去重门和
  21 个供应商排除项只能路由后续取证工作，不能构成任何 release；
- [`native-cadquery-windows-bootstrap.md`](native-cadquery-windows-bootstrap.md)
  的串行导出合同审计：8 批来源、SHA 绑定和 105 个实际组件 ID 必须闭合，但内存、
  roundtrip、合同 release 与 CadQuery 执行必须仍未放行；
- [`s100p-formal-board-bundle.md`](s100p-formal-board-bundle.md)
  的正式快照绑定板端 bundle 审计：四项模型角色、ROS 依赖闭包和 12 项文件绑定完整，
  但 payload 复制、部署、启动及板端运行必须仍为 false；
- [`formal-static-functional-chain-audit.md`](formal-static-functional-chain-audit.md)
  的 13 项静态功能链审计与独立语义校验。

十四个检查及本聚合入口现已纳入 `scripts/ci_fast.py` 的显式纯 Python 测试清单，
并且 CI 会把存储的报告与 live `build_report()` 精确比较，拒绝陈旧报告。
聚合入口仍刻意保持独立，既不会等待缺失的运行时报告，也不会修改
`run_formal_final_acceptance.py` 的最终验收契约。

## Windows 复核

从仓库根目录执行：

```powershell
py -3 scripts/run_formal_vehicle_static_engineering_preflight.py
py -3 scripts/run_formal_vehicle_static_engineering_preflight.py `
  --output "$env:TEMP\tzcup-formal-vehicle-static-engineering-preflight.json"
py -3 -m pytest -q scripts/test_formal_vehicle_static_engineering_preflight.py `
  scripts/test_native_cad_readiness.py `
  scripts/test_native_brep_reconstruction_manifest.py `
  scripts/test_native_brep_first_batch_contract.py `
  scripts/test_native_brep_cleaning_recovery_second_batch_sources.py `
  scripts/test_validate_native_brep_cleaning_recovery_second_batch_contract.py `
  scripts/test_native_brep_source_coverage_audit.py `
  scripts/test_validate_component_addressable_native_cad_assembly_draft.py `
  scripts/test_component_addressable_native_cad_preflight_integration.py `
  scripts/test_validate_per_part_native_cad_release_gap_register.py `
  scripts/test_native_cadquery_serial_export.py `
  scripts/test_validate_native_cadquery_serial_export_contract.py `
  scripts/test_validate_s100p_formal_board_bundle.py `
  scripts/test_formal_mechanical_interface_datums.py `
  scripts/test_static_functional_chain_audit.py `
  scripts/test_validate_formal_vehicle_mechanical_manufacturing_preparation_draft.py
```

它只读取本地 YAML、CSV、XML、文本和文件名，且不会启动 WSL、Docker、Gazebo、ROS、
CAD 导出器或网格转换器。当前权威滚动报告写入
`reports/engineering/formal_vehicle_static_engineering_preflight.json`；若需要保留某次
不可变验收证据，应另存到带运行标识的 `artifacts/` 目录。

## 状态语义

当前报告应同时显示：

- `static_check_count: 14`、`static_check_completed_count: 14` 且
  `static_preflight_complete: true`：十四个静态校验器正常完成；它表示制造设计输入草案、八批参数合同/来源、105 件逐件 source/builder assembly 草案及其 64 个去重制造输入缺口、nominal datum crosswalk、逐件 source-coverage、串行原生导出合同、S100P bundle 与功能链源级证据都已按预期执行，而不是原生 CAD 已导出或板端已部署；
- `native_brep_reconstruction_manifest.summary`：105 个项目自研件保持 pending，21 个供应商参考外形显式排除，126 个生成输出无遗漏；
- `native_brep_first_batch_contract.status: "design_input_pending_native_export"`：四个工作包仍只是设计输入；`native_cad_delivery_ready: false`；
- `native_brep_cleaning_recovery_second_batch_contract.status: "design_input_pending_native_export"`：七个清洗/回收工作包同样仍只是设计输入；其 source manifest 必须验证 source 与 contract 的 SHA-256，CadQuery 必须惰性加载、不得导入 mesh，并且未来 FCStd/STEP 输出不得已经存在；
- `native_brep_source_coverage.status: "STATIC_INDIVIDUAL_COVERAGE_CLOSED"` 且
  `unproven_project_part_count: 0`：105 个项目件均有精确 source-mesh/builder 静态关联；
  `blocker_codes` 为空，但 `native_cad_delivery_ready` 必须仍为 `false`，因为尚未执行
  CadQuery、生成组件装配/STEP/回执或完成制造验证；
- `component_addressable_native_cad_assembly_draft.status: "STATIC_COMPONENT_ADDRESSABLE_DRAFT_VALID_NATIVE_EXPORT_BLOCKED"`：`component_count: 105`、`supplier_excluded_count: 21`，但 `native_cad_assembly_ready: false`；其 blocker 必须包括缺失 STEP/FCStd、真实 CadQuery 装配执行和 export receipt；
- `per_part_native_cad_release_gaps.status: "STATIC_PER_PART_RELEASE_GAPS_VALID_NATIVE_RELEASE_BLOCKED"`：`part_count: 105`、`unresolved_gate_count: 64`、`supplier_excluded_count: 21`；`native_cad_release_ready: false` 和 `manufacturing_release_ready: false` 必须同时保留。该 register 只把材料、GD&T、密封、泵、电气、质量/CoG、分析和供应商接口等合同原文缺口拆为待取证项；
- `native_cadquery_serial_export_contract.status: "STATIC_SERIAL_EXPORT_CONTRACT_VALID_NATIVE_EXPORT_BLOCKED"`：8 批来源与 105 个唯一 ID、SHA、严格串行和 4096 MiB 门已静态验证；`formal_export_ready`、`cadquery_imported` 与 `source_modules_loaded` 必须仍为 `false`；
- `s100p_formal_board_bundle.status: "BLOCKED"` 且 `validator_complete: true`：表示验证器成功证明 bundle 安全阻断；`payload_copy_authorized`、`ready_to_deploy`、`board_runtime_accepted` 与 `board_operations_performed` 必须全部为 `false`；
- `static_functional_chain.status: "STATIC_CLOSED"` 且 `required_item_count: 13`、`static_closed_count: 13`：13 条真实源级依赖链完整；它固定同时写入 `runtime_accepted: false` 与 `fresh_gazebo_runtime_required: true`，因此绝不替代新的 Gazebo 验收；
- `formal_mechanical_interface_datums.status: "STATIC_DERIVED_SNAPSHOT_BOUND_NOT_MANUFACTURING_RELEASE"`：16 个 nominal datum 与 7 个接口链已对当前 URDF snapshot 作静态交叉校验，绝不构成制造 datum/放行；
- `native_cad.outcome: "blocked"`：有效的逐件 assembly draft 会在 inventory 中被识别，
  但正式组件装配清单仍不存在，因此 blocker 精确为
  `NATIVE_ASSEMBLY_MANIFEST_DRAFT_NOT_RELEASED`；原生导出的非网格 STEP、导出回执和可用的
  Windows B-rep 导出器仍缺失；
- `native_cad.blocker_codes` 只列真正硬门；保留的 STL 视觉生成器单列在
  `native_cad.warning_codes`，既不能充当制造 CAD，也不会永久阻止已独立证明的
  原生 B-rep/STEP 闭环；
- `manufacturing_release_ready: false`、`native_export_ready: false`、
  `deployment_ready: false`：制造、原生导出和板端部署仍被明确阻断。

因此，进程退出码 0 只表示静态报告成功生成，绝不表示已经交付原生 CAD、可以采购/
加工、获得供应商批准、完成实物检验，或通过 ROS/Gazebo 运行时验收。若流水线必须要求
原生 CAD 交付就绪，应另行调用
`audit_native_cad_readiness.py --strict`；不得把本聚合入口的成功退出码改写为制造放行。
