# 原生 B-rep 参数化源码逐件覆盖审计

`scripts/audit_native_brep_source_coverage.py` 将
`config/high_fidelity_vehicle/native_brep_reconstruction_manifest.json` 中的 105 个项目自研件，
与八批 CadQuery 设计输入合同、源码和 source-manifest 做只读交叉表。它只解析 JSON 和
Python AST、核对 source-manifest SHA-256；不会导入 CadQuery、启动 WSL/Gazebo/Docker、
创建 FCStd/STEP，或读取/转换 STL。

```powershell
py -3 .\scripts\audit_native_brep_source_coverage.py `
  --output .\reports\engineering\native_brep_source_coverage_audit.json
py -3 -m pytest .\scripts\test_native_brep_source_coverage_audit.py -q
```

## 分类规则

- `EXPLICIT_PARAMETRIC_SOURCE_COVERAGE`：某批合同的 `source_mesh`/`source_asset`
  精确列出清单行的 `source_mesh`，且该批 source-manifest 哈希与每个合同声明的命名 builder
  函数都静态成立。这是唯一的逐件正向证据；仍不表示已导出或制造。
- `HIGH_LEVEL_COMPONENT_RELATED_UNPROVEN`：合同只在组件/工作包或 profile 层级相关。
  审计会给出可能的批次和工作包，但合同没有该清单 part ID 或精确 source mesh，因此绝不提升
  为逐件覆盖。
- `COMPLETELY_UNCOVERED`：已注册批次中既没有精确 mesh 证据，也没有项目定义的组件/profile
  关联。
- `SUPPLIER_EXCLUDED`：原清单的第三方/自有参考外形；它们不是项目自研件，也不会被
  CadQuery 批次覆盖。

`status` 只有在每个项目件都取得精确逐件证据、所有 source hash 和 builder 检查成立时才会
是 `STATIC_INDIVIDUAL_COVERAGE_CLOSED`。当前八批已为 105/105 个项目自制件建立精确
source-mesh 到参数化 builder 的静态映射，因此逐件源码覆盖状态已闭合；这不等于 CadQuery
已执行、几何可成功构建、STEP/FCStd 已导出、装配/制造已批准或仿真已验收。
`runtime_accepted` 和 `native_cad_delivery_accepted` 仍固定为 `false`，原 reconstruction
manifest 也继续保持 `pending_native_brep_reconstruction`，直到原生导出及后续门禁完成。
