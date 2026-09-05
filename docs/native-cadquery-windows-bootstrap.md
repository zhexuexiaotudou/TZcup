# Windows 原生 CadQuery 最小 B-rep 工具链

这是一条仅用于本机 CAD 工具链的轻量路径：锁定的 CadQuery 生成一个带通孔的极小 B-rep，导出 ISO-10303 STEP，再由 CadQuery 读回并对比实体、面、边、顶点和体积。它不调用 WSL、Docker 或 Gazebo。

它也不是整车 CAD 交付：不会创建正式车装配、部件台账、原生 CAD 源或正式证据，绝不能把通过结果写成 `native_editable_step_assembly_ready`、整车 manufacturing CAD，或任何产品 readiness 的 `ready`。

## 锁定范围

[`config/cadquery-windows-cp313.lock`](../config/cadquery-windows-cp313.lock) 是面向 CPython 3.13 / Windows amd64 的完整二进制锁。它由 PyPI 的 [CadQuery 2.8.0](https://pypi.org/project/cadquery/2.8.0/) 和 [cadquery-ocp 7.9.3.1.1](https://pypi.org/project/cadquery-ocp/7.9.3.1.1/) 元数据，以及 clean `pip --dry-run --ignore-installed --only-binary=:all:` 解析结果生成。每个解析包都有 SHA-256；bootstrap 强制 `--require-hashes` 和 `--only-binary=:all:`。

`cadquery-ocp` 的发行版本为 `7.9.3.1.1`。运行报告记录该发行版本，但不把包名版本冒充为独立实测的 OCCT 动态库版本。

## 安全运行

从仓库根目录执行：

```powershell
powershell -NoProfile -ExecutionPolicy RemoteSigned -File .\scripts\bootstrap_native_cadquery_windows.ps1
```

脚本将每一次预检、venv 创建和安装前合同检查都固定为 `py -3.13`。它不接受自定义 Python
可执行文件或 launcher 参数，因而不会跟随本机 `py -3` 的默认 3.14；预检仍会再次验证实际
运行的是 64 位 CPython 3.13。

预检要求 Windows、64 位 CPython 3.13、至少 4096 MiB 空闲物理内存，以及项目驱动器至少 8192 MiB 空闲空间。任一条件不满足时，脚本只在 `.work/cadquery-windows-preflight.json` 写入诊断并退出；不创建 venv、不安装包、不杀进程，也不尝试释放内存。

通过预检后，虚拟环境、解析报告、STEP 和 round-trip JSON 都位于已忽略的 `.work/`：

```text
.work/cadquery-venv/
.work/cadquery-windows-preflight.json
.work/cadquery-step-roundtrip/cadquery_brep_roundtrip.step
.work/cadquery-step-roundtrip/roundtrip-report.json
```

若该 venv 已存在，脚本默认拒绝覆盖；只有在人工检查其归属后，才可显式传入 `-ReuseExistingVenv`。这些本地工具链产物不是版本化整车证据。

## 受合同约束的逐件导出（尚未授权）

[`config/high_fidelity_vehicle/native_cadquery_serial_export_contract.json`](../config/high_fidelity_vehicle/native_cadquery_serial_export_contract.json) 登记了全部 8 批源码。第 1–4 批只作为来源/合同预检；重建 manifest 的逐件集合只由第 5–8 批构成，严格为 47 个车身件、23 个清扫件、34 个储存/服务件和 1 个配电箱，即 105 个可寻址组件。这样不会把早期高层 work package 重复塞进逐件装配。

在所有合同和证据都已受控释放后，才可显式追加 `-FormalExport`：

```powershell
powershell -NoProfile -ExecutionPolicy RemoteSigned -File .\scripts\bootstrap_native_cadquery_windows.ps1 -FormalExport
```

当前无需 CadQuery 的静态审计入口为：

```powershell
py -3 scripts/validate_native_cadquery_serial_export_contract.py `
  --output reports/engineering/native_cadquery_serial_export_contract_audit.json
```

当前结果必须是
`STATIC_SERIAL_EXPORT_CONTRACT_VALID_NATIVE_EXPORT_BLOCKED`：它验证 manifest 中 source/
contract SHA、前四批 provenance、后四批实际 `47+23+34+1=105` 个唯一 ID、严格串行、
无 preview 与禁止 mesh 路径，但不会导入 CadQuery 或装载任何批次源码模块。

静态审计本身不创建 venv、安装包、装载批次源码模块、评估 Windows 内存或导入 CadQuery。正式导出仅会在受控 release 合同和最低限度 round-trip 已通过后，先执行 4096 MiB Windows 资源门；每个 source batch 开始前都会复检该门。任何非空 pending_* 字段、source manifest SHA 漂移或未发布状态都会在 CadQuery/source module 之前被拒绝；孔位、材料、公差、密封、热、EMC、电气安全、运行验收等 pending 不会被此路径标为 ready。只有随后最低限度 round-trip 通过，才会按一个组件一次的顺序构建、检查 B-rep 有效性、导出并读回 STEP。STEP 检查以固定大小分块扫描整个文件，因此 256 KiB 之后出现的 FACETED_BREP/TESSELLATED 也会阻断发布，而不是只检查文件头。

逐件 JSON 日志与 checkpoint 都以同目录临时文件加原子替换写入 .work/.<output>.incomplete。任一件失败会保留该件、此前的 STEP/日志及 hash-bound serial checkpoint；不会发布最终 assembly 或 SHA-256 receipt。再次尝试必须显式传入 --resume，且只接受当前 released source-record SHA、完整连续组件前缀、每个已完成 STEP 的路径/哈希/非网格 header 均一致的 retained directory；任何漂移、乱序、链接或损坏都会拒绝恢复。构建对象、单件 re-import 和 batch module 在每件/批次后显式释放并 GC，但最终 component-addressable assembly 仍是有意保留到发布前的聚合对象，因此 4096 MiB 门会在每批边界重新执行。只有 105 件全通过时才会原子发布组件可寻址的 native_component_addressable_assembly.step 与 sha256-receipt.json。该 receipt 仍只说明原生几何导出过程，不改变产品、制造或功能 readiness。

本版本不提供 preview 模式，避免将设计输入误写入正式输出或错误地纳入 native readiness。未来若增加 preview，只能输出到独立的 `.work` draft 目录，并显式标记为不被 native readiness 接受。
