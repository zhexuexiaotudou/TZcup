# S100P 最终部署前统一硬门（本地只读）

`scripts/validate_s100p_final_predeploy.py` 是唯一的最终部署前判定入口。它消费并交叉
校验正式 bundle、离线准备审计、DOSOD HBM 编译合同、机械/供电 fail-closed 合同、PC 正式
snapshot、正在运行的 acceptance session、runtime-gate binding，以及未来现场人员回传的五类
receipt。它不连接 RDK S100P、不复制 payload、不安装依赖、不启动节点、不采集数据，也不会
生成或回填 receipt。

在进入 receipt 判定前，它还要求 bundle 的所有冻结源、payload/launch/overlay 闭包和离线计划的
完整 package.xml 依赖清单均通过；不能以较短的基础包子集绕过 `geometry_msgs`、`launch`、
`rclpy`、`ros_gz_interfaces` 或 Python 运行依赖。

```powershell
py -3 scripts/validate_s100p_final_predeploy.py --allow-blocked-exit-zero `
  --output reports/engineering/s100p_final_predeploy_audit.json
py -3 -m pytest scripts/test_validate_s100p_final_predeploy.py -q
```

默认 receipt 根目录为 `artifacts/s100p_formal_predeploy_receipts/`。目录目前不存在是预期的：
审计必须返回 `BLOCKED`，而不是用旧 smoke、G0 盘点、数据表典型功耗、PC 侧文件或空 JSON
冒充现场证据。输出文件是这次**本地审计结论**，不是 receipt，也不证明部署或运行通过。

未来在另行授权的维护窗口中，五份由现场流程真实产生并回传的 JSON 必须全部存在，且都精确
绑定同一 `acceptance_session_binding` 与 `runtime_closure_binding`：

| receipt | 必须证明的内容 |
| --- | --- |
| `dosod_hbm_compile_receipt.json` | 项目四类 DOSOD HBM 已由已验证编译器生成，目标路径、SHA-256、大小明确。 |
| `model_payload_receipt.json` | DOSOD HBM/词表及 EdgeSAM encoder/decoder 四项全部存在，目标路径、SHA-256、大小精确；DOSOD HBM 必须等于编译 receipt。 |
| `overlay_build_receipt.json` | `sanitation_perception` 与 `sanitation_perception_interfaces` 这两个 overlay 包的源/安装 hash。 |
| `runtime_dependencies_receipt.json` | package.xml 依赖及 `hobot_dosod`、`mono_edgesam` 的完整版本清单。 |
| `thermal_power_receipt.json` | 同一 identity 下至少 1800 秒的实测，峰温不高于 85 C、可用内存不低于 5%、且有正的实测输入功率。 |

PC snapshot 必须与 board bundle 的快照 SHA、source/output inventory 和 expanded URDF SHA 一致；
session 必须为 `FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING` 且含完整 runtime-closure binding；
runtime binding 的 session 文件 SHA、开始时间、snapshot/output inventory 与 closure 都必须逐字
匹配。缺失、旧格式、跨 session、跨 closure 或任一 receipt 状态不为 `VERIFIED` 都会阻断。
本地 JSON receipt 是受信任操作员的防篡改链，不是 TPM 或签名远程证明，不能替代现场访问控制。
