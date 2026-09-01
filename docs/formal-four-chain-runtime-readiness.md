# 四条正式 Gazebo 验收链的新鲜度审计

`scripts/audit_formal_four_chain_runtime_readiness.py` 是只读、低内存的准备度审计，覆盖前进/制动、物理抓取投箱、地面脏污清扫和积水/漏液回收。它不会启动 WSL、Docker、Gazebo 或 CadQuery，也不会生成、恢复或覆盖任何 runtime report。

审计从当前 `reports/engineering/formal_vehicle_snapshot_manifest.json` 读取并校验 expanded-URDF、source inventory、output inventory 与 manifest 自身的 SHA-256。每条链只有在统一验收合同指定的正式报告、同值的 runtime-binding sidecar、活跃 acceptance session 与当前 snapshot 全部匹配，且报告和 sidecar 的修改时间不早于该 session 时，才会是 `FRESH`。正式 session 在启动时必须已经验证并写入 frozen closure identity；随后每条 sidecar 都必须与这个 session 绑定逐字段相同。审计还会核对 session manifest SHA-256 与启动时间、sidecar 的验证时间，以及 frozen closure manifest 的 schema v6、合同版本、规范绝对路径、文件 SHA-256、closure SHA-256、完整运行包集合、runtime/install 根关系和零符号链接声明；四条链也必须绑定同一个 closure identity。报告或 sidecar 是符号链接同样会被拒绝。正式报告还必须保留合同规定的 PASS 状态和运行时闭环绑定。

缺失、旧 session、旧 snapshot、缺 sidecar、sidecar 与报告不一致，或仅有历史 PASS JSON 时都会是 `FORMAL_FOUR_CHAIN_RUNTIME_BLOCKED`；旧 artifact 只能作为历史参考，不能替代一次新的 Gazebo 正式验收。

```powershell
py -3 scripts/audit_formal_four_chain_runtime_readiness.py `
  --output reports/engineering/formal_four_chain_runtime_readiness.json
```

该 JSON 是四条运行证据的新鲜度报告，不代替运行时通过证据。正确顺序是在统一正式验收创建的同一个 `RUNNING`、snapshot/closure 绑定 session 中先执行四条 runner，再在 session 终结前运行此审计；此时四条链都为 `FRESH` 才可继续完成整体验收。session 转为 `COMPLETE` 后，本审计按设计不再把它称为“活跃运行证据”，最终留存结果应由统一编排器的 complete-session 哈希复核证明。
