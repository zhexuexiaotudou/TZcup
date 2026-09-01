# S100P Windows 连接依赖只读审计

运行 `py -3 scripts/audit_s100p_windows_connectivity_dependencies.py` 会读取保留的
S100P G0 网络清单、相关 SSH 配置存在性、Windows 路由、邻居表和 Tailscale/FSE 适配器状态，
并写出 `reports/engineering/s100p_windows_connectivity_dependency_audit.json`。

该审计不连接板端：不会运行 SSH、ping、端口扫描、socket、包发送或任何板端命令；私钥和
known_hosts 公钥内容不进入报告，板端地址只按 `/24` 候选网段脱敏。路由或 ARP 的 `Stale`
记录只说明本机保留了邻居状态，不能证明当前连通。

若当前候选网段不是由 Tailscale/FSE 路由，报告只可声明“未观察到它们被路由选择”。这不证明
VPN、Hyper-V 扩展或过滤器不会影响流量，因此 `effect_on_board_connectivity` 必须保持
`UNKNOWN_NO_PACKET_OR_FILTER_EVIDENCE`，直到获得经授权的独立连接或包/过滤证据。
