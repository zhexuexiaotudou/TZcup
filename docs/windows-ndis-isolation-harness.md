# Windows NDIS isolation harness

`scripts/prepare_windows_ndis_isolation.ps1` prepares a narrowly scoped Windows diagnosis. Its default is read-only dry-run; it only collects route, memory, and exact target state, then writes `reports/engineering/windows_ndis_isolation_harness_dry_run.json`.

The two allowlisted targets are exact identities, not a name pattern:

- `Tailscale`: adapter `Tailscale` and service `Tailscale`.
- `FSE`: adapters `vEthernet (FSE HostVnic)` and `vSwitch (FSE Switch)`.

It never selects a generic VPN, Ethernet, Wi-Fi, Intel, or Realtek item. Before both dry-run preparation and any execution request, it resolves IPv4 default-route interface indexes. It rejects a selected target if it owns a default route and also hard-rejects the exact local WLAN, AX211, and RTL8125 identities. A refusal is a diagnosis result, not a reason to broaden target matching.

## Safe invocation used for this task

```powershell
pwsh -File scripts/prepare_windows_ndis_isolation.ps1 -Target Tailscale
pwsh -File scripts/prepare_windows_ndis_isolation.ps1 -Target FSE
```

Neither command changes service or adapter state. Each report records the target's before/after memory snapshot, adapter/service state, default-route protection result, and the recovery plan.

## Explicit execution gate

An actual state change requires both `-Execute` and one exact target. This is intentionally not run as part of automated tests or normal diagnostics. The script records pre-isolation, post-isolation, and post-restore snapshots. Tailscale isolation stops only the exact `Tailscale` service when it began running; FSE isolation disables only the two exact FSE virtual adapters that were initially enabled.

`-IsolationSeconds` is strictly limited to 1–60 seconds (default 15). Only a validated `-Execute` request sleeps; the report records both requested and actual elapsed isolation seconds before taking the post-isolation memory snapshot.

All state changes live inside `try`/`finally`. FSE adapters are re-enabled in reverse disable order. The finalizer restores only resources it changed, records exact recovery commands, and compares every originally `Running` service and `Up` adapter with the post-restore state. Any restore, post-restore snapshot, or state-drift failure exits nonzero (`2`). Do not use the harness as a substitute for PoolMon tag attribution; it only supports a controlled comparison once separate approval is given.
