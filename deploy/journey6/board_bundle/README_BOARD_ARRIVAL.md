# Journey 6 board-arrival bundle

This directory is a fail-closed deployment skeleton for the Journey 6 family. It deliberately keeps `target_sku: auto` and `target_march: auto` until the physical board and its official OpenExplorer/OE package establish those facts. RDK and S100-family SDKs or precompiled models are not compatible substitutes.

Before deployment, replace every `blocked_external` entry with evidence from the supplied Journey 6 SDK and read-only board inventory, lock the runtime ABI and versions, add checksum-locked HBM/model artifacts, and provide official sanity-model, inactive project warmup, and static parity commands in the matching march profile. A generic profile does not authorize a board deployment.

Build a checksum-locked directory without claiming readiness:

```bash
python3 scripts/build_journey6_bundle.py --output /tmp/tzcup-j6-bundle
```

The command returns exit code 2 while external blockers remain. On board arrival, first run `scripts/j6_board_inventory.sh`. Register and verify the board SSH host-key fingerprint before deployment; the entry points enforce strict host-key checking. Deployment remains a dry run unless `--execute` (Linux) or `-Execute` (PowerShell) is explicitly supplied. The installer verifies every bundle checksum, rejects unresolved or mismatched inventory/profile/runtime data, runs inactive warmup before switching, updates `active` atomically, preserves `last-known-good`, and restores it if warmup, service start, or health check fails.

No real board FPS, BPU use, CPU/DDR use, temperature, power, HBM latency, network HIL latency, or board stability result is represented by this skeleton.
