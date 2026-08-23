# Journey 6 board deployment

## Bundle contract

The deployable directory is `deploy/journey6/board_bundle`. Git contains only
schemas, profiles, services, scripts, notices, and manifests. SDK archives,
HBM/ONNX models, calibration images, raw bags, and board credentials remain in
the external artifact root.

The generated bundle must contain:

```text
bundle_manifest.json
board_profile.schema.yaml
profiles/
models/                 installed from SHA-locked artifact root
configs/
launch/
bin/
lib/
systemd/
scripts/
licenses/
SBOM.json
SHA256SUMS
README_BOARD_ARRIVAL.md
```

Bundle verification requires `target_family=journey6`, exact file hashes,
installable model locks, a runtime/profile match, health checks, and a verified
rollback contract. A directory skeleton alone is not
`J6_DEPLOYMENT_BUNDLE_READY`.

## Deployment transaction

Deployment defaults to inspection/dry-run. The explicit apply path performs:

1. read-only board inventory;
2. profile selection or rejection from actual SKU/march/runtime;
3. checksum and compatibility verification;
4. official J6 sanity HBM;
5. candidate install into an inactive version directory;
6. inactive warmup and static-image parity;
7. health check;
8. atomic `active` switch while preserving `last-known-good`;
9. evidence collection;
10. automatic rollback on failure.

No credential, host key, SDK archive, or proprietary runtime library is stored
in Git. The Windows and Linux entry points require the operator to provide the
board host and user explicitly.

Physical-board FPS, BPU/CPU/DDR use, temperature, power, HBM latency, Ethernet
HIL latency, and 30-seed results remain `null` or `not_run` until measured on
the identified board.
