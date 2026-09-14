# Day-1 first-map run-06 timeout

**Date:** 2026-09-14

**Run root:** `/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-mapping-run-20260914-06`

**Branch:** `codex/day1-mapping-run`

**Revision:** `6021e7b52ebf6a56aa7113b7b6d7742cbdfe0692`

**Decision:** `NO-GO / TIMEOUT`

**Final map result:** `NOT_MEASURED`

## Result

The bounded first-map lifecycle reached its configured 3,600-second timeout and
exited with code `124`. The authoritative runner timestamps are:

| Field | Value |
|---|---:|
| Start | `2026-09-14T02:44:22Z` |
| Finish | `2026-09-14T03:44:56Z` |
| Runner wall time | `3634 s` |
| Exit code | `124` |
| Last observed simulation time | `303.002 s` |
| RTF over runner wall time | `0.083380` |
| RTF over the monitored interval | `0.084806` |

No `occupancy.yaml`, `occupancy.pgm`, `map_lifecycle_manifest.json`, or
`mapping_runtime.json` was produced. The committed map-area verifier therefore
could not measure a SLAM map against the `20,000 m2` threshold. The area gate
and the complete mapping/lifecycle quality gate are `NOT_MEASURED`, not PASS and
not a numeric FAIL.

The run retained 192 mapping checkpoints. The final explorer state included a
`progress_timeout` cancellation. The collector then encountered an invalid rcl
context during shutdown. These observations describe the timeout path but do
not substitute for a sealed map or a completed map-quality report.

## Partial artifacts

Only the pre-run geofence and neutral-speed masks were available as PGM/YAML
pairs:

| Artifact | Resolution | Known area | SLAM quality gate |
|---|---:|---:|---|
| `geofence_keepout.yaml` | `0.25 m` | `21216.0 m2` | FAIL (`resolution > 0.05 m`) |
| `neutral_speed.yaml` | `0.25 m` | `21216.0 m2` | FAIL (`resolution > 0.05 m`) |

Those masks are not SLAM maps and were excluded from the `20,000 m2` map gate.
No scale-up, synthetic map, or mask substitution was performed.

## Resource use and release

The live `gz sim` PID was `972580`. Monitored samples showed approximately
`151-154%` CPU for that process, `2.203 GiB` RSS, and the run-side GPU CSV
recorded `0-6%` GPU utilization, a peak of `334 MiB` GPU memory, and a peak of
`97.09 W`. The CPU/memory CSV stopped updating before Gazebo started, so the
process samples and load average range (`9.57-18.08`) are the usable CPU
evidence.

The process exited after the timeout. A post-exit census at
`2026-09-14T03:47:21Z` found:

* PID `972580`: absent
* run-owned `gz`, `ruby`, and ROS processes: zero
* `/tmp/tzcup_formal_gazebo.lock`: available for immediate reacquisition
* GPU utilization and memory: `0%` / `1 MiB`
* `SIM_RESOURCE_RELEASED=true`

No orphan process remained at the final census, so no kill was issued. The
timeout/partial run evidence remains preserved at the remote run root and in
the local retained archive described by
`reports/mapping/day1_mapping_run_06_timeout.json`.

## Boundary

This record does not claim that the 20,000 m2 area was unreachable, that the
partial mask area is a map result, or that the run passed any map-quality gate.
A future attempt requires a fresh source-bound session/run root and must
produce a sealed `occupancy.pgm`/`occupancy.yaml` plus the complete lifecycle
manifest before area or quality acceptance is possible.
