# Rental dynamic-avoidance single-run attempt

**Date:** 2026-09-14

**Requested revision:** `31f74a0883e87939356a7ebad34274f8716c532d`

**Status:** `BLOCKED_BEFORE_GAZEBO`

**Official metric:** `NOT_MEASURED`

## Scope

The committed single-run protocol from `c768a47` was cherry-picked onto the
localization-run source as `31f74a0`. One bounded trial was started on the
existing AutoDL RTX 3080 Ti instance using:

* runtime:
  `/root/autodl-tmp/tzcup-competition-sim-only-20260912/runtime/runtime-ws-1a211400-lifecycle-health-v4-r1-27cac7f7773f`
* ROS domain: `89`
* Gazebo partition: `tzcup_dynamic_avoidance_single_run_20260914_01`
* XDG runtime:
  `/tmp/tzcup_dynamic_avoidance_single_run_20260914_01_xdg`
* run root:
  `/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-dynamic-avoidance-run-20260914-01/run-01`

No rebuild was performed. The static runtime/source binding check passed before
the trial.

## Result

The wrapper froze `predeclared_obstacle_route.json` and recorded `STARTED`.
The fresh runtime-closure verifier then aborted because the trial environment
did not contain `FORMAL_ROS2_EXECUTABLE`. Gazebo was not launched. The
single-run evaluator ran against the retained evidence and reported:

| Field | Value |
|---|---:|
| Single-run numerator | 0 |
| Single-run denominator | 1 |
| Observed fraction | 0.0 |
| Functional pass | false |
| Official measured value | null |
| Official status | `NOT_MEASURED` |

This one started trial is an infrastructure failure. It does not establish the
official `>=95%` rate; twenty trials per scenario and the recommended 100-run
campaign remain unperformed.

## Resources and evidence

Immediate post-attempt sampling recorded CPU `27.10%`, GPU `0%`, GPU memory
`1 MiB`, no GPU compute processes, an available formal Gazebo lock, and no
domain-89 runtime process. The local compact archive is
`artifacts/day1_dynamic_avoidance_20260914/remote-day1-dynamic-avoidance-run-20260914-01-compact.tar.gz`.
The full remote evidence and sealed runtime closure remain under the remote run
root above.

## Corrected retry

The authorized retry fixed the missing executable environment:

* verified `/opt/ros/jazzy/bin/ros2` as a Python executable;
* verified `file`, `ros2 --help`, and SHA-256
  `3308327fcefeaefe5601f72e0b473a5f9f9638434f5773d2f40b124f7c9aa04d`;
* added and committed wrapper export logic plus a focused contract test;
* wrapper/test commit: `ab6989b79f46a99819a77a77cd5cbaa5250ffbd6`;
* focused tests: `13 passed`.

The corrected trial ran as `run-04` on ROS domain `92`, Gazebo partition
`tzcup_dynamic_avoidance_single_run_20260914_04`, and XDG
`/tmp/tzcup_dynamic_avoidance_single_run_20260914_04_xdg`. It recorded
`STARTED` and retained denominator `1`, but stopped before Gazebo because the
runtime closure drifted in `source_inventory` and
`source_inventory_sha256` after the wrapper and test files changed. This is a
distinct pre-Gazebo blocker; no further trial was run.

`run-04` therefore has numerator `0`, evaluator status
`SINGLE_RUN_EVIDENCE_INVALID`, and official `>=95%` status `NOT_MEASURED`.
No MCAP, contact, clearance, reroute, wait, or goal-recovery evidence exists
because Gazebo did not launch. Immediate post-attempt sampling recorded CPU
`27.56%`, GPU `0%`, GPU memory `1 MiB`, no runtime processes, an available
formal Gazebo lock, and resources released.

## Frozen-source reclosured run

The current wrapper/source tree was fingerprinted before reclosuring:

* source tree SHA-256:
  `7cdfd0b5957dff87e7ebcb2ce6e4cb93e20dc86e2fd84144bee24903ce9a5b5b`;
* wrapper SHA-256:
  `c44e91b42e310a8966865fab62a5adc02c5be57317fc6ab3483400cc0701491d`;
* test SHA-256:
  `a5c1f539091f499c41aef236c43027c0dd4c96078386cc1a38ed9a06249814cb`.

The established closure tooling recorded
`final_runtime_closure_manifest_run05.json` and then verified it with
`verify-recorded`. Verification passed with:

* status: `FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED`;
* manifest SHA-256:
  `e21c7546341d5bb20aba8e9cba4efe2ea37838a962129071c1d4f322aa281aec`;
* closure SHA-256:
  `6a53522b0a094f7986eb3e9b5399804103f29fa9941e2b86e8ce6f4735bca94f`;
* 20 runtime packages, 1,551 source files, and 1,805 install files;
* the source tree hash was unchanged after verification.

The run-05 wrapper then recorded `STARTED`, but the formal runner could not
acquire `/tmp/tzcup_formal_gazebo.lock`; another formal Gazebo acceptance
owned it. This is a distinct pre-Gazebo blocker. Per the run contract, the
attempt was retained and no further trial was started.

`run-05` has numerator `0`, denominator `1`, evaluator status
`SINGLE_RUN_EVIDENCE_INVALID`, and official `>=95%` status `NOT_MEASURED`.
Gazebo did not launch, so there are no MCAP, contact, clearance, reroute, wait,
or goal-recovery samples. The post-attempt sample recorded CPU `21.83%`, GPU
`0%`, GPU memory `334 MiB`, no GPU compute processes, no runtime processes,
the formal lock available again, and resources released. The lock was not
cleared by this work.

## Run 06

The rental card was idle, the formal lock was absent, and the verified
`final_runtime_closure_manifest_run05.json` was reused unchanged. Run 06 used
ROS domain `94`, Gazebo partition
`tzcup_dynamic_avoidance_single_run_20260914_06`, and XDG
`/tmp/tzcup_dynamic_avoidance_single_run_20260914_06_xdg`.

The wrapper froze the obstacle route and the formal runtime gate bound the
verified closure successfully. The run then stopped before Gazebo because no
qualified `map_lifecycle_manifest.json` saved-map lifecycle artifact exists on
the rental card. This is the new distinct pre-Gazebo blocker; no further trial
was run.

Run 06 has numerator `0`, denominator `1`, evaluator status
`SINGLE_RUN_EVIDENCE_INVALID`, and official `>=95%` status `NOT_MEASURED`.
Gazebo did not launch, so no MCAP, contact, clearance, reroute, wait, or
goal-recovery evidence exists. The post-attempt sample recorded CPU `10.03%`,
GPU `0%`, GPU memory `1 MiB`, no runtime processes, the formal lock available,
and resources released.

## Run 15: non-official functional smoke

The run-09 naming contract was fixed so the runner now writes
`pedestrian_schedule.seed.<seed>.json`, matching the evaluator. A separate
protocol with mode `FUNCTIONAL_SMOKE_NOT_OFFICIAL_95` and nominal leg `6.0 m`
was added at
`config/dynamic_avoidance_functional_smoke_protocol.json`. Its official metric
remains `NOT_MEASURED`.

One bounded smoke attempt was executed from code commit
`ed3a802b298fbfbe697eb22c8efedf8289a1b664` using the existing runtime closure
revision, ROS domain `100`, partition
`tzcup_dynamic_avoidance_smoke_20260914_15`, fresh XDG runtime state, and run
root
`/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-dynamic-avoidance-smoke-20260914-15/run-15`.
The fresh closure and RUNNING binding session passed, and the focused smoke
contract tests passed `16/16`.

The attempt stopped before Gazebo while materializing the offline raycast map
source. The Windows-generated overlay archive converted JSON/YAML text files
to CRLF. Their hashes therefore differed from the LF provenance manifest even
though the source commit and binary PGM matched:

| Artifact | Expected SHA-256 | Observed SHA-256 |
|---|---|---|
| `offline_raycast_manifest.json` | `ed4e52fe...f3cc49` | `69718c19...d262c0` |
| `occupancy.yaml` | `78f39b80...e89538` | `e5de8252...0f3673` |
| `map_area_verification.json` | `c739ef43...318cac` | `19429d4a...daeb48` |
| `occupancy.pgm` | `8b74f368...4586a3` | matched |

The wrapper exited `1`; Gazebo and both collectors did not start. Therefore
there is no MCAP, contact, clearance, reroute, wait, goal-recovery, or
short-mission success result. The single-run result is not a valid functional
smoke pass, and official `>=95%` remains `NOT_MEASURED`. No retry was launched.

Post-attempt sampling recorded CPU `9.55%`, GPU `0%`, GPU memory `1 MiB` of
`12288 MiB`, no remaining runtime processes, no formal-Gazebo lock holder, and
resources released. Compact evidence is retained at
`artifacts/day1_dynamic_avoidance_functional_smoke_20260914/remote/`; the
archive SHA-256 is
`fc056f063421ac239d2b838908a4d03baa57f89d59fbd54ae3276a10514ad844`.

## Run 16: final non-official smoke retry

The LF/CRLF fix was committed as
`b6183e037ea49ea3ccb664582144826a805f6353`. Text map artifacts are normalized
to LF before validation, the PGM remains byte-identical, and the frozen
provenance values remain unchanged.

One final smoke attempt used ROS domain `101`, partition
`tzcup_dynamic_avoidance_smoke_20260914_16`, fresh XDG state, and run root
`/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-dynamic-avoidance-smoke-20260914-16/run-16`.
Offline-map preparation passed:

| Evidence | Value |
|---|---|
| Mode | `OFFLINE_MAP_SOURCE` |
| PGM SHA-256 | `8b74f368...4586a3` |
| Runtime origin | `[-7.0, -55.0, 0.0]` |
| Frozen manifest hash | `ed4e52fe...f3cc49` |
| LF manifest hash | `69718c19...d262c0` |

The wrapper then stopped before Gazebo in `prepare_dynamic_avoidance_single_run`
with `ValueError: could not place enough obstacle-free mission crossings`.
Gazebo and both collectors did not start. There is therefore no command-chain,
contact, clearance, reroute, wait, goal-recovery, or short-mission result; the
single-run denominator remains `0`, and official `>=95%` remains
`NOT_MEASURED`. No retry was launched.

Post-attempt sampling recorded CPU `10.32%`, GPU `0%`, GPU memory `1 MiB` of
`12288 MiB`, no remaining runtime processes, no formal-Gazebo lock holder, and
resources released. The evidence archive SHA-256 is
`0a32f88cfde66384ddf2295ab8dd12b740e0ba09b8e6cc108be951911e5e1909`.

## Run 17b: static 6 m route materialization and smoke attempt

The run-16 failure was reproduced offline against the exact public episode
manifest, world, and base schedule. The three-crossing layout is impossible for
the central 60% of a 6.0 m mission: three centers separated by at least 4.0 m
require an 8.0 m span, but only 3.6 m is available. Two crossings are likewise
impossible in that corridor. This is a geometry constraint, not a seed failure.

The smoke protocol now declares one mission-corridor crossing and both schedule
materializations receive the same crossing count and corridor range. Official
30.0 m protocol behavior remains three crossings. The obstacle radius, 0.12 m
surface-clearance threshold, collision definitions, collision-monitor
intervention, reroute, and goal-recovery requirements are unchanged.

The static search selected:

| Field | Value |
|---|---|
| Seed | `2026091401` |
| Corridor | declared `0.20..0.80` |
| Crossing count | `1` |
| Walker | `walker_eb2f451c21ba` |
| Mission-local crossing center | `x = 3.63 m` |
| First centerline-crossing time | `5.16295 s` |
| Estimated nominal surface gap at first crossing | `0.518156 m` |
| Minimum static surface clearance | `3.732629 m` |

The selected frozen search evidence is
`artifacts/day1_dynamic_avoidance_functional_smoke_route_20260914/static_route_search.json`.
It records the full source-world and mission-local route, the predeclared route
manifest, the geometric exclusion proof, and the static-input hashes. Gazebo was
not started during this static materialization step.

The fresh remote preparation used a copied source tree, `28` overlay files,
`53` focused tests, a fresh runtime closure, and a RUNNING non-official session.
The remote static search independently selected the same seed, walker, crossing
count, and timing, with evidence SHA-256
`92d6b431ad287ae9358cffda7ada2fe566805e741a0daec290e3dc8360d47c3c`.

The one authorized smoke attempt then froze
`predeclared_obstacle_route.json` before `STARTED`, but stopped before Gazebo in
`formal_runtime_configure` because ROS domain `103` is outside the repository's
bounded DDS policy (`0..101` or `215..231`). The evaluator retained a single-run
denominator of `1`, numerator `0`, and status `SINGLE_RUN_EVIDENCE_INVALID`.
Because the wrapper had recorded `STARTED`, no retry was launched.

Gazebo and both collectors did not start. There is no command-chain, contact,
clearance, reroute, wait, or goal-recovery evidence from this run. Post-attempt
sampling found no Gazebo/ROS process or formal-lock holder, GPU utilization `0%`,
and `1 MiB` GPU memory in use. The compact evidence is
`artifacts/day1_dynamic_avoidance_functional_smoke_20260914/remote/run17b-evidence.tar.gz`
with SHA-256
`4a8f95d029d19138566d7ccb5402197ba76ac9d5ed30420821596c4874f6e790`.
Official `>=95%` remains `NOT_MEASURED`.

## Run 18b: binding repair and live smoke outcome

Run 17c reached `formal_runtime_gate_binding.py:94` and failed because the
RUNNING session copied `verify-recorded` output without
`runtime_closure_binding.runtime_install_root`, while the gate added that
canonical field before strict comparison. Commit `ff7375b` now makes
`verify_recorded_manifest()` return the canonical install root itself, so the
CLI verification result, session generation, and gate all consume the same
complete closure object. The focused regression is
`scripts/test_formal_final_runtime_closure.py::test_recorded_verifier_binds_the_canonical_runtime_install_root`.

The repaired source was prepared as `ff7375b-r2`, with fresh snapshot,
closure, canonical RUNNING session, and a preflight binding. The preflight
returned `FORMAL_RUNTIME_GATE_BOUND`, and the session and preflight closure
bindings matched exactly. The one authorized smoke trial used ROS domain `215`,
partition `tzcup_dynamic_avoidance_smoke_20260914_18b`, fresh XDG state, and
run root
`/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-dynamic-avoidance-smoke-20260914-18b/run-18b`.

Gazebo started and the run reached the 120-second task timeout. The retained
product telemetry contains the full command chain with nonzero samples, front
and rear contact samples, `91` map-pose samples, and `2.0425 m` of physical
travel. There were zero physical collisions and zero geofence violations.
However, no predeclared obstacle interaction candidate was selected, the
minimum surface gap remained unmeasured, the maximum cross-track detour was
only `0.0564 m`, collision-monitor interventions were `0`, and Nav2 did not
reach a terminal success status. The offline evaluator therefore reports
`SINGLE_RUN_EVIDENCE_INVALID`, numerator `0`, denominator `1`, and observed
fraction `0.0`.

The final archive is
`artifacts/day1_dynamic_avoidance_functional_smoke_20260914/remote/run18b-evidence-final.tar.gz`
with SHA-256
`36ffcaf9ad97bee15db0534e0e3b97d019045722d57ad3358a56bb5c4be5ab1a`.
Post-attempt sampling recorded GPU utilization `0%`, GPU memory `1 MiB` of
`12288 MiB`, no formal-lock holders, and no remaining runtime process.
Official `>=95%` remains `NOT_MEASURED`.

## Run 19b: static interaction materialization and live evaluator clock blocker

Run 18b exposed two independent issues: the static route proof used the
corridor midpoint rather than the exact centerline crossing, and the 6 m smoke
used a 120 s wall budget that ended before the measured PRoot simulation rate
could complete the route. The smoke protocol now declares corridor fraction
`0.30..0.35`, seed `2026091715`, and a bounded 600 s task / 660 s wall budget.
The obstacle radius, `0.12 m` minimum surface clearance, collision definition,
safety gate, and sampling thresholds are unchanged.

The frozen route manifest now requires a
`STATIC_OBSTACLE_INTERACTION_SAMPLEABLE` proof. The selected route is:

| Field | Value |
|---|---:|
| Seed | `2026091715` |
| Route heading | `positive_y` |
| Centerline crossing | `x = 1.8825 m` |
| Centerline crossing time | `4.5866 s` |
| Static evaluator sample | `2.4966 s` |
| Trigger delta at sample | `2.09 s` |
| Surface gap at sample | `1.0286 m` |
| Status-clock alignment | `0.0034 s` |
| Predicted envelope encounter in monitor horizon | `0.7364 m` |

The offline search enumerated `64` seeds in the declared corridor: `15`
candidates passed the static sampleability proof and `49` were retained as
failures. The frozen search evidence is
`artifacts/day1_dynamic_avoidance_functional_smoke_route_20260914/static_route_search_run19.json`.

Run 19b used ROS domain `217`, Gazebo partition
`tzcup_dynamic_avoidance_smoke_20260914_19b`, fresh XDG state, and fresh run
root `.../day1-dynamic-avoidance-smoke-20260914-19b/run-19b`. The runtime did
produce a selected obstacle interaction:

| Raw metric | Value |
|---|---:|
| Selected interaction candidates | `1` |
| Collision-monitor interventions | `1` |
| Smoothed command | `0.45 m/s` |
| Gated command | `0.225 m/s` |
| Nearest scan range | `0.7500 m` |
| Vehicle pose at candidate | `[0.8983, -0.0027] m` |
| Physical collisions | `0` |
| Walker pose frames / active walkers | `57 / 8` |
| Physical travel | `3.1376 m` |
| Maximum cross-track detour | `0.1110 m` |
| Nav2 goal result | `not succeeded` |

The offline evaluator still returned `SINGLE_RUN_EVIDENCE_INVALID` with
numerator `0`, denominator `1`. The selected candidate was not matched because
the product collector timestamp was Unix epoch
`1789386411749680004 ns`, while the evaluator-only environment status stream
used simulation time from `100000000 ns` to `28900000000 ns`. Their nearest
absolute alignment was `1789386382.8496802 s`, far above the unchanged `0.5 s`
limit; therefore `minimum_surface_gap_m`, trigger alignment, and selected
candidate count remained unmeasured in the evaluator report despite the raw
interaction. Empty collector logs, `collision_monitor_state` count `2 < 5`,
physical travel below `5 m`, and non-successful Nav2 recovery also keep the
single-run result invalid. No second Gazebo attempt was launched.

The sealed evidence archive is
`artifacts/day1_dynamic_avoidance_functional_smoke_20260914/remote/run19b-final-evidence.tar.gz`
with SHA-256
`21e923e70b9e49c17b667227160f28fa24612bf4fd77fb42c7568e1e5198d6af`.
Official `>=95%` remains `NOT_MEASURED`.
