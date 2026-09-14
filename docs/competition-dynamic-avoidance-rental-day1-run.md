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
