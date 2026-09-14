# Day-1 bounded saved-map coverage attempt

**Date:** 2026-09-14

**Revision:** `47e3cb3a7ecc01edd82aa23a3b54cbeaffc418bc`

**Status:** `BLOCKED`

**Decision:** stop before coverage; no second run

## Scope

This attempt reused the successful localization runtime, saved map, fixed start,
and rental host. The bounded region was a 48 m2 map-frame rectangle near the
fixed `(0, 0, 0)` start:

```text
[(0,-3), (8,-3), (8,3), (0,3)]
```

The mission used the optimized coverage planner at 0.55 m lane spacing and a
20-minute wall deadline. It did not attempt or claim the full 20,000 m2 mapping
result, official 95% recognition, or the official competition-efficiency gate.

## Result

The formal launch, navigation stack, and coverage server started. The cleaning
adapter failed before the coverage probe started:

```text
AttributeError: property 'publishers' of
'BoundedCoverageCleaningBridge' object has no setter
```

The adapter had attempted to assign `self.publishers`, but `rclpy.Node`
reserves that attribute. Coverage report, coverage trajectory, dirt-clearance
count, path length, speed, and efficiency were therefore not measured. The run
stopped fail-closed and no retry was launched.

The local code was corrected after the attempt by renaming the publisher
dictionary to `self.pub`. The corrected script has not been re-executed.

## Resource release

The primary rc was `4`. The formal Gazebo lock was available after the run,
the task partition had zero survivors, and `sim_resource_released` was `true`.

## Run command

```powershell
& 'F:\Project\TZcup\.workspace\tools\Invoke-TZcupRemoteLatest.ps1' -Command @'
bash /root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-bounded-coverage-20260914-01/ops/run_day1_bounded_coverage_remote_dispatch_v2.sh
'@
```

## Evidence

The compact remote archive is
`artifacts/day1_bounded_coverage_20260914/remote-day1-bounded-coverage-20260914-01.tar.gz`
with SHA-256
`516d890d28260757932eed02c90a44284f8277b1b7261268f8348362418da15f`.
The exact blocker is in `run-01/cleaning_bridge.log`; the structured receipt is
`artifacts/day1_bounded_coverage_20260914/failure_receipt.json`.

No video evidence was produced.

## Verification

Focused tests passed (`4 passed`), Python compilation passed, both new shell
entry points passed `bash -n`, and `git diff --check` passed. Repository-wide
`scripts/ci_fast.py` was blocked before test execution by the worktree's
existing `README.md must remain a concise project front door` hygiene check.
