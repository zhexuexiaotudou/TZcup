# Day-1 bounded saved-map coverage attempt

**Date:** 2026-09-14

**Latest attempt revision:** `be499a9174dd013a756e6bafd382da40772ccb55`

**Status:** `BLOCKED`

**Decision:** retain the corrected-run blocker; no further Gazebo run

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

## Corrected run 02

One corrected run was authorized after commit
`8eb01907516194443866cc3a2c93bf83d0df116e`. It used:

* run root: `day1-bounded-coverage-20260914-02`
* ROS domain: `93`
* Gazebo partition: `tzcup_day1_bounded_coverage_20260914_02`
* XDG runtime: `/tmp/tzcup_day1_bounded_coverage_20260914_02_xdg`

The regression test confirmed that the bridge no longer assigns the reserved
`rclpy.Node.publishers` attribute. The corrected bridge process started and
remained alive. It did not reach actuator-ready within the 180-second readiness
window, and the runner exited fail-closed with:

```text
cleaning actuators never became ready
```

During cleanup, `BoundedCoverageCleaningBridge._tick` raised
`KeyboardInterrupt` from the timer callback while `finalize()` called
`rclpy.spin_once`. That masked the bridge's structured status JSON. The
coverage probe was not started. Coverage, path, dirt, brush, and final-state
metrics remain `NOT_MEASURED`.

```powershell
& 'F:\Project\TZcup\.workspace\tools\Invoke-TZcupRemoteLatest.ps1' -Command @'
TZCUP_DAY1_COVERAGE_RUN_ID=day1-bounded-coverage-20260914-02 \
TZCUP_DAY1_COVERAGE_DOMAIN_ID=93 \
TZCUP_DAY1_COVERAGE_PARTITION=tzcup_day1_bounded_coverage_20260914_02 \
TZCUP_DAY1_COVERAGE_XDG_RUNTIME=/tmp/tzcup_day1_bounded_coverage_20260914_02_xdg \
bash /root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-bounded-coverage-20260914-02/ops/run_day1_bounded_coverage_remote_dispatch.sh
'@
```

The corrected run released its resources: primary rc `4`, Gazebo lock
available, zero task-partition survivors, and `sim_resource_released=true`.
No second corrected run was launched. The structured receipt is
`artifacts/day1_bounded_coverage_20260914/failure_receipt_run02.json`, and the
archive SHA-256 is
`185f052094695a7efb699c3c736e559a7a691531c34de008f49635a3f6592bc9`.

The exact Run-02 root cause is a circular readiness condition. The bridge
required `left_ready`, `right_ready`, and `roller_ready` before writing
`cleaning_bridge_ready.json`. Those flags require nonzero brush velocity, but
the bridge intentionally sends zero brush commands until `/brush_enabled` is
true. `/brush_enabled` is published only by the coverage probe, which the
runner refuses to start until the readiness file exists.

## Corrected run 03

Commit `be499a9174dd013a756e6bafd382da40772ccb55` removed that circular
condition. Pre-coverage readiness now requires a live GroundDirt status/ledger,
the safety permit, the `0.095 m` work-pose lift, and valid left/right/roller
contact clearances with brushes off. The full rotating-tool ready flags remain
mandatory in the post-coverage summary. The cleanup-time timer
`KeyboardInterrupt` was also removed so the final bridge report survives.

Run 03 used:

* run root: `day1-bounded-coverage-20260914-03`
* ROS domain: `92`
* Gazebo partition: `tzcup_day1_bounded_coverage_20260914_03`
* XDG runtime: `/tmp/tzcup_day1_bounded_coverage_20260914_03_xdg`

The circular blocker is confirmed resolved: the bridge received 503 GroundDirt
status samples and observed a true safety permit at simulated second `3.338`.
The remaining timeout is a separate motion-cadence defect. The repaired bridge
reissued the `20.9 s` lift trajectory on a `0.5 s` wall timer, but simulation
advanced roughly seven times slower than wall time. The controller therefore
received 349 overlapping requests and never completed interpolation. At
simulated second `27.201`, lift was `0.035695123 m` instead of the required
`0.095 m`; clearances remained `0.0643 m` and all three brush velocities were
zero. The 180-second wall readiness window expired before work pose was reached,
so the coverage probe never started.

The post-run correction restores a single bounded lift request after the
controller subscription exists and preserves permit-ever evidence in the final
bridge report. That correction has not been executed because the one authorized
corrected run is spent. Coverage report, path, dirt-clearance delta, and
efficiency remain `NOT_MEASURED`; final cleaning state is zero cells cleared,
brush disabled, and dirt system disabled.

```powershell
& 'F:\Project\TZcup\.workspace\tools\Invoke-TZcupRemoteLatest.ps1' -Command @'
TZCUP_DAY1_COVERAGE_RUN_ID=day1-bounded-coverage-20260914-03 \
TZCUP_DAY1_COVERAGE_DOMAIN_ID=92 \
TZCUP_DAY1_COVERAGE_PARTITION=tzcup_day1_bounded_coverage_20260914_03 \
TZCUP_DAY1_COVERAGE_XDG_RUNTIME=/tmp/tzcup_day1_bounded_coverage_20260914_03_xdg \
bash /root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-bounded-coverage-20260914-03/ops/run_day1_bounded_coverage_remote_dispatch.sh
'@
```

The structured receipt is
`artifacts/day1_bounded_coverage_20260914/failure_receipt_run03.json`, and the
archive SHA-256 is
`9fd4e17c313352f17bbe0d4394861df238145470a01b46c6e6ecf6b92849140e`.
