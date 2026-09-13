# Day-1 localization 50 mm diagnostic

**Date:** 2026-09-14

**Candidate:** `f1f2282`

**Status:** `NOT_MEASURED`

**Decision:** stopped before dynamic data; the one bounded dynamic attempt was
not converted into a second run.

## Preflight

The designated worktree was clean on `codex/day1-localization-run`. The formal
Gazebo lock was available, no Gazebo or ROS runtime was running, and the WSL
runtime had 12 GiB RAM, 229 GB free on the work volume, and working D3D12/NVIDIA
rendering.

The focused localization tests passed (`44 passed`). `scripts/ci_fast.py`
stopped on the pre-existing README-length hygiene check before running its
tests. Bash syntax and `git diff --check` passed.

An isolated overlay was built from the committed worktree for
`sanitation_localization_acceptance`, `sanitation_formal_campus_integration`,
`sanitation_localization`, `sanitation_vehicle_description`,
`sanitation_gazebo_control`, and the required `sanitation_tasks` dependency.

## Fixture and ownership binding

The public base episode matched prior artifact hashes:

* manifest: `a97be33d0ceaf558a6fdf12dc914e7819567e9eaf4af33f473835c1731cfd4dc`
* world: `f7460fdcc7ad221e613fdda1d1731bedb23c872094f03c6e2664934d18a54471`

The dirt patch and pedestrian were reconstructed with the prior fixture
semantics. The reconstructed manifest hash is
`5145327d76f14ce6590e7c3005181559b93a6a9d6e47a2a5745132b315e28050`;
it does not match the old run's manifest hash
`6df919e8835807ca6a7c4e335fedc66f6e66503887d66b85622ccf6e8ac3fc58`.
The start poses and diagnostic patch semantics match, but no exact-byte
reproduction is claimed.

The saved map and masks were copied byte-for-byte from the prior
`localization-50mm-20260913-01` evidence. The occupancy map and YAML hashes are:

* `occupancy.pgm`: `25b51cdf3f6b52a47db91ffa9e397c09e219e73684db9ddafe6023a5ef6997ca`
* `occupancy.yaml`: `02cecd0afc85cfd385d4a92a580e40bfc08b64115bce83e2020cd986fcffce17`

## Attempt

The existing committed runner was invoked once with
`PROBE_AMCL_TF_BROADCAST=false`, `PROBE_FOCUS_VALIDATION=1`, LOCALHOST
discovery, a fresh output directory, and the preserved map:

```powershell
wsl.exe -d TZcup-Ubuntu-24.04 -- env `
  SOURCE=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-run `
  RUNTIME=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-integrated-functional-acceptance/.work/final_frozen_runtime_r40_timeout_boundary_20260831_231231 `
  COMPETITION_RUNTIME_OVERLAY=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-run/.work/day1-localization-run/overlay `
  PYTHONPATH=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-run/.work/day1-localization-run/deps `
  OUTPUT=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-run/.work/day1-localization-run/evidence/run-01 `
  EPISODE=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-run/.work/day1-localization-run/episode `
  MAP_SOURCE=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-run/.work/day1-localization-run/map-source `
  DRIVER=/mnt/f/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-run/scripts/competition_localization_route.py `
  CANDIDATE_REVISION=f1f228232aafd7ec2187e1ef4914f91f8fbe38d7 `
  PROBE_FOCUS_VALIDATION=1 PROBE_AMCL_TF_BROADCAST=false `
  PROBE_DOMAIN=98 PROBE_PARTITION=tzcup_localization_day1_20260914_01 `
  PROBE_CAMERAS=false PROBE_PEDESTRIANS=false `
  PROBE_SECONDS=700 PROBE_PREPARE_SECONDS=0 PROBE_GOAL_X=6 `
  PROBE_ESTOP_DISTANCE=0 `
  bash /mnt/f/Project/TZcup/.workspace/worktrees/TZcup-day1-localization-run/scripts/run_competition_motion_cleaning_probe.sh
```

## Blocker

The campus launch stopped before creating Gazebo:

```text
[ERROR] [launch]: Caught exception in launch (see debug for traceback):
local_costmap inflation_radius must be strictly greater than the enabled-footprint
inset radius plus padding (0.55)
```

No `/clock`, MCAP, TF registry, route, or localization scorer dataset was
created. Sample counts are zero and no RMSE, P95, maximum error, displacement,
pair coverage, or 50 mm result exists.

The current attempt is therefore `NOT_MEASURED`, not PASS or FAIL on the 50 mm
criterion. Ground truth was not used for control. The attempt was interrupted
after the pre-Gazebo failure so it would not consume the full bounded window.

The formal Gazebo lock was released and a final process census found no Gazebo
or ROS runtime: `SIM_RESOURCE_RELEASED=true`.

## Retained evidence

Raw partial evidence is retained at:

`.work/day1-localization-run/evidence/run-01`

It contains the generated `nav2.yaml`, AMCL parameter snapshot, empty local EKF
parameter snapshot, launch/navigation logs, dirt bridge log, and copied map
masks. The compact receipt with hashes is in
`artifacts/day1_localization_20260914/failure_receipt.json`. No MCAP or TF
authority report exists to preserve.

No second dynamic run was attempted. A future run requires a separately
reviewed fix for the pre-dynamic Nav2 costmap/footprint contract failure.
