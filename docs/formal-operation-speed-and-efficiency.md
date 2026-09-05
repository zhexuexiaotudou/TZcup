# Formal operation speed and competition-efficiency boundary

The formal A300 is a four-wheel skid-steer vehicle.  Its canonical geometry and
declared effective dry-cleaning width are defined by
[`formal_motion_cleaning_profile.yaml`](../config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml),
not by legacy small-demo coverage settings.

The source contract in
[`formal_operation_speed_profiles.yaml`](../config/high_fidelity_vehicle/formal_operation_speed_profiles.yaml)
separates three roles:

- `mapping_safe` retains the current 0.45 m/s ceiling for first-map, localization
  and safety-constrained transit. Mapping mode requires this profile; it is not
  eligible to claim competition efficiency.
- `dry_cleaning_competition_candidate` records a 1.0 m/s dry-cleaning candidate.
  At 1.32 m declared width and 75% route efficiency its design calculation is
  3564 m2/h, only 64 m2/h above the 3500 m2/h requirement.  The exact calculated
  speed floor is 0.9820426487093153 m/s; 0.982323 m/s is retained as a conservative
  decimal candidate floor. The saved-map dry-cleaning lifecycle now selects this
  profile explicitly and materializes only Nav2 `CleanPath` plus the velocity
  smoother request at 1.0 m/s. The final `whole_vehicle_safety_manager` still
  enforces the unrequalified 0.45 m/s envelope, so this plumbing does not yet
  authorize or demonstrate actual 1.0 m/s vehicle motion. Transit remains
  governed by its existing controller settings, and collision-monitor/speed-filter
  safety constraints remain active. Neither number is acceptance evidence.
- `wet_puddle_recovery` retains the existing hydraulic limits of 0.115899 m/s
  at 0.002 m depth and 0.023180 m/s at 0.010 m depth.  Wet operation does not
  inherit the dry-efficiency claim.

The formal competition claim can be enabled only after one source-bound measured
coverage gate recomputes area per hour from the same frozen session, A300
runtime, effective-width evidence and measured cleaning duration.  The measured
duration must include cleaning motion, turns, obstacle avoidance and repair
work according to the published task timing rule; it may not be replaced by a
static width-times-speed calculation.

Validation order is therefore: retain mapping-safe operation, validate A300
mobility and skid-steer tracking at the candidate speed, validate physical
ground-dirt cleaning, validate dynamic-obstacle behavior, then run a
source-bound end-to-end coverage measurement.  Only then may the dry profile be
considered for formal runtime and competition efficiency.

The current final runner selects `dry_cleaning_competition_candidate` upstream,
but the final safety envelope does not yet authorize that speed. It remains
`NOT_READY_FOR_COMPETITION_EFFICIENCY` until 1.0 m/s is requalified through the
source-bound mobility, interlock, dynamic-obstacle and cleaning gates and the
measured efficiency gate passes. The
evidence must calculate performance from the source-bound effective-coverage
union and the actual total task duration, including turns, obstacle avoidance
and repair work. The speed contract is a fail-closed audit, not a static
width-times-speed acceptance calculation. See also the
[high-fidelity competition vehicle plan](high-fidelity-competition-vehicle-plan.md).

## Isolated 1.0 m/s requalification lane

`config/high_fidelity_vehicle/formal_dry_speed_requalification.yaml` defines an
evidence-only lane. It is deliberately unavailable unless the operator sets
`FORMAL_DRY_SPEED_REQUALIFICATION=1` and invokes
`scripts/run_formal_dry_speed_requalification.sh` in a fresh artifact root.
The wrapper creates one run-scoped explicit opt-in marker; the four reused
runners reject every non-default cap without that marker. Before it
aggregates, it rebuilds the runtime binding, so a session no longer `RUNNING`
or a changed frozen closure blocks the report.
The wrapper runs one Gazebo process at a time and reuses the source/session/
runtime-bound mobility, whole-vehicle interlock, dynamic-obstacle and
ground-dirt runners. It admits a pass only when all four reports bind to one
acceptance session and prove commanded 1.0 m/s motion, final physical stop,
E-stop/fault command suppression, collision-monitor intervention with zero
collision, and physical brush/ground-dirt contact cleaning.

The launch-only 1.0 m/s value is never written to
`operational_envelopes.yaml`; its product default remains 0.45 m/s. A passing
requalification report sets only `dry_speed_safety_requalified=true` and keeps
`competition_efficiency_eligible=false`. Its machine-checkable
`requalification_evidence_marker` is evidence-only: a future PR that attempts to
raise the product cap must fail closed unless it explicitly consumes this
source-bound report and then passes the separate measured coverage gate. This
lane neither changes the final formal acceptance scheduler nor authorizes
hardware operation.
