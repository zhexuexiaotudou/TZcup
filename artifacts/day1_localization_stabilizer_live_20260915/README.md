# Day-1 localization stabilizer live attempt

The single permitted harness invocation failed before Gazebo or ROS startup.
The runner passed a host-only `/root/autodl-tmp/...` path as `OUTPUT` into the
PRoot guest, so `mkdir` could not create the run directory from the guest.

No retry was performed. The formal Gazebo lock was available after the failure,
and no process carrying the task partition remained.

## Sealed files

| File | SHA-256 |
|---|---|
| `preflight-live-invocation.json` | `97FBF71D18FE99A0B7F72FF191A7EA9FD1C9BA184F418CEA7A6F14356121F89B` |
| `live_invocation_failure_receipt.json` | `2727F6F7E20B7CE1BFE4ADC1FC485B4FA1FC776003CDB505DC38C628120FEDE3` |
| `resource_release.json` | `5C104AC0A446EB93FEA4C38562C53668801F70243EED44D4FEAC06DFF22754AE` |
| `live-invocation.log` | `4411F4448B7E08E18ED6E4428713179E6F81EBE75F7A405500924D7CD3939024` |
| `live-invocation.rc` | `4355A46B19D348DC2F57C046F8EF63D4538EBB936000F3C9EE954A27460DD865` |
| `build.rc` | `9A271F2A916B0B6EE6CECB2426F0B3206EF074578BE55D9BC94F6F3FE3AB86AA` |

## Boundary

This is a precondition failure receipt, not a localization result. It proves
that the sole bounded live invocation ended before Gazebo and that resources
were released; it does not provide a live RMSE, P95, maximum error, TF authority
receipt, MCAP, or `LIVE_CANDIDATE_PASS`.
