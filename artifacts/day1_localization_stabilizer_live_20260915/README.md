# Day-1 localization stabilizer live attempt

Two bounded harness invocations were retained. The first failed before Gazebo
or ROS startup because a host-only `/root/autodl-tmp/...` path was passed as
`OUTPUT` into the PRoot guest.

After the user explicitly authorized one corrected run, the second invocation
started the vehicle/Nav2 graph but the Gazebo launch rejected the inherited
stabilizer configuration before Gazebo startup:

```text
map_odom_stabilizer requires start_global_fusion:=true
```

The local-only vehicle launch include is being fixed to set
`map_odom_stabilizer:=false` explicitly. No third run was performed. The formal
Gazebo lock was available after both failures, and no process carrying the task
partition remained.

## Sealed files

| File | SHA-256 |
|---|---|
| `preflight-live-invocation.json` | `97FBF71D18FE99A0B7F72FF191A7EA9FD1C9BA184F418CEA7A6F14356121F89B` |
| `live_invocation_failure_receipt.json` | `2727F6F7E20B7CE1BFE4ADC1FC485B4FA1FC776003CDB505DC38C628120FEDE3` |
| `resource_release.json` | `5C104AC0A446EB93FEA4C38562C53668801F70243EED44D4FEAC06DFF22754AE` |
| `live-invocation.log` | `4411F4448B7E08E18ED6E4428713179E6F81EBE75F7A405500924D7CD3939024` |
| `live-invocation.rc` | `4355A46B19D348DC2F57C046F8EF63D4538EBB936000F3C9EE954A27460DD865` |
| `build.rc` | `9A271F2A916B0B6EE6CECB2426F0B3206EF074578BE55D9BC94F6F3FE3AB86AA` |
| `preflight-corrected-live-invocation.json` | `450F3522F8E7575161A88F80385FEC8FEF7FF48A4A99D7F731619AC183ED2179` |
| `corrected_live_failure_receipt.json` | `B2C0167711E76174A4BC40F16CB1B19E58099BAB8EB0C67CAE9248D08D53E909` |
| `live-invocation-corrected.log` | `D2EC1B6927FF2B5ACE76DD3D8EAC2FA0F0843A120277206A95189014EB45113F` |
| `live-invocation-corrected.rc` | `4355A46B19D348DC2F57C046F8EF63D4538EBB936000F3C9EE954A27460DD865` |
| `corrected-run-01-remote/launch.log` | `DA6EBA3449DDBC3EF576BFAA7CE0F1E3ABFB1DD82111A9853B43116C787E7C07` |
| `corrected-run-01-remote/navigation.log` | `FDBE5CDA8B7A1E6A8332A575218818D47CBA8A372BF135BF718B00FF7DFF6784` |
| `corrected-run-01-remote/amcl.params.yaml` | `32CAAF2CC9679DA400F8B699F1CFBD65937B23479E5E9919F69E5015CF6B7CF4` |
| `corrected-run-01-remote/resource_release.json` | `EA853C0ED9CE425FCAE5919F64AF247E48EA4A11E306CD50FD117D50780988CE` |

## Boundary

These are precondition failure receipts, not localization results. They prove
both bounded invocations ended before Gazebo and that resources were released;
they do not provide a live RMSE, P95, maximum error, TF authority receipt,
MCAP, driver rc, focus scorer, or `LIVE_CANDIDATE_PASS`.
