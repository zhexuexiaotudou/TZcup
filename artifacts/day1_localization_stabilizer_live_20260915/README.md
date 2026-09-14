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

The local-only vehicle launch include was fixed to set
`map_odom_stabilizer:=false` explicitly. Under a separate explicit user
authorization, the fix was applied at the remote source, only
`sanitation_vehicle_description` was rebuilt, and offline launch signature
passed. The final `run-02` then started Gazebo but failed before a live pass:

- the driver sent its first goal before the Nav2 lifecycle was active, so
  `bt_navigator` rejected it;
- route `passed=false`, `nav_results=[]`;
- the stabilizer stayed `WAITING` with zero raw, accepted, rejected, and
  published updates;
- the focus scorer failed because the selected Python environment had no
  `mcap` module, so no live RMSE/P95/max were computed;
- `driver.rc` and `live_candidate_receipt.json` were not written;
- cleanup released Gazebo/ROS and the formal lock.

The route is stopped. No further live retry was performed.

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
| `preflight-final-live-invocation.json` | `B2B6390DA9217ECD230D1BEC495A57AF6204C9AC18794333051A2A65C95F383D` |
| `final_live_failure_receipt.json` | `15DB279B7305EB90A2AF2C814A72751BA52BAE1E975643B91F558D295BE29ED3` |
| `live-invocation-final.log` | `E3B0C44298FC1C149AFBF4C8996FB92427AE41E4649B934CA495991B7852B855` |
| `live-invocation-final.rc` | `53C234E5E8472B6AC51C1AE1CAB3FE06FAD053BEB8EBFD8977B010655BFDD3C3` |
| `final-run-02-remote/effective_parameters.json` | `EE9845B56F7392AE4A52A30C2545B4E802CF83B42207FAAA89209A689FC9896D` |
| `final-run-02-remote/tf_authority.json` | `44CF83AD44E7B6BA85EA3DFF97B45DD88037053D7CD53518ADF3C516BB5380AD` |
| `final-run-02-remote/map_odom_stabilizer.status.yaml` | `9DC0FE6B64E339AF6F5061F0BCE315FFAFCDAC170B209741CD51D8140F0F83F4` |
| `final-run-02-remote/route.json` | `87125271D7F32BBA0E24B533B0E3640BC6BFAB3AD509A6D82C92438E2C32B5B9` |
| `final-run-02-remote/driver.log` | `B761FCA672551A0EF0179E2B4361F9522CCD94D7B60BF60B13A7FDDC6DEB54D4` |
| `final-run-02-remote/localization_focus.json` | `DD4F455CEF90F23EE8515FB8DB100C4C0EA51658A7112C42C2CFDFDB07E97566` |
| `final-run-02-remote/bag_info.txt` | `AD3B82ADB0074A9CFB90111E77DA6057F79413F73ACE095F6BAE6AC3039470F6` |
| remote `run-02/bag/bag_0.mcap` | `2C067BB15FB4CCF0DD2BC9345ABD55D6017AD1A8881CE5B1234692352E64FC23` |

## Boundary

These are precondition failure receipts, not localization results. They prove
both bounded invocations ended before Gazebo and that resources were released;
they do not provide a live RMSE, P95, maximum error, TF authority receipt,
MCAP, driver rc, focus scorer, or `LIVE_CANDIDATE_PASS`.
