# Journey 6 loopback HIL report

The split-HIL topic/QoS contract, PC safety gateway, command sequence/time
checks, health timeout, E-stop, physical envelope, network-loss latch, operator
resume, PC-node blacklist, Docker isolation, and network-fault plan are
implemented. Pure-Python tests pass and the ROS package builds in a clean ROS 2
workspace.

The proprietary OE image and real J6 algorithm runtime were unavailable, so
the 30-minute closed loop, dynamic-obstacle, insertion/removal, re-observation,
spot/post-clean, process-crash, delay/loss, disconnect/reconnect, and stale
replay matrix was not run.

```text
duration_s=not_run
GT_control_violation=not_run
PC_duplicate_algorithm_nodes=not_run
J6_command_authority=contract_only
command_timeout_safe_stop=unit_test_only
network_loss_safe_stop=unit_test_only
J6_LOOPBACK_HIL_READY=false
```
