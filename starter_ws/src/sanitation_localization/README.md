# Formal localization fusion

The product localization architecture has one owner per transform:

- `local_ekf` fuses wheel odometry and IMU and owns `odom -> base_link`;
- AMCL consumes the saved lidar map and publishes `/amcl_pose` with
  `tf_broadcast=false`;
- `navsat_transform` converts GNSS fixes into metric `/odometry/gps`;
- `global_ekf` fuses local velocity, AMCL x/y/yaw and GNSS x/y and owns
  `map -> odom` during saved-map cleaning.

During first-task SLAM, global fusion is disabled because `slam_toolbox` owns
`map -> odom`; the local wheel/IMU EKF remains available. This prevents the
SLAM, AMCL and GNSS filters from competing for the same TF edge.

The package is not considered integrated until the formal campus launch stops
republishing raw wheel odometry on `/odom`, disables AMCL TF output, starts the
correct fusion mode, and passes a live no-duplicate-writer runtime gate.

## Runtime acceptance tools

The collector observes only product localization topics, ROS graph endpoint
ownership and `/tf` publisher GIDs. It never subscribes to Gazebo world state,
model state, reference pose or ground truth:

```bash
ros2 run sanitation_localization_acceptance \
  formal_localization_runtime_collector \
  --mode mapping --duration-seconds 20 \
  --output /tmp/formal_localization_mapping_runtime.json

ros2 run sanitation_localization validate_formal_localization_runtime \
  --input /tmp/formal_localization_mapping_runtime.json \
  --output /tmp/formal_localization_mapping_acceptance.json
```

Use `--mode cleaning` for saved-map cleaning. Validation is fail-closed and
requires all of the following from the same observation window:

- `/odom` has exactly one active or observed owner, `local_ekf`;
- wheel odometry and IMU are active and `local_ekf` subscribes to both;
- mapping has `slam_toolbox` as the only observed `map -> odom` authority and
  no `global_ekf` node;
- cleaning has `global_ekf` as the only observed `map -> odom` authority and no
  `slam_toolbox` node;
- cleaning additionally has live AMCL, GNSS, converted GPS odometry and fused
  odometry, with the expected subscriptions and publisher ownership.

The collector is implemented with `rclcpp::MessageInfo` because Jazzy `rclpy`
does not expose the publisher GID in its message metadata on the supported RMW.
It records endpoint GIDs as well as node names, so two nodes sharing `/tf`
cannot hide a duplicate `map -> odom` writer. A missing GID-to-node
mapping, insufficient messages, forbidden truth subscription or silent second
`/odom` publisher blocks acceptance rather than being treated as inconclusive.
