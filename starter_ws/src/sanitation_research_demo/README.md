# sanitation_research_demo

This package is a deliberately non-authoritative integration harness for work
that can proceed before the measured vehicle and arm URDF arrive. It generates
one campus episode, exposes evaluator truth only to the environment, runs the
belief-only sensing-greedy trajectory policy, and requires the mock
manipulation adapter to verify placement in the rear bin before a cube clears.

The report always states that no measured URDF, real robot, RDK S100 runtime, or
Journey 6 evidence was used. The circular dirt rasterization and synthetic
perceived cube geometry are research approximations, not product evidence.

```bash
ros2 run sanitation_research_demo urdf_independent_research_demo \
  --config /path/to/default_scenario.yaml --profile research \
  --split train --map-index 0 --mission-index 0 \
  --output /tmp/urdf-independent-demo
```
