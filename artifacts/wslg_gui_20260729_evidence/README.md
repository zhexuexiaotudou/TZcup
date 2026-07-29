# WSLg GUI acceptance evidence

This compact evidence records the 2026-07-29 local-machine acceptance of the
TZcup ROS 2 Jazzy / Gazebo Harmonic visualization path.

- Host: Windows 11 build 26200
- WSL: 2.7.3.0; WSLg: 1.0.73
- Distribution: `TZcup-Ubuntu-24.04`, Ubuntu 24.04.4 LTS
- Repository baseline: `main@11ee369590f543d78eab66b7e790ba27c82cc0d5`
- ROS package set: Jazzy Desktop, `ros_gz`, Nav2, SLAM Toolbox, Fields2Cover
- Gazebo Sim: 8.11.0
- Renderer: D3D12, NVIDIA GeForce RTX 4080 Laptop GPU, OpenGL 4.6,
  hardware acceleration enabled
- Workspace tests: 449 tests, 0 errors, 0 failures, 49 skipped
- Runtime smoke check: 11/11 required topics, no missing topics, success

`smoke_check.json` is the direct output of `sanitation_smoke_check`.
`acceptance_summary.json` is the compact machine and visual acceptance summary.

The Gazebo and RViz windows were inspected live on the host. A persistent
screenshot is intentionally not claimed by this evidence packet. This result
does not represent real-vehicle, real-domain, J6, human-review, or competition
acceptance.
