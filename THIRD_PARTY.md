# Third-party source snapshots

The initial clean workspace was created from the source snapshots already
present in `/home/nvidia/go2_mid360_nav/src` on 2026-09-01.

| Component | Source directory used | Recorded revision/status |
|---|---|---|
| FAST-LIO | `src/third_party/FAST_LIO` | GO2 snapshot derived from `251c328d1f51a958a18cceb7055d52c46a815f44` |
| Livox ROS Driver 2 | `src/third_party/livox_ros_driver2` | `4a1def929e5b59c7a8122d19fce6efba581ce9f7` snapshot |
| Livox SDK2 | `src/third_party/Livox-SDK2` | snapshot paired with the driver |
| Unitree SDK2 | `src/third_party/Unitree_SDK2` | `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b` snapshot |
| Patchwork++ ROS1 | `src/third_party/patchworkpp` | `f8c070bf2774b2f3ef622644a511bdfe3f2f27bb`, GPL-3.0 |

The dynamic Bayesian mapping, offline terrain reconstruction, and global
terrain-layer architecture use AADCL/ugv WheelTech V5.1 at commit
`4a240019f3776e82f0de1b55029339d056e03bcc` as a behavioral and architectural
reference. The GO2 implementations in `go2_mapping` and `go2_terrain` were
written for this workspace; no WheelTech source file, chassis integration,
TF tree, recovery behavior, or bringup package is copied into this tree.
Those GO2 packages carry their own package-level BSD-3-Clause license notice.

Patchwork++ remains an independent ROS package under its upstream GPL-3.0
license. Robot-specific frame conversion, queueing, health checks, and local
costmap routing live in `go2_terrain`; the pinned third-party algorithm is not
used for FAST-LIO/NDT map input.

Do not create a second active copy of any of these components. Local robot
behavior belongs in the `go2_*` wrapper packages, not in third-party code.
