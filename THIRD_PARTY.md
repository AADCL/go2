# Third-party source snapshots

The initial clean workspace was created from the source snapshots already
present in `/home/nvidia/go2_mid360_nav/src` on 2026-09-01.

| Component | Source directory used | Recorded revision/status |
|---|---|---|
| FAST-LIO | `src/third_party/FAST_LIO` | GO2 snapshot derived from `251c328d1f51a958a18cceb7055d52c46a815f44` |
| Livox ROS Driver 2 | `src/third_party/livox_ros_driver2` | `4a1def929e5b59c7a8122d19fce6efba581ce9f7` snapshot |
| Livox SDK2 | `src/third_party/Livox-SDK2` | snapshot paired with the driver |
| Unitree SDK2 | `src/third_party/Unitree_SDK2` | `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b` snapshot |

Do not create a second active copy of any of these components. Local robot
behavior belongs in the `go2_*` wrapper packages, not in third-party code.
