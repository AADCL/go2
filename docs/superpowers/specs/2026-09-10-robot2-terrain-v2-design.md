# Robot 2 Terrain-Aware Navigation V2 Design

## Goal

Upgrade the second GO2 (`unitree@192.168.50.111`) from the
`unitree-orin-nano` baseline at `e934e95` to the terrain-aware behavior in
`v2.0.1` (`7d01bb1`) while preserving the existing mapping, localization,
navigation, command-line, and Unitree SDK2 behavior.

## Fixed baselines

- Algorithm source: Git tag `v2.0.1`, commit `7d01bb1`.
- Existing Robot 2 workspace: `/home/unitree/go2_nav_ws`.
- Robot 2 management address: `192.168.50.111` on `eth1`.
- Livox host address: `192.168.1.50` on `eth0`.
- Livox Mid-360 address: `192.168.1.168`.
- GO2 DDS interface: `go2dds` with host address `192.168.123.18`.
- Build target: Ubuntu 20.04, ROS Noetic, AArch64 Orin Nano, approximately
  7.2 GiB RAM.

## Required behavior

### Dynamic-obstacle-aware mapping

Use the V2 coarse/fine voxel evidence model, Bayesian hit/miss updates,
timestamped 6-DoF ray origins, bounded cloud/odometry queues, and persistent
map storage. Transient objects must remain candidates until they satisfy the
configured observation count, duration, and hit-ratio gates. Repeated free-ray
evidence must clear stale candidates and map voxels according to the V2
thresholds.

### Terrain map export and ceiling rejection

Use the V2 `go2_terrain` exporter and vendored Patchwork++ integration. Build
the 2.5D elevation, slope, roughness, step, cost, and confidence products from
the saved 3D map and traversed path. Ground reconstruction must use the low
support band and connected-surface constraints so ceiling and suspended
horizontal returns cannot seed or clear traversable ground.

The existing PGM/YAML occupancy map remains the baseline. Verified terrain may
fill unknown driven space and correct slope-induced false obstacles only under
the V2 quality gates. Source PCD files are never modified.

### Global slope cost

Load `Go2TerrainLayer` only in the non-rolling global costmap when a validated
`terrain_2p5d.yaml` is present. Preserve V2 slope costs: zero through 8 degrees,
soft cost from 8 to 30 degrees, and lethal cost above 30 degrees only for a
large-enough connected cluster. The standard `GlobalPlanner` and its Dijkstra
configuration remain unchanged.

### No local slope cost

Do not load `Go2TerrainLayer` into the local costmap. In terrain mode, use the
gravity-aligned cloud, Patchwork++ candidates, and `terrain_guard` only to
produce ground-relative marking and clearing point clouds. TEB remains the
local planner and receives no explicit slope, roughness, or step cost layer.

## Compatibility constraints

- Preserve `run_go2 mapping`, `save-map`, `export-map`, `navigation`,
  `status`, `enable`, and `disable` entry points.
- Preserve FAST-LIO, NDT-OMP, `move_base`, GlobalPlanner, TEB, velocity shaping,
  navigation supervision, and Unitree SDK2 SportClient behavior except where
  V2.0.1 already changes safety integration for terrain health.
- Preserve `max_vel_y: 0.0`; the robot remains non-holonomic at the navigation
  interface.
- Preserve Robot 2 maps. Existing maps without terrain metadata must continue
  to use legacy navigation mode.
- Preserve the Orin Nano Unitree SDK2/CycloneDDS library handling from the
  `unitree-orin-nano` branch.
- Preserve the Robot 2 network profile and Livox address; do not copy Robot 1
  values such as `/home/nvidia`, `eth0` for DDS, or `192.168.1.191`.
- Do not enable real chassis motion during automated deployment validation.

## Source integration approach

Start from `v2.0.1`, not from the old Nano tree, because V2 introduces a new
terrain package, Patchwork++, mapping storage interfaces, tests, launch wiring,
and command validation across many files. Reapply only the proven Robot 2
hardware adaptations:

1. `/home/unitree/go2_nav_ws` documentation and operational paths.
2. Livox device `192.168.1.168` with host `192.168.1.50` on `eth0`.
3. Unitree DDS on `go2dds`, host `192.168.123.18`.
4. Orin Nano AArch64 SDK/DDS linkage and runtime SONAME compatibility.

No terrain algorithm is forked or rewritten for Robot 2.

## Deployment and rollback

The current `/home/unitree/go2_nav_ws` is a copied workspace without Git
metadata and contains generated files and legacy top-level packages. It must
not be updated in place.

1. Create a clean staging checkout from the approved integration commit.
2. Copy Robot 2 maps into staging without deleting or rewriting the originals.
3. Build and test staging with one compile job.
4. Rename the current workspace to a timestamped backup only after staging
   passes offline checks.
5. Move staging to `/home/unitree/go2_nav_ws` and recreate the user-level
   `run_go2` symlink.
6. Keep the backup until Robot 2 completes supervised mapping and navigation
   acceptance. Rollback is a directory-name swap followed by rebuilding the
   `run_go2` symlink.

## Validation

### Before deployment

- Confirm no ROS mapping, navigation, or SDK bridge process is active.
- Record network interfaces, routes, radar reachability, disk space, maps, and
  checksums of the active workspace.
- Verify the integration diff contains the V2.0.1 source plus only the listed
  Robot 2 hardware adaptations.

### Offline build and automated checks

- Build with `catkin_make -DROS_EDITION=ROS1 -j1`.
- Run all `go2_mapping` and `go2_terrain` C++ tests.
- Run `go2_terrain` Python validation tests.
- Run workspace package and launch validation.
- Verify `run_go2` command parsing for legacy and terrain modes.

### Non-motion runtime checks

- Confirm Livox `192.168.1.168` is reachable from `eth0`.
- Start mapping without commanding the chassis and verify Livox, IMU,
  FAST-LIO odometry, registered cloud, mapping status, and trajectory topics.
- Stop mapping cleanly and confirm no SDK motion bridge was enabled.
- Validate an existing map remains usable in legacy navigation mode.
- Terrain mode is accepted only for a map with complete metadata and passing
  checksums; an incomplete terrain map must fail closed.

## Acceptance criteria

- Robot 2 builds and passes the V2.0.1 mapping and terrain test suites on its
  Orin Nano.
- All Robot 2 IP addresses and interfaces remain unchanged.
- Existing maps remain present and launch through legacy mode.
- New mapping output contains the persistent PCD and traversed-path data needed
  for terrain export.
- Terrain export rejects incomplete or ceiling-contaminated ground products
  through the V2 quality gates.
- Terrain navigation loads slope cost globally and never loads slope cost into
  the rolling local costmap.
- Real SDK control remains disabled until a person performs the supervised
  field acceptance test.

