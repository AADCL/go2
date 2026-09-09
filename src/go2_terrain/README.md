# GO2 terrain integration

This package separates terrain handling into two independent paths:

- Offline/global terrain products are loaded by the global costmap.
- Live local navigation uses only ground-relative obstacle filtering. It does
  not add local slope, step, or roughness costs.

## Offline export and global cost

`export_terrain.launch` consumes `public_map.pcd` and
`traversed_path_map.pcd` plus a complete legacy `map.yaml`/PGM generated in a
hidden staging directory. Its exact geometry is retained. The production
profile refuses export without this occupancy baseline. The exporter estimates
base-to-floor height from the path start and fails unless the result is within
0.20-0.55 m.

When an input map is supplied, its PGM is retained as the occupancy baseline
inside the same staged transaction. Verified continuous ground may reclassify
legacy false obstacles caused by its absolute-Z projection on a slope.
Trajectory-only evidence may fill unknown cells but never clears a baseline
obstacle. Rejected terrain remains unchanged, and measured terrain obstacles
are applied last, so walls and steps stay occupied. Every recorded trajectory
point contributes direct free-space evidence because the robot physically
occupied that position; segments are connected only when their spacing is at
most 0.50 m. This evidence can fill unknown cells but cannot clear an existing
baseline or measured terrain obstacle. A lost
ground trace can reacquire only from a PMF-admissible seed inside a 0.65 m
absolute band around the initial floor. This supports accumulated height on a
long ramp while excluding the observed ceiling band. Ground trace validity
continues to control elevation and slope reconstruction, independently of the
gap-bounded driven free corridor.

Before commit, the exporter requires at least 80 percent trajectory-ground
trace coverage, reconstruction of 30 percent of PMF-admissible observed
ground cells, and 60 percent of reconstructed ground in one eight-connected
quality component. Primary ground propagation remains four-connected for step
safety, and the existing height-bounded hole fill is unchanged; only the
non-mutating completeness measurement accepts diagonal lidar sampling
adjacency. The denominator deliberately excludes unobserved bounding-box
area, so a valid narrow corridor is not rejected. These checks are computed
from terrain itself rather than the preserved occupancy baseline, so a
complete PGM cannot hide a sparse or fragmented terrain result.
Ground must additionally cover 10 percent of legacy free cells, at least 95
percent of the complete driven corridor must be known in the merged map, and
every known legacy cell must remain known.

The exporter writes a PGM/YAML occupancy map plus:

```text
terrain_2p5d.yaml
terrain_elevation.f32     terrain_slope.f32
terrain_roughness.f32     terrain_step.f32
terrain_cost.u8           terrain_confidence.u8
terrain_checksums.sha256
terrain_ground.pcd        terrain_obstacles.pcd
terrain_preview.ppm
```

The four `.f32` files are row-major little-endian floats with NaN for unknown
cells. Cost uses ROS values 0-254 with 255 unknown; confidence uses 0-100 with
255 unknown. Every array shares the PGM resolution, dimensions, lower-left
origin, and row-major map coordinates. All files are generated in a staging
directory, read back and checksummed before per-file atomic replacement;
`terrain_2p5d.yaml` is installed last as the commit marker. Source PCDs are
never modified.

`go2_terrain::Go2TerrainLayer` raises costs only in cells already known free
by the static layer. It refuses rolling costmaps and frames other than `map`,
so it cannot accidentally be loaded into the local costmap. Approved slope
costs are zero through 8 degrees, 15-80 from 8-30 degrees, and lethal above 30
degrees only for an eight-connected group of at least four cells. Costs are
expanded laterally by 0.20 m.

For compatibility with the already validated occupancy exporter, obstacle
inflation retains its legacy four-neighbour, rounded-cell behavior. At the
0.05 m map resolution the configured 0.03 m value therefore expands one
cardinal grid cell (an effective 0.05 m). Trajectory anchoring is separate: its
0.25 m seed area uses the true metric distance between the path sample and
each grid-cell center, so diagonal cells outside the circular radius cannot
seed a disconnected surface.

Manual export command:

```bash
roslaunch go2_terrain export_terrain.launch map_dir:=/absolute/map/directory
rosrun go2_terrain validate_terrain_map.py --map-dir /absolute/map/directory
```

## Runtime data flow

```text
/cloud_registered_base (base_link, exact scan time)
  -> go2_terrain_cloud_adapter
/cloud_registered_terrain (terrain_sensor, gravity aligned)
  -> go2_navigation_patchworkpp
/terrain/patchwork_ground + /terrain/patchwork_nonground
  -> go2_terrain_guard
/terrain/obstacle_points + /terrain/clearing_points
  -> move_base/local_costmap/obstacle_layer
```

`terrain_sensor` is dynamically published under `odom`. Its origin is the
calibrated `lidar_link` origin and its orientation preserves robot yaw while
removing roll and pitch. All transforms use the cloud timestamp; there is no
latest-TF fallback.

The Patchwork wrapper and all external cloud publishers/subscribers use queue
depth one. The wrapper replaces upstream `demo`, which uses queue depth 100,
latched publishers, and per-frame logging. High/ceiling returns above 1.50 m
relative to nearby ground are excluded from both marking and clearing.
Nonground returns are marked only when a ground cell exists within 0.45 m.
A return without nearby ground is always unknown, so a Patchwork miss on a
ramp cannot turn that ramp into a hard local obstacle.

Patchwork ground is also treated as a candidate rather than trusted clearing
evidence. The guard selects the low support band in each cell, anchors near
the nominal floor below the MID360, and grows with four-connected cells only
across a spatially connected surface no steeper than 35 degrees. Four-way
growth prevents low steps from bypassing the grade limit through a diagonal.
A ceiling or other disconnected
horizontal surface mistakenly labelled ground is sent to the unknown/debug
class and cannot generate a clearing ray. Rejected candidates from either
Patchwork stream are returned to normal obstacle-height classification unless
they pass the strict continuous-steep-surface test below. This preserves
marking for boxes, table surfaces, legs, walls, and disconnected platform tops
when valid nearby ground exists.

Patchwork non-floor returns are checked once more for a sustained ground-like
profile.
A segment is downgraded to unknown only when it starts at connected ground,
continues monotonically in one of eight metric directions for at least three
cells, stays between 8 and 70 degrees, has consistent first differences, at
least two raw points per cell, and passes a point-level Huber fit of the full
two-dimensional plane `z=ax+by+c`. Only the robust low-support band from each
cell enters this fit, so a same-column ceiling or other high return cannot
invalidate an otherwise continuous ramp; all high returns still pass through
the final unknown/obstacle classifier. The fitted grade must remain within the
same angle range, RMSE must not exceed 0.025 m, and no point residual may
exceed 0.05 m. A thin wall, a flat-topped box, or
a regular stair profile therefore remains marking evidence. A long planar
object leaning at a ground-like angle is geometrically indistinguishable from
a real slope in one LiDAR frame; accepting that ambiguity is an inherent risk
of the selected design in which the local costmap has no slope layer.

Health is published on `/terrain/healthy` and `/terrain/status`. Diagnostics
include `ground_ratio`, `output_rate_hz`, point counts, processing latency, and
high-return counts. Health additionally requires at least 0.60 square metres
of one connected ground component, 0.18 square metres of support within 1.0 m
of the robot, and at least four of eight near-field sectors with two cells
each. A Huber-robust plane `z=ax+by+c` is fitted to the selected near-field
ground component within 1.50 m of the MID360. Diagnostics report height `-c`,
slope, weighted RMSE, sample count, and fit status. At least 12 samples with
non-degenerate planar spread are required, RMSE may not exceed 0.04 m, and the
measured height must remain within 0.43-0.59 m. The local-radius limit prevents
distant terrain or a slope transition from moving the height at the robot.
Sparse, fragmented, remote-only, one-sided, crouched, or abnormally elevated
support therefore cannot arm navigation. Health also remains false until at
least three valid frame-rate
samples have been collected and the EWMA rate is at least 8 Hz. Startup and
low-rate grace periods affect diagnostic severity only; they never bypass the
control gate.

## Dependency

The vendored GPL package owns both the algorithm and its GO2 wrapper:

```text
Repository: https://github.com/url-kaist/patchwork-plusplus-ros
Commit:     f8c070bf2774b2f3ef622644a511bdfe3f2f27bb
Path:       src/third_party/patchworkpp
License:    GPL-3.0
```

Its upstream CMake did not export `include`; this workspace adds
`catkin_package(INCLUDE_DIRS include)` and builds the bounded queue-one wrapper
inside that same GPL package. No Patchwork algorithm source is changed. The
BSD `go2_terrain` binaries communicate with it only through ROS messages and
do not include or link the GPL template implementation. See
`src/third_party/patchworkpp/LOCAL_CHANGES.md`.

## Modes

`go2_bringup/launch/navigation.launch` accepts `terrain_enabled` and
`terrain_metadata`:

- `terrain_enabled:=false` (default): preserves the old local costmap input and
  does not start this runtime path. This is compatible with existing maps.
- `terrain_enabled:=true`: starts this runtime path, requires terrain health,
  and does not start the old `/cloud_registered_costmap` branch. The top-level
  launcher must validate and pass `terrain_metadata` before selecting it.

The robot 1 control interface remains `eth0`. The nominal measured sensor
height is 0.51 m; startup fails outside 0.43-0.59 m. Re-measure it whenever the
standing height or mechanical mounting changes.
