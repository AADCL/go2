# Robot 1 terrain export failure fix

Date: 2026-09-09 (Asia/Shanghai)

Target:

- Host: `192.168.50.110`
- Workspace: `/home/nvidia/go2_nav_ws`
- Primary failed map: `lab_202609091450`

## Failure

The stable occupancy baseline exported correctly, but the terrain exporter
failed with:

```text
Robust ground reconstruction produced too few cells
```

The source map was valid and unchanged:

- `public_map.pcd`: 107593 points
- `traversed_path_map.pcd`: 233 points
- both files matched `mapping_snapshot.sha256`
- base-to-floor estimate: 0.362 m
- trajectory trace: 216/233 (92.7%)

Added diagnostics isolated the failure:

```text
candidates=27123
pmf_cells=1823
admissible_candidates=1821
trajectory_seeds=0
accepted=0
```

PMF found plausible floor elsewhere in the indoor cloud, but none of those
cells overlapped the 0.25 m trajectory corridor within the 0.04 m height
tolerance. The old implementation therefore discarded every otherwise valid
trajectory-ground seed.

After fixing seed creation, the old quality gate exposed a second issue. It
treated raw PMF overlap as ground truth and required all fitted ground pixels
to form one dominant image component. On the real multi-room map this produced
an invalid 1.5% reconstruction score and a 35.1% component score despite 3492
successfully fitted, trajectory-anchored ground cells.

## Changes

Changed files:

- `src/go2_terrain/include/go2_terrain/terrain_algorithms.hpp`
- `src/go2_terrain/src/terrain_algorithms.cpp`
- `src/go2_terrain/src/terrain_exporter.cpp`
- `src/go2_terrain/test/test_terrain_algorithms.cpp`
- `src/go2_terrain/config/terrain_export.yaml` (documentation only)

Behavioral changes:

1. A trajectory-ground candidate that already passed base-height, low-column
   support, local slope continuity and height-tolerance checks can seed ground
   reconstruction without an additional PMF overlap.
2. PMF remains required for long-gap re-anchoring and is still reported for
   diagnostics.
3. Plane-fit quality is measured as retained fitted cells divided by accepted,
   trajectory-anchored propagation cells.
4. Ground connectivity quality may use the physically traversed, gap-bounded
   trajectory corridor to connect sparse fitted pixels. This mask is used only
   for the quality metric; it does not create elevation, slope, cost, free-space
   or obstacle evidence.
5. Existing 35 degree ground-growth limit, 5 cm step barrier, ceiling exclusion,
   relative obstacle envelope, atomic output transaction and checksums remain
   unchanged.

## Verification

Build and tests:

```text
go2_terrain build: passed
catkin tests: 163 tests, 0 errors, 0 failures, 0 skipped
workspace package/launch validation: passed
```

`lab_202609091450` final export:

```text
trajectory trace: 216/233 (92.7%)
trajectory seeds: 930
accepted propagation cells: 4094
fitted ground cells: 3492
fit retention: 85.3%
trajectory-anchored component: 100.0%
baseline reconstructed-ground/free: 19.3%
baseline known preserved: 21911/21911
trajectory corridor known: 2325/2325 (100.0%)
obstacle cells: 1125
terrain asset checksums: 15/15 passed
```

`lab_202609091625` final-version regression export:

```text
trajectory trace: 223/241 (92.5%)
trajectory seeds: 1212
accepted propagation cells: 4176
fitted ground cells: 3450
fit retention: 82.6%
trajectory-anchored component: 97.3%
baseline reconstructed-ground/free: 19.0%
baseline known preserved: 23303/23303
trajectory corridor known: 2177/2177 (100.0%)
obstacle cells: 736
terrain asset checksums: 15/15 passed
```

Both maps pass `validate_terrain_map.py`. No staging baseline files, export
processes or ROS masters remain, and the stack lock is available.

## Future maps

The fix is in the shared exporter implementation and therefore applies to all
new terrain-enabled mapping sessions. A future map still fails closed when its
input is genuinely unsafe or incomplete, for example when the source checksum
is wrong, no usable trajectory exists, the base height is outside 0.20-0.55 m,
trajectory tracing falls below 80%, plane fitting retains below 30%, connected
trajectory-anchored quality falls below 60%, or asset validation fails.

Normal command:

```bash
run_go2 export-map <map_name>
```

## Backup and rollback

Backup directory:

```text
/home/nvidia/go2_nav_ws/backups/codex_20260909_export_failure_fix
```

It contains the pre-fix sources and an immutable copy of the original
`lab_202609091450` PCD, trajectory, marker and mapping snapshot. The
`after_seed_fix` subdirectory records the intermediate diagnostic/seed-only
state.
