# Robot 2: robust live ground-height fitting

Robot 2 standing capture reproduced intermittent false low-height rejection:
22 of 338 recorded Patchwork frame pairs produced a fitted sensor height below
0.43 m, despite a median of about 0.48 m. Enlarging the selected ground set
made the estimate worse. Least-squares-initialized Huber IRLS could be pulled
by a spatial cluster of candidate points inconsistent with the majority floor.

The height estimator now first seeks deterministic three-point plane consensus
(96 trials). It uses that subset only with at least 70% of the original samples
and at least 12 samples, then applies the existing planar-spread, Huber fit,
residual, and physical-height checks. Without a majority it keeps the prior
fit and rejection path. The consensus does not use nominal sensor height as
a preference. No cells are added; ground selection, local obstacle/clearing
classification, all health thresholds and hysteresis, mapping, global slope
costs, chassis interfaces and gait policy are unchanged.

`ground_plane_candidate_samples` reports samples before consensus;
`ground_plane_samples` reports those actually used in fitting.

## Evidence and scope

- Source/model regression tests: 45 passed, including flat/sloped ground with
  clustered contamination, low stance, disconnected components, steps, boxes,
  walls and ceiling handling.
- Standing capture: old fitted height 0.372–0.497 m, 22/338 below minimum;
  updated estimate 0.461–0.503 m, 0/338 below minimum.
- Full updated guard replay, after two-second warm-up: 471 healthy diagnostics
  and 7 held-healthy transient geometry diagnostics; no reported gate closure.
  Maximum processing time 5.294 ms.
- Low-posture capture: all 206 frames remain below the unchanged 0.43 m limit
  in the candidate estimator (estimated 0.231–0.261 m).
- Built the deployed guard and model test targets on Robot 2.

This is a fix for the false height failures reproduced in the standing capture.
It does not prove the separate prolonged sparse-ground failure observed during
the previous evening's navigation is solved, nor does stationary replay replace
walking/turning/obstacle field acceptance. No real SDK was enabled and no motion
or gait command was sent during this work.

## Files and replay

Changed runtime: `src/go2_terrain/include/go2_terrain/terrain_model.hpp`,
`src/go2_terrain/src/terrain_guard.cpp`. Regression coverage lives in
`src/go2_terrain/test/terrain_model_test.cpp`.

Robot evidence directory:

```text
/home/unitree/robot2_terrain_replay_20260911_1jhy0H
```

It contains `standing.bag`, `stationary.bag`, `params.yaml`, pre-change
`before-height-fit.tar.gz`, build/test logs and replay results. The full
workspace backup remains:

```text
/home/unitree/robot2_before_scout_review_20260911_4CgXkF/workspace.tar.gz
```

`tools/replay_terrain_guard.py` runs only the guard on a dedicated ROS master
at localhost:11321. It never creates a navigation or SDK process. Example:

```bash
ROS_MASTER_URI=http://localhost:11321 roscore -p 11321
# In another terminal with ROS Noetic and this workspace sourced:
ROS_MASTER_URI=http://localhost:11321 python3 tools/replay_terrain_guard.py \
  /home/unitree/robot2_terrain_replay_20260911_1jhy0H/standing.bag \
  /home/unitree/go2_nav_ws /tmp/standing-replay.json
```

`tools/terrain_frame_probe.cpp` and `tools/audit_terrain_bag.py` are offline
stage-analysis aids. The probe intentionally uses the recorded Robot 2 default
grid/configuration; it is not a generic parameter-aware validator.

Normal operator commands remain unchanged. Start navigation initially disabled,
check localization and terrain readiness, then have the operator enable and
issue a fresh goal for field testing.
