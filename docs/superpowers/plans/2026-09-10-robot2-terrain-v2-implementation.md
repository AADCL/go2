# Robot 2 Terrain-Aware Navigation V2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deploy the first robot's tested V2.0.1 terrain-aware mapping and navigation algorithms to Robot 2 while preserving Robot 2 hardware, network, control, and legacy-map behavior.

**Architecture:** Start from tag `v2.0.1` and apply a small Robot 2 hardware-profile patch instead of merging V2 into the old copied workspace. Build and test the clean source in a staging directory, preserve all existing maps and the complete old workspace, then activate at `/home/unitree/go2_nav_ws` and rebuild at the final absolute path. Terrain cost is loaded only in the global costmap; the local costmap receives ground-relative obstacle and clearing clouds without a slope layer.

**Tech Stack:** Ubuntu 20.04, ROS Noetic/catkin, C++14, Python 3, PCL, FAST-LIO, Patchwork++, NDT-OMP, move_base, GlobalPlanner, TEB, Unitree SDK2, Bash, Git.

**Spec:** `docs/superpowers/specs/2026-09-10-robot2-terrain-v2-design.md`

## Global Constraints

- Algorithm source is exactly tag `v2.0.1`, commit `7d01bb1`.
- Robot 2 management is `unitree@192.168.50.111`.
- Livox host is `192.168.1.50` on `eth0`; Mid-360 is `192.168.1.168`.
- GO2 DDS uses `go2dds` with `192.168.123.18`.
- Existing maps and `/home/unitree/go2_nav_ws` remain recoverable.
- Existing maps without terrain metadata continue through legacy mode.
- No automated step may enable the real SDK bridge or command chassis motion.
- Build with one compile job on the 7.2 GiB Orin Nano.

---

### Task 1: Add Robot 2 profile validation before changing configuration

**Files:**
- Modify: `validate_workspace.sh`

**Interfaces:**
- Consumes: the V2 workspace tree and Robot 2 fixed profile from the design.
- Produces: static validation that fails when Robot 1 paths, interfaces, or radar addresses remain active.

- [ ] **Step 1: Change validation expectations first**

Make validation relocatable so it tests the staging tree rather than silently
testing the old final path:

```bash
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
WS_ROOT="$(dirname "${SCRIPT_PATH}")"
```

Replace the two launch checks for `eth0` with `go2dds`. Add exact checks for:

```bash
grep -q 'WS_ROOT="$(dirname "${SCRIPT_PATH}")"' "${WS_ROOT}/run_go2"
grep -q 'GO2_INTERFACE="${GO2_INTERFACE:-go2dds}"' "${WS_ROOT}/run_go2"
grep -q 'livox_interface: eth0' "${WS_ROOT}/src/go2_core/config/robot.yaml"
grep -q 'livox_host_ip: 192.168.1.50' "${WS_ROOT}/src/go2_core/config/robot.yaml"
grep -q 'livox_device_ip: 192.168.1.168' "${WS_ROOT}/src/go2_core/config/robot.yaml"
grep -q 'go2_interface: go2dds' "${WS_ROOT}/src/go2_core/config/robot.yaml"
grep -q 'go2_host_ip: 192.168.123.18' "${WS_ROOT}/src/go2_core/config/robot.yaml"
grep -q '"ip" : "192.168.1.168"' \
  "${WS_ROOT}/src/third_party/livox_ros_driver2/config/MID360_config.json"
```

- [ ] **Step 2: Verify the profile checks fail against unmodified V2.0.1**

Run static commands from the repository root:

```bash
grep -q 'WS_ROOT="$(dirname "${SCRIPT_PATH}")"' run_go2
grep -q 'GO2_INTERFACE="${GO2_INTERFACE:-go2dds}"' run_go2
grep -q 'livox_device_ip: 192.168.1.168' src/go2_core/config/robot.yaml
```

Expected: all three return non-zero because V2.0.1 still carries Robot 1 values.

- [ ] **Step 3: Commit the failing profile validation**

```bash
git add validate_workspace.sh
git commit -m "test: require Robot 2 deployment profile"
```

### Task 2: Apply only the Robot 2 hardware profile

**Files:**
- Modify: `run_go2`
- Modify: `build_workspace.sh`
- Modify: `validate_workspace.sh`
- Modify: `src/go2_core/config/robot.yaml`
- Modify: `src/go2_bringup/launch/navigation.launch`
- Modify: `src/go2_control/launch/control.launch`
- Modify: `src/go2_control/src/sdk_bridge.cpp`
- Modify: `src/go2_control/CMakeLists.txt`
- Modify: `src/third_party/livox_ros_driver2/config/MID360_config.json`

**Interfaces:**
- Consumes: V2.0.1 algorithms and the validation written in Task 1.
- Produces: a V2 source tree that addresses Robot 2 devices without changing ROS topics or command semantics.

- [ ] **Step 1: Update absolute operational paths**

Make `run_go2`, `build_workspace.sh`, and `validate_workspace.sh` resolve their
workspace from their own real path:

```bash
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
WS_ROOT="$(dirname "${SCRIPT_PATH}")"
USER_BIN="${HOME}/.local/bin"
```

Change ROS-log guidance from `/home/nvidia/.ros/log` to `/home/unitree/.ros/log`.

- [ ] **Step 2: Update DDS defaults**

Set the default interface to `go2dds` in all four control entry points:

```text
run_go2
src/go2_bringup/launch/navigation.launch
src/go2_control/launch/control.launch
src/go2_control/src/sdk_bridge.cpp
```

Keep the explicit `GO2_INTERFACE` environment override available.

- [ ] **Step 3: Update the Robot 2 network profile**

Set `src/go2_core/config/robot.yaml` to:

```yaml
network:
  livox_interface: eth0
  livox_host_ip: 192.168.1.50
  livox_device_ip: 192.168.1.168
  go2_interface: go2dds
  go2_host_ip: 192.168.123.18
```

- [ ] **Step 4: Update the active Livox configuration**

Keep all host fields in `MID360_config.json` at `192.168.1.50` and set the only active `lidar_configs[].ip` to `192.168.1.168`.

- [ ] **Step 5: Preserve Orin Nano SDK/DDS linkage**

Resolve Unitree libraries from `${CMAKE_SYSTEM_PROCESSOR}` and create missing
SONAME links in the vendored architecture directory with CMake:

```cmake
set(UNITREE_SDK_LIB_DIR
  "${UNITREE_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}")
set(UNITREE_DDS_LIB_DIR
  "${UNITREE_SDK_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")

foreach(DDS_LIBRARY ddsc ddscxx)
  set(DDS_LIBRARY_FILE "${UNITREE_DDS_LIB_DIR}/lib${DDS_LIBRARY}.so")
  set(DDS_SONAME_LINK "${DDS_LIBRARY_FILE}.0")
  if(NOT EXISTS "${DDS_LIBRARY_FILE}")
    message(FATAL_ERROR "Bundled Unitree DDS library not found: ${DDS_LIBRARY_FILE}")
  endif()
  if(NOT EXISTS "${DDS_SONAME_LINK}")
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E create_symlink
              "lib${DDS_LIBRARY}.so" "${DDS_SONAME_LINK}"
      RESULT_VARIABLE DDS_LINK_RESULT)
    if(NOT DDS_LINK_RESULT EQUAL 0)
      message(FATAL_ERROR "Failed to create ${DDS_SONAME_LINK}")
    endif()
  endif()
endforeach()
```

Link `libunitree_sdk2.a`, `libddscxx.so`, and `libddsc.so` through those two
variables. Do not replace Unitree SDK2 or system CycloneDDS versions.

- [ ] **Step 6: Run the profile checks again**

Run:

```bash
bash -n run_go2
bash -n build_workspace.sh
bash -n validate_workspace.sh
grep -q 'GO2_INTERFACE="${GO2_INTERFACE:-go2dds}"' run_go2
grep -q 'livox_device_ip: 192.168.1.168' src/go2_core/config/robot.yaml
grep -q 'network_interface" default="go2dds"' src/go2_bringup/launch/navigation.launch
grep -q 'network_interface" default="go2dds"' src/go2_control/launch/control.launch
grep -q '"ip" : "192.168.1.168"' src/third_party/livox_ros_driver2/config/MID360_config.json
```

Expected: every command succeeds.

- [ ] **Step 7: Verify the feature diff did not change algorithms**

Run:

```bash
git diff --stat v2.0.1 -- \
  src/go2_mapping src/go2_terrain src/go2_localization \
  src/go2_navigation/config src/go2_navigation/src
```

Expected: no changes in mapping, terrain, localization, planner parameters, or navigation algorithms. Only hardware-profile, build, validation, and documentation files differ from V2.0.1.

- [ ] **Step 8: Commit the hardware profile**

```bash
git add run_go2 build_workspace.sh validate_workspace.sh \
  src/go2_core/config/robot.yaml \
  src/go2_bringup/launch/navigation.launch \
  src/go2_control/launch/control.launch \
  src/go2_control/src/sdk_bridge.cpp \
  src/go2_control/CMakeLists.txt \
  src/third_party/livox_ros_driver2/config/MID360_config.json
git commit -m "feat: adapt terrain V2 for Robot 2 Orin Nano"
```

### Task 3: Document Robot 2 V2 deployment without altering runtime behavior

**Files:**
- Create: `DEPLOYMENT_ROBOT2_V2.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: the exact profile and rollback design.
- Produces: operator-facing commands that match the installed paths and hardware.

- [ ] **Step 1: Write a Robot 2 deployment document**

Document the fixed IP/interface table, V2.0.1 base commit, staging/backup locations, build commands, legacy-map fallback, terrain-map export, non-motion validation, supervised field acceptance, and rollback commands.

- [ ] **Step 2: Link the Robot 2 document from README**

Add a short Robot 2 section that identifies this integration branch as Orin Nano profile data layered over V2.0.1 algorithms.

- [ ] **Step 3: Verify stale active instructions are absent**

Run:

```bash
if grep -nE '/home/nvidia|192\.168\.1\.191|GO2_INTERFACE.*eth0' \
  DEPLOYMENT_ROBOT2_V2.md run_go2 build_workspace.sh validate_workspace.sh; then
  exit 1
fi
```

Expected: no output and exit status zero.

- [ ] **Step 4: Commit documentation**

```bash
git add DEPLOYMENT_ROBOT2_V2.md README.md
git commit -m "docs: add Robot 2 terrain V2 deployment guide"
```

### Task 4: Audit and stage the second robot without touching the active workspace

**Files:**
- Remote create: `/home/unitree/go2_nav_ws_v2_staging/`
- Remote create: `/home/unitree/robot2_upgrade_<timestamp>/preflight.txt`
- Remote create: `/home/unitree/robot2_upgrade_<timestamp>/robot2-profile.patch`

**Interfaces:**
- Consumes: the committed integration patch and public V2.0.1 repository.
- Produces: a clean, traceable staging checkout plus a preflight record.

- [ ] **Step 1: Record preflight state**

Record hostname, timestamp, `ip -br addr`, routes, radar ping, free disk, memory, ROS process list, map directory listing, and SHA-256 checksums of the active workspace's key configuration files.

- [ ] **Step 2: Confirm the active system is idle**

Run:

```bash
pgrep -af 'roscore|rosmaster|roslaunch|move_base|fastlio|livox|sdk_bridge'
```

Expected: no active ROS stack. If anything is active, stop deployment without killing it.

- [ ] **Step 3: Verify GitHub access from Robot 2**

Run:

```bash
git ls-remote https://github.com/AADCL/go2.git refs/tags/v2.0.1
```

Expected: tag resolves to the V2.0.1 commit chain.

- [ ] **Step 4: Create the clean staging checkout**

Clone tag `v2.0.1` into a new staging directory. Refuse to continue if the exact staging path already exists; do not delete an unknown directory.

- [ ] **Step 5: Apply the reviewed Robot 2 patch**

Transfer a `git format-patch` generated from the local integration branch, apply it with `git am`, and verify the resulting commit and clean working tree.

- [ ] **Step 6: Preserve maps in staging**

Copy `/home/unitree/go2_nav_ws/maps/` into staging with metadata preserved. Confirm the source and destination map directory names and file counts match. Do not run terrain export against old maps that lack `traversed_path_map.pcd`.

### Task 5: Build and test the staging workspace on Orin Nano

**Files:**
- Remote generated: `/home/unitree/go2_nav_ws_v2_staging/build/`
- Remote generated: `/home/unitree/go2_nav_ws_v2_staging/devel/`
- Remote create: `/home/unitree/robot2_upgrade_<timestamp>/staging-build.log`
- Remote create: `/home/unitree/robot2_upgrade_<timestamp>/staging-tests.log`

**Interfaces:**
- Consumes: staged source and installed ROS dependencies.
- Produces: compiled V2 binaries and complete automated-test evidence.

- [ ] **Step 1: Resolve dependencies**

Run `rosdep check --from-paths src --ignore-src`. Install only reported missing ROS/system packages; do not perform a general operating-system upgrade.

- [ ] **Step 2: Build with one job**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DROS_EDITION=ROS1 -DCATKIN_ENABLE_TESTING=ON -j1
```

Expected: build exits zero and produces all GO2, Patchwork++, terrain, FAST-LIO, Livox, and control targets.

- [ ] **Step 3: Run mapping and terrain tests**

Run:

```bash
catkin_make run_tests_go2_mapping run_tests_go2_terrain -j1
catkin_test_results --verbose build/test_results
```

Expected: all C++ and Python tests pass with zero failures.

- [ ] **Step 4: Run workspace validation at the staging path**

Run the relocatable validation script directly from staging:

```bash
cd /home/unitree/go2_nav_ws_v2_staging
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./validate_workspace.sh
```

Expected: every package, launch, mapping, terrain, global-layer, local-layer,
control-safety, and Robot 2 profile assertion succeeds against staging.

- [ ] **Step 5: Verify global/local terrain separation**

Run launch-node and parameter-file checks proving:

```text
global_costmap_terrain.yaml -> contains Go2TerrainLayer
local_costmap_terrain.yaml  -> does not contain Go2TerrainLayer
terrain mode                -> starts cloud adapter, Patchwork++, terrain guard
legacy mode                 -> does not require terrain metadata
```

### Task 6: Activate the validated source with a recoverable directory swap

**Files:**
- Remote move: `/home/unitree/go2_nav_ws` to `/home/unitree/go2_nav_ws_backup_<timestamp>`
- Remote move: `/home/unitree/go2_nav_ws_v2_staging` to `/home/unitree/go2_nav_ws`
- Remote update: `/home/unitree/.local/bin/run_go2`

**Interfaces:**
- Consumes: passing staging build and untouched active workspace.
- Produces: V2.0.1 terrain-aware source at the established path with a complete rollback copy.

- [ ] **Step 1: Recheck system idle state**

Repeat the ROS process check immediately before activation. Abort on any active stack.

- [ ] **Step 2: Perform exact-path directory moves**

Resolve and print the three exact absolute paths. Move the old workspace to its timestamped backup, then move staging into the final path. Never use a wildcard or recursive deletion.

- [ ] **Step 3: Move staging build products aside**

Move `build/` and `devel/` to `.staging-build/` and `.staging-devel/` inside the new workspace because catkin embeds absolute paths. Do not delete them until final-path compilation succeeds.

- [ ] **Step 4: Rebuild at the final absolute path**

Run `/home/unitree/go2_nav_ws/build_workspace.sh`, capture the full log, and verify `/home/unitree/.local/bin/run_go2` resolves to the new workspace.

- [ ] **Step 5: Roll back automatically on build failure**

If final-path build or validation fails, move the failed new directory aside, restore the timestamped old directory to `/home/unitree/go2_nav_ws`, and restore the old `run_go2` symlink. Report the failure without attempting additional fixes on the robot.

### Task 7: Perform non-motion runtime acceptance

**Files:**
- Remote create: `/home/unitree/robot2_upgrade_<timestamp>/runtime-check.log`

**Interfaces:**
- Consumes: activated V2 workspace and powered Mid-360.
- Produces: evidence that perception and launch wiring work without chassis motion.

- [ ] **Step 1: Verify profile and packages**

Confirm `.168` radar reachability, `rospack find` for all packages, expected Git commit, map inventory, and `run_go2 status` behavior with no ROS master.

- [ ] **Step 2: Start mapping while the robot remains stationary**

Start `run_go2 mapping robot2_v2_smoke` without the real SDK bridge. Verify within a bounded observation period:

```text
/livox/lidar
/livox/imu
/lio/odometry
/lio/cloud_registered
/odom_nav
/cloud_registered_odom
/go2_mapping/status
```

Record topic type, publisher, frame, and measured rate. Do not walk the robot or claim a valid terrain map from this stationary smoke test.

- [ ] **Step 3: Stop mapping cleanly**

Send SIGINT to the mapping launch, wait for clean shutdown, and verify no real SDK bridge was started and no control-enable topic became true.

- [ ] **Step 4: Verify legacy-map compatibility without real control**

Start navigation on one preserved map in mock mode. Confirm map server, NDT, move_base, TEB, mock bridge, and legacy costmaps start. Do not publish a goal and do not run `run_go2 enable`.

- [ ] **Step 5: Stop navigation and archive evidence**

Stop with SIGINT, verify all started processes exit, and keep the backup workspace and logs for supervised field acceptance.

### Task 8: Handoff supervised field acceptance and rollback instructions

**Files:**
- Modify: `DEPLOYMENT_ROBOT2_V2.md` only if observed commands differ from the documented commands.

**Interfaces:**
- Consumes: deployment and non-motion runtime evidence.
- Produces: exact operator steps for mapping a ramp, exporting terrain, planning, and deciding whether to keep or roll back V2.

- [ ] **Step 1: Provide the mapping acceptance sequence**

The operator maps a route containing flat floor, a ramp, ceiling returns, and a moving person; saves the map; exports terrain; and validates all terrain artifacts and checksums.

- [ ] **Step 2: Provide the planning acceptance sequence**

The operator first launches mock navigation, verifies the global slope-cost layer and absence of a local slope layer, then performs a separately supervised real-SDK run with an emergency stop available.

- [ ] **Step 3: State pass/fail criteria**

Pass requires transient objects not to persist in the saved map, ceiling returns not to become ground, global paths to account for slope cost, local paths not to receive slope cost, and unchanged legacy navigation/control commands. Any regression triggers the documented directory-swap rollback.

- [ ] **Step 4: Keep the old workspace until field acceptance passes**

Do not remove `/home/unitree/go2_nav_ws_backup_<timestamp>` as part of this task.
