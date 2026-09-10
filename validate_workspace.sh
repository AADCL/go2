#!/usr/bin/env bash
set -eo pipefail
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
WS_ROOT="$(dirname "${SCRIPT_PATH}")"
source /opt/ros/noetic/setup.bash
source "${WS_ROOT}/devel/setup.bash"
set -u

command -v flock >/dev/null

test -f \
  "${WS_ROOT}/src/third_party/jsk_recognition_msgs/msg/PolygonArray.msg" || {
  echo "Vendored jsk_recognition_msgs compatibility package is missing" >&2
  exit 1
}

cmp -s \
  "${WS_ROOT}/src/third_party/livox_ros_driver2/package_ROS1.xml" \
  "${WS_ROOT}/src/third_party/livox_ros_driver2/package.xml" || {
  echo "Livox ROS1 package.xml was not prepared before catkin discovery" >&2
  exit 1
}

for package in go2_core go2_mapping go2_terrain go2_localization go2_navigation go2_control go2_bringup patchworkpp fast_lio livox_ros_driver2; do
  rospack find "${package}" >/dev/null
done

roslaunch --nodes go2_bringup mapping.launch map_name:=site01 >/tmp/go2_mapping_nodes.txt
roslaunch --nodes go2_bringup navigation.launch map_name:=site01 terrain_enabled:=false >/tmp/go2_navigation_legacy_nodes.txt
roslaunch --nodes go2_bringup navigation.launch map_name:=site01 terrain_enabled:=true \
  terrain_metadata:="${WS_ROOT}/maps/site01/terrain_2p5d.yaml" \
  >/tmp/go2_navigation_terrain_nodes.txt

grep -qx '/go2_map_builder' /tmp/go2_mapping_nodes.txt
grep -qx '/move_base' /tmp/go2_navigation_legacy_nodes.txt
grep -qx '/go2_terrain_cloud_adapter' /tmp/go2_navigation_terrain_nodes.txt
grep -qx '/go2_terrain_guard' /tmp/go2_navigation_terrain_nodes.txt
grep -qx '/go2_navigation_patchworkpp' /tmp/go2_navigation_terrain_nodes.txt

rosrun go2_terrain validate_terrain_map.py --help >/dev/null
rosrun go2_terrain validate_terrain_map.py --help | \
  grep -q -- '--expected-export-id'
grep -q 'network_interface" default="go2dds"' \
  "${WS_ROOT}/src/go2_bringup/launch/navigation.launch"
grep -q 'network_interface" default="go2dds"' \
  "${WS_ROOT}/src/go2_control/launch/control.launch"
grep -q 'WS_ROOT="$(dirname "${SCRIPT_PATH}")"' "${WS_ROOT}/run_go2"
grep -q 'GO2_INTERFACE="${GO2_INTERFACE:-go2dds}"' "${WS_ROOT}/run_go2"
grep -q 'livox_interface: eth0' \
  "${WS_ROOT}/src/go2_core/config/robot.yaml"
grep -q 'livox_host_ip: 192.168.1.50' \
  "${WS_ROOT}/src/go2_core/config/robot.yaml"
grep -q 'livox_device_ip: 192.168.1.168' \
  "${WS_ROOT}/src/go2_core/config/robot.yaml"
grep -q 'go2_interface: go2dds' \
  "${WS_ROOT}/src/go2_core/config/robot.yaml"
grep -q 'go2_host_ip: 192.168.123.18' \
  "${WS_ROOT}/src/go2_core/config/robot.yaml"
grep -q '"ip" : "192.168.1.168"' \
  "${WS_ROOT}/src/third_party/livox_ros_driver2/config/MID360_config.json"
grep -q 'inflation_radius: 0.10' \
  "${WS_ROOT}/src/go2_navigation/config/global_costmap.yaml"
grep -q 'inflation_radius: 0.10' \
  "${WS_ROOT}/src/go2_navigation/config/local_costmap.yaml"
grep -q 'inflation_radius: 0.10' \
  "${WS_ROOT}/src/go2_navigation/config/global_costmap_terrain.yaml"
grep -q 'inflation_radius: 0.10' \
  "${WS_ROOT}/src/go2_navigation/config/local_costmap_terrain.yaml"
grep -q 'go2_terrain::Go2TerrainLayer' \
  "${WS_ROOT}/src/go2_navigation/config/global_costmap_terrain.yaml"
if grep -q 'Go2TerrainLayer' \
    "${WS_ROOT}/src/go2_navigation/config/local_costmap.yaml" \
    "${WS_ROOT}/src/go2_navigation/config/local_costmap_terrain.yaml"; then
  echo "Local costmap must not load a terrain layer" >&2
  exit 1
fi
grep -q 'mapping_snapshot.sha256' \
  "${WS_ROOT}/src/go2_mapping/src/map_builder.cpp"
grep -q 'snapshot_writer_busy' \
  "${WS_ROOT}/src/go2_mapping/src/map_builder.cpp"
grep -q 'miss_probability: 0.40' \
  "${WS_ROOT}/src/go2_mapping/config/mapper.yaml"
grep -q 'fine_min_hit_scans: 6' \
  "${WS_ROOT}/src/go2_mapping/config/mapper.yaml"
grep -q 'min_clear_miss_scans: 12' \
  "${WS_ROOT}/src/go2_mapping/config/mapper.yaml"
grep -q 'candidate_timeout: 6.0' \
  "${WS_ROOT}/src/go2_mapping/config/mapper.yaml"
grep -q 'writeMapOutputsAtomically' \
  "${WS_ROOT}/src/go2_mapping/src/occupancy_map_exporter.cpp"
grep -q 'required="true"' \
  "${WS_ROOT}/src/go2_navigation/launch/navigation_stack.launch"
grep -q 'name="go2_sdk_bridge_mock".*output="screen" required="true"' \
  "${WS_ROOT}/src/go2_control/launch/control.launch"
grep -q 'require_terrain_health" value="$(arg terrain_enabled)"' \
  "${WS_ROOT}/src/go2_bringup/launch/navigation.launch"
grep -q 'terrainPermitted' \
  "${WS_ROOT}/src/go2_control/src/velocity_shaper.cpp"
grep -q '/go2_sdk_bridge_mock/enable' "${WS_ROOT}/run_go2"
grep -q '/move_base_internal/cancel' "${WS_ROOT}/run_go2"
grep -q 'map_root:="${WS_ROOT}/maps"' "${WS_ROOT}/run_go2"
grep -q 'map_has_terrain_evidence' "${WS_ROOT}/run_go2"
grep -q -- '--expected-export-id "${terrain_export_id}"' \
  "${WS_ROOT}/run_go2"
grep -q 'export_id:="${terrain_export_id}"' "${WS_ROOT}/run_go2"
grep -q 'input_map_yaml:="${baseline_yaml}"' "${WS_ROOT}/run_go2"
grep -q 'committed map remains unchanged' "${WS_ROOT}/run_go2"
grep -q 'legacy_export_receipt' "${WS_ROOT}/run_go2"
grep -q 'export_id:="${legacy_export_id}"' "${WS_ROOT}/run_go2"
grep -q 'name="export_receipt" value="$(arg export_receipt)"' \
  "${WS_ROOT}/src/go2_mapping/launch/export_occupancy.launch"
grep -q 'name="export_id" value="$(arg export_id)"' \
  "${WS_ROOT}/src/go2_terrain/launch/export_terrain.launch"
grep -q 'preserve_existing_map: true' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_export.yaml"
grep -q 'minimum_traced_trajectory_ratio: 0.80' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_export.yaml"
grep -q 'minimum_ground_observation_ratio: 0.30' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_export.yaml"
grep -q 'minimum_largest_ground_component_ratio: 0.60' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_export.yaml"
grep -q 'max_reanchor_height_from_initial_m: 0.65' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_export.yaml"
grep -q 'minimum_ground_to_baseline_free_ratio: 0.08' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_export.yaml"
grep -q 'minimum_trajectory_corridor_known_ratio: 0.95' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_export.yaml"
grep -q 'verified_trajectory_free' \
  "${WS_ROOT}/src/go2_terrain/src/terrain_algorithms.cpp"
for action_topic in goal cancel status feedback result; do
  grep -q "/move_base/${action_topic}.*move_base_internal/${action_topic}" \
    "${WS_ROOT}/src/go2_navigation/launch/navigation_stack.launch"
done
grep -q 'SimpleActionServer<move_base_msgs::MoveBaseAction>' \
  "${WS_ROOT}/src/go2_navigation/src/navigation_supervisor.cpp"
grep -q 'obstacle_inflation_m: 0.03' \
  "${WS_ROOT}/src/go2_mapping/config/occupancy.yaml"
grep -q 'uprightness_thr: 0.819152044' \
  "${WS_ROOT}/src/go2_terrain/config/patchworkpp_go2.yaml"
grep -q 'max_connected_slope_deg: 35.0' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'steep_unknown_min_slope_deg: 8.0' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'steep_unknown_max_slope_deg: 70.0' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'steep_unknown_min_run_cells: 3' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'steep_unknown_min_points_per_cell: 2' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'steep_unknown_max_step_delta: 0.04' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'steep_unknown_plane_huber: 0.02' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'steep_unknown_max_plane_rmse: 0.025' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'steep_unknown_max_point_residual: 0.05' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'minimum_connected_ground_area_m2: 0.60' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'minimum_near_support_area_m2: 0.18' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'anchor_radius: 1.20' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'minimum_ground_sectors: 2' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'ground_plane_min_connected_cells: 12' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'ground_plane_maximum_radius_m: 1.50' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'ground_plane_min_planar_variance_m2: 0.01' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'ground_plane_huber_m: 0.03' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'ground_plane_max_rmse_m: 0.04' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'ground_plane_irls_iterations: 5' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'open_after_consecutive_healthy_frames: 5' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'max_soft_geometry_failure_frames: 3' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'max_soft_geometry_failure_duration_sec: 0.25' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'min_sensor_height_m: 0.43' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
grep -q 'max_sensor_height_m: 0.59' \
  "${WS_ROOT}/src/go2_terrain/config/terrain_guard_go2.yaml"
for diagnostic_key in ground_plane_fit_status ground_plane_samples \
  estimated_sensor_height_m ground_plane_slope_deg ground_plane_rmse_m \
  frame_health_class health_gate_open consecutive_healthy_frames \
  consecutive_soft_geometry_failures soft_geometry_failure_age_sec; do
  grep -q "${diagnostic_key}" \
    "${WS_ROOT}/src/go2_terrain/src/terrain_guard.cpp"
done
grep -q 'selectGroundComponent' \
  "${WS_ROOT}/src/go2_terrain/src/terrain_guard.cpp"
grep -q 'show_topic "Terrain healthy" /terrain/healthy' "${WS_ROOT}/run_go2"
if grep -qE 'add_executable\((demo|offline_kitti|video)|add_subdirectory\(include/jsk' \
    "${WS_ROOT}/src/third_party/patchworkpp/CMakeLists.txt"; then
  echo "Patchwork++ must build only the GO2 online wrapper" >&2
  exit 1
fi
bash -n "${WS_ROOT}/run_go2"
bash -n "${WS_ROOT}/build_workspace.sh"

if grep -RIE 'scout/odom|scout_global|livox_fastlio|waypoint_generator|ego_planner|planning/pos_cmd|camera_init' \
    "${WS_ROOT}/src/go2_"* \
    --include='*.launch' --include='*.yaml' --include='*.cpp' \
    --include='*.py' --include='*.sh'; then
  echo "Legacy Scout/EGO reference found in active configuration" >&2
  exit 1
fi

echo "Workspace package and launch validation passed."
