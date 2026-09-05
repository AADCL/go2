#!/usr/bin/env bash
set -eo pipefail
WS_ROOT="/home/nvidia/go2_nav_ws"
source /opt/ros/noetic/setup.bash
source "${WS_ROOT}/devel/setup.bash"
set -u

for package in go2_core go2_mapping go2_localization go2_navigation go2_control go2_bringup fast_lio livox_ros_driver2; do
  rospack find "${package}" >/dev/null
done

roslaunch --nodes go2_bringup mapping.launch map_name:=site01 >/tmp/go2_mapping_nodes.txt
roslaunch --nodes go2_bringup navigation.launch map_name:=site01 >/tmp/go2_navigation_nodes.txt

if grep -RIE 'scout/odom|scout_global|livox_fastlio|waypoint_generator|ego_planner|planning/pos_cmd|camera_init' \
    "${WS_ROOT}/src/go2_"* \
    --include='*.launch' --include='*.yaml' --include='*.cpp' \
    --include='*.py' --include='*.sh'; then
  echo "Legacy Scout/EGO reference found in active configuration" >&2
  exit 1
fi

echo "Workspace package and launch validation passed."
