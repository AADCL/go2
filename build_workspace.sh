#!/usr/bin/env bash
set -eo pipefail
WS_ROOT="/home/nvidia/go2_nav_ws"
source /opt/ros/noetic/setup.bash
cd "${WS_ROOT}"
catkin_make -j1
source "${WS_ROOT}/devel/setup.bash"
set -u
rospack profile
"${WS_ROOT}/validate_workspace.sh"

USER_BIN="/home/nvidia/.local/bin"
USER_COMMAND="${USER_BIN}/run_go2"
mkdir -p "${USER_BIN}"
if [[ -L "${USER_COMMAND}" || ! -e "${USER_COMMAND}" ]]; then
  ln -sfn "${WS_ROOT}/run_go2" "${USER_COMMAND}"
  echo "User command installed: ${USER_COMMAND}"
else
  echo "Warning: ${USER_COMMAND} exists and is not a symlink; left unchanged." >&2
fi
