#!/usr/bin/env bash
set -eo pipefail
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
WS_ROOT="$(dirname "${SCRIPT_PATH}")"
source /opt/ros/noetic/setup.bash
set -u
cd "${WS_ROOT}"

available_kb="$(df -Pk "${WS_ROOT}" | awk 'NR == 2 {print $4}')"
if [[ ! "${available_kb}" =~ ^[0-9]+$ ]] || (( available_kb < 5 * 1024 * 1024 )); then
  echo "Build refused: at least 5 GB free space is required in ${WS_ROOT}." >&2
  echo "ROS logs are not deleted automatically; inspect ${HOME}/.ros/log manually." >&2
  exit 3
fi

# Patchwork++ is intentionally built as a single online wrapper target. Its
# vendored demos and embedded JSK source are retained but excluded by CMake.
catkin_make -j1
set +u
source "${WS_ROOT}/devel/setup.bash"
set -u
rospack profile
"${WS_ROOT}/validate_workspace.sh"

USER_BIN="${HOME}/.local/bin"
USER_COMMAND="${USER_BIN}/run_go2"
mkdir -p "${USER_BIN}"
USER_COMMAND_TMP="${USER_COMMAND}.tmp.$$"
ln -s "${WS_ROOT}/run_go2" "${USER_COMMAND_TMP}"
mv -Tf "${USER_COMMAND_TMP}" "${USER_COMMAND}"
if [[ "$(readlink -f "${USER_COMMAND}")" != "${WS_ROOT}/run_go2" ]]; then
  echo "Failed to activate the workspace run_go2 command." >&2
  exit 4
fi
echo "User command installed atomically: ${USER_COMMAND} -> ${WS_ROOT}/run_go2"
