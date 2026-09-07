#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
WS_ROOT="$(dirname "${SCRIPT_PATH}")"

# Ubuntu 20.04 images shipped before June 2025 often contain an expired ROS
# repository key. Install the current Open Robotics key bundled with this
# deployment before refreshing package indexes.
sudo apt-key add "${WS_ROOT}/docs/ros-archive-key.asc"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libgoogle-glog-dev \
  libomp-dev \
  libpcl-dev \
  libyaml-cpp-dev \
  python3-rosdep \
  ros-noetic-diagnostic-updater \
  ros-noetic-eigen-conversions \
  ros-noetic-map-server \
  ros-noetic-move-base \
  ros-noetic-navigation \
  ros-noetic-pcl-ros \
  ros-noetic-teb-local-planner \
  ros-noetic-tf-conversions

# livox_ros_driver2 and Livox-SDK2 must be kept at the same revision.  Building
# the SDK bundled in this workspace avoids accidentally using an older Jetson
# image copy from /usr/local (which lacks newer MID-360 packet definitions).
LIVOX_SDK_SOURCE="${WS_ROOT}/src/third_party/Livox-SDK2"
LIVOX_SDK_BUILD="${WS_ROOT}/.vendor_build/livox_sdk2"
cmake -S "${LIVOX_SDK_SOURCE}" -B "${LIVOX_SDK_BUILD}" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${LIVOX_SDK_BUILD}" \
  --target livox_lidar_sdk_static livox_lidar_sdk_shared -- -j2
sudo cmake --install "${LIVOX_SDK_BUILD}"
sudo ldconfig

echo "System and bundled Livox SDK2 dependencies installed. Run ./build_workspace.sh next."
