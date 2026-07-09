#!/usr/bin/env bash
# One-time setup for the cloud dev container (GitHub Codespaces or any Docker host).
# Installs dependencies, builds the workspace, and wires up sourcing.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${SCRIPT_DIR}/../rover_ws"

echo "==> Installing dependencies"
apt-get update
# Foxglove bridge lets you visualise topics/TF/pointclouds/costmaps in the
# Foxglove web app (https://app.foxglove.dev) with the sim running headless.
# diagnostic-updater is a runtime dep of robot_localization's ekf_node that the
# base image can miss; install it explicitly so the EKF starts reliably.
apt-get install -y \
  python3-colcon-common-extensions \
  ros-humble-foxglove-bridge \
  ros-humble-teleop-twist-keyboard \
  ros-humble-diagnostic-updater \
  ros-humble-robot-localization \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-velocity-controllers \
  ros-humble-position-controllers

source /opt/ros/humble/setup.bash
rosdep update
# Report (but do not silently swallow) any unresolved dependencies.
rosdep install --from-paths "${WS}/src" --ignore-src -r -y || \
  echo "WARNING: rosdep reported unresolved dependencies (see above)."

echo "==> Building workspace"
cd "${WS}"
colcon build --symlink-install

echo "==> Configuring shell"
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || \
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
grep -qxF "source ${WS}/install/setup.bash" ~/.bashrc || \
  echo "source ${WS}/install/setup.bash" >> ~/.bashrc

echo "==> Done. Open a new terminal, then e.g.:"
echo "    ros2 launch rover_gazebo sim.launch.py headless:=true rviz:=false localization:=true terrain:=true"
echo "    ros2 launch foxglove_bridge foxglove_bridge_launch.xml   # then connect from app.foxglove.dev"
