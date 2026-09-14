#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: 未检测到 /opt/ros/jazzy/setup.bash"
  echo "请先按 ROS 2 官方文档安装 Ubuntu 24.04 对应的 ROS 2 Jazzy Desktop。"
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

sudo apt-get update
sudo apt-get install -y \
  git rsync jq graphviz \
  build-essential cmake \
  python3-pip python3-rosdep python3-vcstool \
  python3-colcon-common-extensions \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-xacro ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-vision-msgs ros-jazzy-cv-bridge \
  ros-jazzy-tf2-tools ros-jazzy-rqt-graph

sudo apt-get install -y ros-jazzy-fields2cover || true

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

WS="${SANITATION_WS:-$HOME/sanitation_ws}"
mkdir -p "$WS/src" "$WS/artifacts"
echo "Workspace: $WS"
