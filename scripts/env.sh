#!/usr/bin/env bash

WS_ROOT_DEFAULT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="${WS_ROOT:-$WS_ROOT_DEFAULT}"

if [ ! -d /opt/ros/humble ]; then
  echo "[env] 没找到 /opt/ros/humble" >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/humble/setup.bash
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
fi

if [ -f "$WS_ROOT/config/fastdds_wsl.xml" ]; then
  export FASTDDS_DEFAULT_PROFILES_FILE="$WS_ROOT/config/fastdds_wsl.xml"
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS_ROOT/config/fastdds_wsl.xml"
fi
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"

echo "[env] ROS=humble  WS=$WS_ROOT  DDS_profile=${FASTDDS_DEFAULT_PROFILES_FILE:-默认}  LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
