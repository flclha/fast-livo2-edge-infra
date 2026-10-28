#!/usr/bin/env bash
set -euo pipefail

RUNTIME_TIMESHARE=/home/nuc/timeshare
INTERNAL_TIMESHARE=/home/nuc/timeshare.livox_driver_internal
READY_FILE=/tmp/fast_livo_livox_timeshare_namespace.ready

# This script runs as root inside a private mount namespace. The official
# driver can keep its historical writer without sharing an inode with the
# timestamp consumed by the camera.
mount --bind "$INTERNAL_TIMESHARE" "$RUNTIME_TIMESHARE"
printf '%s\n' "$(stat -c %i "$RUNTIME_TIMESHARE")" > "$READY_FILE"

exec setpriv --reuid=nuc --regid=nuc --init-groups env \
  HOME=/home/nuc USER=nuc LOGNAME=nuc \
  bash --noprofile --norc -c 'unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES CATKIN_SHELL PYTHONPATH; source /opt/ros/noetic/setup.bash; source /home/nuc/ws_livox/devel/setup.bash; exec roslaunch livox_ros_driver2 msg_MID360.launch'
