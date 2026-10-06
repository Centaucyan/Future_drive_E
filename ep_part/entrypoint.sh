#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ep_ws/install/setup.bash
exec "$@"
