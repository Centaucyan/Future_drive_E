#!/usr/bin/env bash
# Source in a dedicated shell; does not change system or platform configuration.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
MANAGEMENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$MANAGEMENT_DIR/.." && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$MANAGEMENT_DIR/fastdds-pc.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export ROS_LOG_DIR="$PROJECT_DIR/log"
