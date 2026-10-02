#!/usr/bin/env bash
set -e
MANAGEMENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$MANAGEMENT_DIR/ros-env.sh"
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1 port:=9090 send_action_goals_in_new_thread:=true call_services_in_new_thread:=true default_call_service_timeout:=5.0
