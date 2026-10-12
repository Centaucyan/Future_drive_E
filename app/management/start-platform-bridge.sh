#!/usr/bin/env bash
set -euo pipefail

BRIDGE_PORT=${BRIDGE_PORT:-8765}
BRIDGE_CONTAINER=${BRIDGE_CONTAINER:-future-drive-nav2-bridge}
BRIDGE_IMAGE=${BRIDGE_IMAGE:-slam-ros-bridge:20260813-3d-map-v2}
PROJECT_APP_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
AUTONOMY_APP_DIR=${AUTONOMY_APP_DIR:-$HOME/.config/lms/runtime/docker-contexts/slam-ros-bridge}
PROFILE_DIR=${PROFILE_DIR:-$HOME/autonomy-studio/runtime/nav2/docker}
BRIDGE_WORKSPACE=${BRIDGE_WORKSPACE:-$PROJECT_APP_DIR/runtime/platform-bridge-workspace}

if curl -fsS --max-time 2 "http://127.0.0.1:${BRIDGE_PORT}/health" >/dev/null 2>&1; then
  echo "Autonomy Studio bridge is already ready: http://127.0.0.1:${BRIDGE_PORT}"
  exit 0
fi

if [ -n "${FASTDDS_PROFILE:-}" ]; then
  PROFILE_PATH=$FASTDDS_PROFILE
else
  PROFILE_PATH=$(find "$PROFILE_DIR" -maxdepth 1 -type f -name 'fastdds-peer-*-d*.xml' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)
fi

[ -n "${PROFILE_PATH:-}" ] && [ -f "$PROFILE_PATH" ] || {
  echo "Fast DDS peer profile was not found." >&2
  echo "Connect the robot in Autonomy Studio once, then run this script again." >&2
  exit 1
}
[ -f "$AUTONOMY_APP_DIR/bridge.py" ] || {
  echo "Autonomy Studio bridge source was not found: $AUTONOMY_APP_DIR/bridge.py" >&2
  exit 1
}
docker image inspect "$BRIDGE_IMAGE" >/dev/null 2>&1 || {
  echo "Autonomy Studio bridge image was not found: $BRIDGE_IMAGE" >&2
  exit 1
}

PROFILE_NAME=$(basename "$PROFILE_PATH")
if [[ $PROFILE_NAME =~ -d([0-9]+)\.xml$ ]]; then
  ROS_DOMAIN=${ROS_DOMAIN_ID:-${BASH_REMATCH[1]}}
else
  ROS_DOMAIN=${ROS_DOMAIN_ID:-0}
fi

mkdir -p "$BRIDGE_WORKSPACE/maps"
docker rm -f "$BRIDGE_CONTAINER" >/dev/null 2>&1 || true

docker run -d \
  --name "$BRIDGE_CONTAINER" \
  --network host \
  --ipc host \
  --restart unless-stopped \
  -e "PORT=$BRIDGE_PORT" \
  -e "ROS_DOMAIN_ID=$ROS_DOMAIN" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/fastdds_profiles.xml \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_profiles.xml \
  -e "BRIDGE_HTTP_URL=http://127.0.0.1:$BRIDGE_PORT" \
  -e "BRIDGE_WS_URL=ws://127.0.0.1:$BRIDGE_PORT" \
  -e SAVED_MAPS_PATH=/workspace/maps \
  -e RTABMAP_DATABASE_PATH=/workspace/maps/rtabmap-active.db \
  -e WORLD_LAYOUT_PATH=/workspace/world-layout.json \
  -e ROBOT_STACK=nav2 \
  -e "CAMERA_IMAGE_TOPIC=${CAMERA_IMAGE_TOPIC:-/image_raw/compressed}" \
  -e RCUTILS_COLORIZED_OUTPUT=1 \
  -e RCUTILS_LOGGING_BUFFERED_STREAM=0 \
  -e PYTHONUNBUFFERED=1 \
  -v /dev/shm:/dev/shm \
  -v "$AUTONOMY_APP_DIR:/app:ro" \
  -v "$BRIDGE_WORKSPACE:/workspace" \
  -v "$PROFILE_PATH:/fastdds_profiles.xml:ro" \
  "$BRIDGE_IMAGE" \
  bash -lc 'source /opt/ros/humble/setup.bash && (python3 -c "import fastapi, uvicorn, websockets" || python3 -m pip install --no-cache-dir -r /app/requirements.txt) && cp /app/bridge.py /workspace/bridge-runtime.py && sed -i "s/self.camera_image_topic, self._on_camera_image, qos_profile_sensor_data/self.camera_image_topic, self._on_camera_image, reliable_qos/g" /workspace/bridge-runtime.py && exec python3 /workspace/bridge-runtime.py' \
  >/dev/null

for _ in $(seq 1 30); do
  if curl -fsS --max-time 1 "http://127.0.0.1:${BRIDGE_PORT}/health" >/dev/null 2>&1; then
    echo "Independent platform bridge started."
    echo "  URL:     http://127.0.0.1:${BRIDGE_PORT}"
    echo "  Domain:  $ROS_DOMAIN"
    echo "  Profile: $PROFILE_PATH"
    exit 0
  fi
  sleep 1
done

echo "Bridge did not become ready. Recent logs:" >&2
docker logs --tail 60 "$BRIDGE_CONTAINER" >&2 || true
exit 1
