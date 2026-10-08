#!/usr/bin/env bash
set -euo pipefail
BRIDGE_CONTAINER=${BRIDGE_CONTAINER:-future-drive-nav2-bridge}
if docker ps -a --format '{{.Names}}' | grep -Fxq "$BRIDGE_CONTAINER"; then
  docker rm -f "$BRIDGE_CONTAINER" >/dev/null
  echo "Independent platform bridge stopped: $BRIDGE_CONTAINER"
else
  echo "Independent platform bridge is not running: $BRIDGE_CONTAINER"
fi
