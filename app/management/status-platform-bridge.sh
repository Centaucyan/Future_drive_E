#!/usr/bin/env bash
set -euo pipefail
BRIDGE_PORT=${BRIDGE_PORT:-8765}
BRIDGE_CONTAINER=${BRIDGE_CONTAINER:-future-drive-nav2-bridge}
docker ps -a --filter "name=^/${BRIDGE_CONTAINER}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
if curl -fsS --max-time 2 "http://127.0.0.1:${BRIDGE_PORT}/health"; then
  echo
else
  echo "Bridge health endpoint is not ready on port $BRIDGE_PORT." >&2
  exit 1
fi
