#!/usr/bin/env bash
set -e
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJECT_ROOT"
PYTHON=python3
if [[ -x app/.venv/bin/python ]] && app/.venv/bin/python -c \
  'import fastapi, uvicorn, cv2, numpy, torch, ultralytics' >/dev/null 2>&1; then
  PYTHON=app/.venv/bin/python
fi
exec "$PYTHON" -m app.app
