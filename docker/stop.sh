#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Isaac Sim Radar - Docker Stop ==="

if [ "$1" = "--clean" ]; then
    echo "Stopping containers and removing named volumes (Isaac Sim caches)..."
    docker compose down -v
else
    docker compose down
fi

# Revoke X11 access
if command -v xhost &>/dev/null; then
    xhost -local:docker 2>/dev/null || true
fi

echo "Done."
