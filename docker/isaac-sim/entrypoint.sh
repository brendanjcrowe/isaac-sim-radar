#!/bin/bash
set -e

if [ $# -gt 0 ]; then
    exec "$@"
fi

exec /isaac-sim/python.sh /app/isaac_sim_scripts/run_headless.py
