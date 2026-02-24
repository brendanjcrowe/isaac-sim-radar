#!/bin/bash
set -e

if [ $# -gt 0 ]; then
    exec "$@"
fi

exec ./runheadless.native.sh -v
