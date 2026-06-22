#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

if [ ! -f "$SCRIPT_DIR/build/d4" ]; then
    echo "Error: competition executable not found. Please run ./build.sh first." >&2
    exit 1
fi

exec "$SCRIPT_DIR/build/d4" "$1"
