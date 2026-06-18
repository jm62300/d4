#!/bin/bash

set -e
set -u

echo "c [CLEAN] Starting cleanup..."

if [ -d "build" ]; then
    echo "c [CLEAN] Removing local projmc build directory..."
    rm -rf build/
else
    echo "c [CLEAN] Local build directory already clean."
fi

D4_ROOT_DIR="../../"
CURRENT_DIR="$PWD"

echo "c [CLEAN] Cleaning parent d4 project..."
cd "$D4_ROOT_DIR"

if [ -x "clean.sh" ]; then
    ./clean.sh "$@"
elif [ -f "clean.sh" ]; then
    bash clean.sh "$@"
fi

cd "$CURRENT_DIR"
echo "c [CLEAN] Done."
