#!/bin/bash
set -e
set -u

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo "c [CLEAN] Cleaning competition build directory..."
rm -rf "$SCRIPT_DIR/build"

echo "c [CLEAN] Cleaning parent D4 build directory..."
if [ -f "$SCRIPT_DIR/../../clean.sh" ]; then
    "$SCRIPT_DIR/../../clean.sh"
fi

echo "c [CLEAN] Cleaning FetchContent build and subbuild caches..."
rm -rf "$SCRIPT_DIR/../../3rdParty"/*-build/
rm -rf "$SCRIPT_DIR/../../3rdParty"/*-subbuild/
