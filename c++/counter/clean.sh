#!/bin/bash

# Exit immediately if a command fails, but allow missing files
set -e
set -u

echo "c [CLEAN] Starting cleanup process..."

# 1. Clean the local 'counter' build directory
if [ -d "build" ]; then
    echo "c [CLEAN] Removing local 'counter' build directory..."
    rm -rf build/
else
    echo "c [CLEAN] Local 'counter' build directory already clean."
fi

# 2. Clean the parent 'd4' project
D4_ROOT_DIR="../../"

echo "c [CLEAN] Navigating to parent project (../../) to clean d4 libraries..."

# Save our current location so we can safely return
CURRENT_DIR="$PWD"

cd "$D4_ROOT_DIR"

# Check if the parent clean.sh exists before trying to run it
if [ -x "clean.sh" ]; then
    ./clean.sh
    echo "c [CLEAN] Parent 'd4' project cleaned successfully."
elif [ -f "clean.sh" ]; then
    # In case it exists but isn't marked as executable
    bash clean.sh
    echo "c [CLEAN] Parent 'd4' project cleaned successfully."
else
    # Fallback just in case the parent project relies on a standard Makefile
    echo "c [WARNING] No clean.sh found in $D4_ROOT_DIR!"
    if [ -f "Makefile" ]; then
        echo "c [CLEAN] Found Makefile. Falling back to 'make clean'..."
        make clean
    fi
fi

# Return to the starting directory
cd "$CURRENT_DIR"

echo "c [CLEAN] All cleanup operations finished successfully!"