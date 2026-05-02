#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e
set -u
set -o pipefail

# 1. Check if the parent D4 library exists
D4_ROOT_DIR="../../"
D4_LIB_FILE="../../build/libd4.a"

if [ ! -f "$D4_LIB_FILE" ]; then
    echo "c [BUILD] Parent library libd4.a is missing! Building d4 first..."
    
    # Save our current location and move to the d4 root
    CURRENT_DIR="$PWD"
    cd "$D4_ROOT_DIR"
    
    # Pass ALL arguments (e.g., -s -j) down to the parent script
    ./build.sh "$@"
    
    cd "$CURRENT_DIR"
    echo "c [BUILD] d4 built successfully. Resuming counter configuration..."
fi

# 2. Default configuration for counter
BUILD_TYPE="Release"
STATIC_FLAG="OFF"
PROFILE_FLAG="OFF"
PARALLEL_FLAG=""

# Parse command-line options
while getopts 'cdspj' OPTION
do
    echo "c [BUILD] Option Selected: -$OPTION"
    case "$OPTION" in
        c)
            BUILD_TYPE="Release"
            STATIC_FLAG="OFF"
            PROFILE_FLAG="OFF"
            ;;
        d)
            BUILD_TYPE="Debug"
            STATIC_FLAG="OFF"
            PROFILE_FLAG="OFF"
            ;;
        s)
            BUILD_TYPE="Release"
            STATIC_FLAG="ON"
            PROFILE_FLAG="OFF"
            ;;        
        p)
            BUILD_TYPE="RelWithDebInfo" 
            STATIC_FLAG="OFF"
            PROFILE_FLAG="ON"
            ;;
        j)
            # Enable parallel compilation
            PARALLEL_FLAG="--parallel"
            ;;
        *)
            echo "Usage: $0 [-c (release) | -d (debug) | -s (static) | -p (profile) | -j (parallel)]"
            exit 1
            ;;
    esac
done

# Create and enter the build directory
mkdir -p build
cd build

echo "c [BUILD] Configuring CMake (Type: $BUILD_TYPE, Static: $STATIC_FLAG, Profile: $PROFILE_FLAG)..."

cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DBUILD_STATIC="$STATIC_FLAG" \
    -DBUILD_PROFILE="$PROFILE_FLAG"

echo "c [BUILD] Compiling executable using Make..."

# Unquoted $PARALLEL_FLAG so it expands to nothing if empty, or --parallel if set
cmake --build . $PARALLEL_FLAG

echo "c [BUILD] Build complete! The 'counter' executable is ready."