#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e
set -u
set -o pipefail

# ==============================================================================
# Parse command-line options FIRST (so we can pass -j to dependencies)
# ==============================================================================
BUILD_TYPE="Release"
STATIC_FLAG="OFF"
PROFILE_FLAG="OFF"
PARALLEL_FLAG=""

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

# ==============================================================================
# Check and build dependencies (d4 and bipe)
# ==============================================================================

# Check d4
D4_ROOT_DIR="../../"
if [ ! -f "$D4_ROOT_DIR/build/libd4.a" ] && [ ! -f "$D4_ROOT_DIR/build/libd4.so" ]; then
    echo "c [BUILD] Parent library d4 is missing! Building d4 first..."
    
    CURRENT_DIR="$PWD"
    cd "$D4_ROOT_DIR"
    
    # Pass ALL arguments (e.g., -s -j) down to the parent script
    ./build.sh "$@"
    
    cd "$CURRENT_DIR"
    echo "c [BUILD] d4 built successfully."
fi

# Check bipe
BIPE_ROOT_DIR="../../preproc/bipe"
if [ ! -f "$BIPE_ROOT_DIR/build/libbipe.a" ] && [ ! -f "$BIPE_ROOT_DIR/build/libbipe.so" ]; then
    echo "c [BUILD] bipe library is missing! Building bipe first..."
    
    CURRENT_DIR="$PWD"
    mkdir -p "$BIPE_ROOT_DIR/build"
    cd "$BIPE_ROOT_DIR/build"
    
    # Configure and build bipe using the same build type and parallel flag
    cmake .. -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    
    # Unquoted $PARALLEL_FLAG so it expands to nothing if empty
    cmake --build . $PARALLEL_FLAG
    
    cd "$CURRENT_DIR"
    echo "c [BUILD] bipe built successfully."
fi

# ==============================================================================
# Configure and build 'counter'
# ==============================================================================

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