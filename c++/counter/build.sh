#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e
set -u
set -o pipefail

# Default configuration (Equivalent to 'make c')
BUILD_TYPE="Release"
STATIC_FLAG="OFF"
PROFILE_FLAG="OFF"

# Parse command-line options
while getopts 'cdsp' OPTION
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
            # Profiling usually benefits from debug symbols (-O2 -g)
            BUILD_TYPE="RelWithDebInfo" 
            STATIC_FLAG="OFF"
            PROFILE_FLAG="ON"
            ;;
        *)
            echo "Usage: $0 [-c (release) | -d (debug) | -s (static) | -p (profile)]"
            exit 1
            ;;
    esac
done

# Create and enter the build directory
mkdir -p build
cd build

echo "c [BUILD] Configuring CMake (Type: $BUILD_TYPE, Static: $STATIC_FLAG, Profile: $PROFILE_FLAG)..."

# Generate standard Unix Makefiles with the selected options
cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DBUILD_STATIC="$STATIC_FLAG" \
    -DBUILD_PROFILE="$PROFILE_FLAG"

echo "c [BUILD] Compiling executable using Make..."

# The modern way to invoke Make via CMake, utilizing all available CPU cores
cmake --build . --parallel 

echo "c [BUILD] Build complete! The 'counter' executable is ready in the build/ directory."