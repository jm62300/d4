#!/bin/bash

set -e
set -u
set -o pipefail

BUILD_TYPE="Release"
BUILD_STATIC="OFF"
PARALLEL_FLAG="--parallel 1" 

# We will collect the flags to pass them up to the root build script
ROOT_ARGS=""

while getopts 'dsj' OPTION
do
    echo "c [APP BUILD] Option Selected: -$OPTION"
    case "$OPTION" in
        d) 
            BUILD_TYPE="Debug" 
            ROOT_ARGS="$ROOT_ARGS -d"
            ;;
        s) 
            BUILD_STATIC="ON" 
            ROOT_ARGS="$ROOT_ARGS -s"
            ;;        
        j) 
            PARALLEL_FLAG="--parallel" 
            ROOT_ARGS="$ROOT_ARGS -j"
            ;;
        *) 
            echo "Usage: $0 [-d] [-s] [-j]"
            exit 1 
            ;;
    esac
done

# ==============================================================================
# Build the Root Library First
# ==============================================================================
echo "c [APP BUILD] Step 0: Compiling core bipe library in parent directory..."

# We use a subshell (the parentheses) so we can 'cd' into the parent directory, 
# run its build script, and immediately snap back without messing up our paths here.
(cd .. && ./build.sh $ROOT_ARGS)

echo "c [APP BUILD] Core library successfully built. Proceeding with application..."

# ==============================================================================
# Create Build Directory
# ==============================================================================
mkdir -p build
cd build

# ==============================================================================
# Configure CMake
# ==============================================================================
echo "c [APP BUILD] Configuring App CMake (Type: $BUILD_TYPE, Static: $BUILD_STATIC)..."
cmake .. -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DBUILD_STATIC="$BUILD_STATIC"

# ==============================================================================
# Compile
# ==============================================================================
echo "c [APP BUILD] Compiling App..."
cmake --build . $PARALLEL_FLAG

echo "c [APP BUILD] Build complete! Your executable is ready at: c++/build/bipe"