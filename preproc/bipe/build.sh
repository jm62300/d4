#!/bin/bash

set -e
set -u
set -o pipefail

BUILD_TYPE="Release"
PARALLEL_FLAG="--parallel 1" 

while getopts 'dsj' OPTION
do
    echo "c [BUILD] Option Selected for compilation: -$OPTION"
    case "$OPTION" in
        d) BUILD_TYPE="Debug" ;;
        s) BUILD_TYPE="Release" ;;        
        j) PARALLEL_FLAG="--parallel" ;;
        *) echo "Usage: $0 [-d] [-s] [-j]"; exit 1 ;;
    esac
done

# Create Build Directory
mkdir -p build
cd build

# Configure CMake
echo "c [BUILD] Configuring CMake (Type: $BUILD_TYPE)..."
cmake .. -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

# Build the completely merged library natively
echo "c [BUILD] Compiling and merging via CMake..."
cmake --build . $PARALLEL_FLAG

echo "c [BUILD] Build complete!"