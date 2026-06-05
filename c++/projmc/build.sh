#!/bin/bash

set -e
set -u
set -o pipefail

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
            PARALLEL_FLAG="--parallel"
            ;;
        *)
            echo "Usage: $0 [-c (release) | -d (debug) | -s (static) | -p (profile) | -j (parallel)]"
            exit 1
            ;;
    esac
done

D4_ROOT_DIR="../../"
echo "c [BUILD] Building parent library d4 first..."

CURRENT_DIR="$PWD"
cd "$D4_ROOT_DIR"

export GITLAB_TOKEN_LOGICAL="${GITLAB_TOKEN_LOGICAL:-glpat-eHavaAslBimi87EABf6MIW86MQp1OjJpdgk.01.0z0dxf0ko}"

./build.sh "$@"

cd "$CURRENT_DIR"
echo "c [BUILD] d4 built successfully."

mkdir -p build
cd build

echo "c [BUILD] Configuring CMake (Type: $BUILD_TYPE, Static: $STATIC_FLAG, Profile: $PROFILE_FLAG)..."

cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DBUILD_STATIC="$STATIC_FLAG" \
    -DBUILD_PROFILE="$PROFILE_FLAG"

echo "c [BUILD] Compiling projmc..."

cmake --build . $PARALLEL_FLAG

echo "c [BUILD] Build complete! The 'projmc' executable is ready."
