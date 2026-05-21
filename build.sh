#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

set -e
set -u
set -o pipefail

opt=0
python_mode=0
PARALLEL_FLAGS=""

# Note: Added 'j' to cleanly trigger parallel builds
while getopts 'dspyj' OPTION
do
    case "$OPTION" in
        d)
            opt=1
            ;;
        s)
            opt=2
            ;;
        p)
            opt=3
            ;;
        y)
            python_mode=1
            ;;
        j)
            # Instructs CMake to use all available CPU cores safely
            PARALLEL_FLAGS="--parallel" 
            ;;
    esac
done

CMAKE_FLAGS="-DBUILD_MODE=$opt"

if [ "$python_mode" -eq 1 ]; then
    echo "=== BUILD MODE: PYTHON (Position Independent Code) ==="
    export CFLAGS="-fPIC ${CFLAGS:-}"
    export CXXFLAGS="-fPIC ${CXXFLAGS:-}"
    # Explicitly turn on the Python wrapper option from your CMakeLists.txt
    CMAKE_FLAGS="$CMAKE_FLAGS -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_PYTHON_WRAPPER=ON"
fi

cd "$SCRIPT_DIR"
mkdir -p build
cd build

export GITLAB_TOKEN="glpat-XpIZ4boV_wOSId8eNA54cm86MQp1OjJpcAk.01.0z0mopjl8"

echo "c [BUILD] Configuring CMake..."
cmake .. $CMAKE_FLAGS 

echo "c [BUILD] Compiling..."
cmake --build . $PARALLEL_FLAGS

echo "c [BUILD] Build complete! A monolithic libd4.a has been created natively."