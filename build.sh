#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

set -e
set -u
set -o pipefail

opt=0
PARALLEL_FLAGS=""

# Note: Added 'j' to cleanly trigger parallel builds
while getopts 'dspj' OPTION
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
        j)
            # Instructs CMake to use all available CPU cores safely
            PARALLEL_FLAGS="--parallel" 
            ;;
    esac
done

CMAKE_FLAGS="-DBUILD_MODE=$opt"


cd "$SCRIPT_DIR"
mkdir -p build
cd build

export GITLAB_TOKEN_LOGICAL="${GITLAB_TOKEN_LOGICAL:-glpat-eHavaAslBimi87EABf6MIW86MQp1OjJpdgk.01.0z0dxf0ko}"

echo "c [BUILD] Configuring CMake..."
cmake .. $CMAKE_FLAGS 

echo "c [BUILD] Compiling..."
cmake --build . $PARALLEL_FLAGS

echo "c [BUILD] Build complete! A monolithic libd4.a has been created natively."

if [ -f "$SCRIPT_DIR/completion.bash" ]; then
    source "$SCRIPT_DIR/completion.bash"
    export -f _d4_completions
fi