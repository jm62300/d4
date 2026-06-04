#!/bin/bash
set -e
set -u
set -o pipefail

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ROOT_DIR=$(cd "$SCRIPT_DIR/.." &> /dev/null && pwd)

# 1. Build d4 library if not already present
if [ ! -f "$ROOT_DIR/build/libd4.a" ]; then
    echo "c [TEST] libd4.a not found under build/! Building it first..."
    "$ROOT_DIR/build.sh"
fi

# 2. Configure and build test_api
mkdir -p "$SCRIPT_DIR/build"
cd "$SCRIPT_DIR/build"

echo "c [TEST] Configuring CMake for tests..."
export GITLAB_TOKEN_LOGICAL="${GITLAB_TOKEN_LOGICAL:-glpat-eHavaAslBimi87EABf6MIW86MQp1OjJpdgk.01.0z0dxf0ko}"
cmake ..

echo "c [TEST] Compiling test_api..."
cmake --build . --parallel

# 3. Run the test from the repository root
echo "c [TEST] Running test_api..."
cd "$ROOT_DIR"
./test/build/test_api
