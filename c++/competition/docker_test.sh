#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WORKSPACE_ROOT=$(cd "$SCRIPT_DIR/../../" && pwd)

# Exit immediately if a command exits with a non-zero status
set -e
set -u
set -o pipefail

IMAGE="registry.gitlab.com/sosy-lab/benchmarking/competition-scripts/user:latest"

echo "=== Pulling official competition Docker image: $IMAGE ==="
docker pull "$IMAGE"

echo "=== Running Clean Script inside the Docker container ==="
docker run --rm -v "$WORKSPACE_ROOT:/tool" -w /tool "$IMAGE" ./c++/competition/clean.sh

echo "=== Running Build Script (Regular Build) inside the Docker container ==="
docker run --rm -v "$WORKSPACE_ROOT:/tool" -w /tool "$IMAGE" ./c++/competition/build.sh -j

echo "=== Verifying Solver Execution (Regular Build) inside the Docker container ==="
docker run --rm -v "$WORKSPACE_ROOT:/tool" -w /tool "$IMAGE" ./c++/competition/run.sh instancesTest/cnfs/cnf1.cnf

echo "=== Running Clean Script inside the Docker container ==="
docker run --rm -v "$WORKSPACE_ROOT:/tool" -w /tool "$IMAGE" ./c++/competition/clean.sh

echo "=== Running Build Script (Static Build) inside the Docker container ==="
docker run --rm -v "$WORKSPACE_ROOT:/tool" -w /tool "$IMAGE" ./c++/competition/build.sh -s -j

echo "=== Checking Binary Type on Host ==="
file "$WORKSPACE_ROOT/c++/competition/build/d4"

echo "=== Verifying Solver Execution (Static Build) inside the Docker container ==="
docker run --rm -v "$WORKSPACE_ROOT:/tool" -w /tool "$IMAGE" ./c++/competition/run.sh instancesTest/cnfs/cnf1.cnf

echo "=== SUCCESS: Solver built and executed successfully in the Docker container! ==="
