#!/usr/bin/env bash
set -euo pipefail

rm -rf CMake*
rm -rf src
rm -rf cmake*
rm -rf Make*
rm -rf sbva*
rm -rf .cmake
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
make -j$(nproc)
