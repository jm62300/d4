#!/bin/bash

set -e

rm -rf cm*
rm -rf CM*
rm -rf lib*
rm -rf Testing*
rm -rf tests*
rm -rf include
rm -rf tests
rm -rf Make*
rm -rf test
rm -rf sbva
CXX=clang++ cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DENABLE_TESTING=ON -DSANITIZE=ON ..
make -j$(nproc)
make test
