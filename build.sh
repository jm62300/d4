#!/bin/bash

set -e
set -u
set -o pipefail

opt=0

while getopts 'd' OPTION
do
    case "$OPTION" in
        d)
            echo "debugging mode"
            opt=1
            ;;
    esac
done


mkdir -p build
cd build
cmake -GNinja $* .. -DDEBUG_MODE=$opt
ninja 
