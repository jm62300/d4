#!/bin/bash

set -e
set -u
set -o pipefail

opt=0

while getopts 'ds' OPTION
do
    case "$OPTION" in
        d)
            opt=1
            ;;
        s)
            opt=2
            ;;

    esac
done


mkdir -p build
cd build
cmake -GNinja $* .. -DBUILD_MODE=$opt
ninja 
