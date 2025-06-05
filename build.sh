#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

set -e
set -u
set -o pipefail

opt=0

while getopts 'dsp' OPTION
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
    esac
done

cd $SCRIPT_DIR/3rdParty/flowCutter
make -j DEBUG=$opt

cd $SCRIPT_DIR/3rdParty/glucose-3.0/core/
make libst       
mv lib_static.a lib_glucose.a

cd $SCRIPT_DIR/3rdParty/bipe/
./build.sh -s    

cd $SCRIPT_DIR
mkdir -p build
cd build
cmake .. -DBUILD_MODE=$opt 
make -j

# make a library of everything
mv libd4.a libd4tmp.a
ar cqT libd4.a libd4tmp.a ../3rdParty/flowCutter/libflowCutter.a ../3rdParty/patoh/libpatoh.a ../3rdParty/glucose-3.0/core/lib_glucose.a ../3rdParty/bipe/build/libbipe.a && echo -e 'create libd4.a\naddlib libd4.a\nsave\nend' | ar -M
