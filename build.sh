#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
set -e
set -u
set -o pipefail
opt=0

python_mode=0

NB_CORE=""

while getopts 'dspyc' OPTION
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
        c)
            NB_CORE=1
    esac
done

CMAKE_PIC_FLAG=""
if [ "$python_mode" -eq 1 ]; then
    echo "=== BUILD MODE: PYTHON (Position Independent Code) ==="
    export CFLAGS="-fPIC ${CFLAGS:-}"
    export CXXFLAGS="-fPIC ${CXXFLAGS:-}"
    export CMAKE_PIC_FLAG="-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
fi

cd "$SCRIPT_DIR"
mkdir -p build
cd build
cmake .. -DBUILD_MODE=$opt $CMAKE_PIC_FLAG
make -j $NB_CORE

# make a library of everything
mv libd4.a libd4tmp.a

ar cqT libd4.a libd4tmp.a 3rdParty/flowCutter/libflowCutter.a 3rdParty/glucose-3.0/libglucose.a 3rdParty/bipe/libbipe.a 3rdParty/bipe/libglucose_bipe.a && echo -e 'create libd4.a\naddlib libd4.a\nsave\nend' | ar -M

rm libd4tmp.a
