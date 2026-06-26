#!/bin/bash

if [ $# -eq 1 ]
then
    rm -f 3rdParty/glucose-3.0/core/*.o*
    rm -f 3rdParty/glucose-3.0/core/lib*
    rm -f 3rdParty/glucose-3.0/core/depend.mk
    rm -f 3rdParty/glucose-3.0/utils/*.o*
    
    CUR_REP="$PWD"
    cd 3rdParty/flowCutter
    make mrproper
    cd "$CUR_REP"

    echo "c [CLEAN] Removing fetched external libraries from 3rdParty..."
    rm -rf 3rdParty/*-src/
    rm -rf 3rdParty/*-build/
    rm -rf 3rdParty/*-subbuild/
fi

rm -rf build/
rm -f gmon.out
