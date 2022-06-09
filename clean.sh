#!/bin/bash

if [ $# -eq 1 ]
then
    rm -f 3rdParty/glucose-3.0/core/*.o*
    rm -f 3rdParty/glucose-3.0/core/lib*
    rm -f 3rdParty/glucose-3.0/core/depend.mk
    rm -f 3rdParty/glucose-3.0/utils/*.o*
    rm -rf 3rdParty/kahypar/build
    rm -rf 3rdParty/reducer/build

    curRep=$PWD
    cd 3rdParty/bipe/
    ./clean.sh 1
    cd $curRep

    cd 3rdParty/reducer
    ./clean.sh 1
    cd $curRep

    cd 3rdParty/eliminator
    ./clean.sh 1
    cd $curRep
fi

rm -rf build/
rm -f gmon.out
