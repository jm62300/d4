#!/bin/bash

set -e
set -u
set -o pipefail

opt=0

while getopts 'dsl' OPTION
do
    case "$OPTION" in
        d)
            opt=1
            ;;
        s)
            opt=2
            ;;
        l)
            opt=3
            ;;

    esac
done

curRep=$PWD
if [ $opt -eq 1 ]
then
   if ! [ -f 3rdParty/glucose-3.0/core/lib_glucose.a ]
   then
       cd 3rdParty/glucose-3.0/core/
       make libd       
       mv lib_debug.a lib_glucose.a
   fi
elif [ $opt -eq 2 ]
then
    if ! [ -f 3rdParty/glucose-3.0/core/lib_glucose.a ]
    then
        cd 3rdParty/glucose-3.0/core/
        make libst       
        mv lib_static.a lib_glucose.a
    fi
else
    if ! [ -f 3rdParty/glucose-3.0/core/lib_glucose.a ]
    then
        cd 3rdParty/glucose-3.0/core/
        make libs
        mv lib_standard.a lib_glucose.a
    fi
fi


if ! [ -f 3rdParty/kahypar/build/lib/libkahypar.a ]
then
    cd $curRep
    cd 3rdParty/kahypar/
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE
    make -j
fi

cd $curRep
cd 3rdParty/reducer/
./build.sh -l    

cd $curRep
cd 3rdParty/bipe/
./build.sh -l    

cd $curRep
cd 3rdParty/eliminator
./build.sh -l    

cd $curRep
mkdir -p build
cd build
cmake -GNinja .. -DBUILD_MODE=$opt 

if [ $opt -eq 3 ]; then rm -f libd4.a; fi
ninja 


if [ $opt -eq 3 ]
then        
    mkdir -p kahypar
    cd kahypar
    ar -x ../../3rdParty/kahypar/build/lib/libkahypar.a
    cd ..

    mkdir -p patoh
    cd patoh
    ar -x ../../3rdParty/patoh/libpatoh.a
    cd ..

    mkdir -p glucose 
    cd glucose
    ar -x ../../3rdParty/glucose-3.0/core/lib_glucose.a
    cd ..

    mkdir -p reducer
    cd reducer    
    ar -x ../../3rdParty/reducer/build/libreducer.a
    cd ..

    mkdir -p bipe
    cd bipe
    ar -x ../../3rdParty/bipe/build/libbipe.a
    cd ..

    mkdir -p eliminator
    cd eliminator
    ar -x ../../3rdParty/eliminator/build/libeliminator.a
    cd ..

    ar -x libd4.a    
    ar -rc libd4.a *.o kahypar/* patoh/* glucose/* reducer/* bipe/* eliminator/*
fi