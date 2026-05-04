#!/bin/bash

# $1 the command
# $2 the timeout

ROOT_PATH="."
CNF_GENERATOR="$ROOT_PATH/cnfuzz"
SOLVER="$ROOT_PATH/minisat"

LIMIT_SIZE=50
TIMEOUT=2

isExecutableReady()
{
    if ! [ -f $CNF_GENERATOR ]; then make -C $ROOT_PATH; fi
    if ! [ -f $SOLVER ]; then make -C $ROOT_PATH; fi       
}


generateSatisfiableCNF()
{
    ret=20
    nbVar=0
    while [ $ret -ne 10 ]
    do
        echo "c t wmc" > /tmp/test.cnf
        $CNF_GENERATOR | grep -v "c max " >> /tmp/test.cnf

        nbVar=$(grep "p cnf" /tmp/test.cnf | cut -d ' ' -f3)
        if [ $nbVar -gt $LIMIT_SIZE ]; then continue; fi
        
        $SOLVER /tmp/test.cnf > /dev/null
        ret=$?
    done


    # add weight
    for i in $(seq 1 $nbVar)
    do
        pw1=$((RANDOM % 100))
        pw2=$((RANDOM % 100))
        nw1=$((RANDOM % 100))
        nw2=$((RANDOM % 100))
               
        if [ $pw1 -lt 10 ]; then pw1="0$pw1"; fi
        if [ $pw2 -lt 10 ]; then pw2="0$pw2"; fi
        if [ $nw1 -lt 10 ]; then nw1="0$nw1"; fi
        if [ $nw2 -lt 10 ]; then nw2="0$nw2"; fi

        echo "c p weight $i 0.$pw1 0.$pw2 0" >> /tmp/test.cnf
        echo "c p weight -$i 0.$nw1 0.$nw2 0" >> /tmp/test.cnf
    done

    echo "/tmp/test.cnf"
}

# prepare the executable files
isExecutableReady

nbInst=1
cpt=0

while [ true ]
do
    printf "number of instances tested %d\r" "$cpt" 

    benchName=$(generateSatisfiableCNF)    
    timeout $TIMEOUT $1 $benchName > /dev/null 2>/dev/null
    code=$?
    
    if [ $code -ne 124 ]; then cpt=`expr $cpt + 1`; fi
    
    if [ $code -ne 124 -a $code -ne 0 ]
    then               
        echo "/tmp/${nbInst}test.cnf" `cat /tmp/test.cnf | grep "^p cnf" | cut -d ' ' -f3-`
        cp /tmp/test.cnf /tmp/${nbInst}test.cnf
        nbInst=`expr $nbInst + 1`

        # echo "error"
        # exit 0
        if [ $nbInst -gt 10 ]
        then
            echo "OK, there is a problem"
            exit 0
        fi
    fi
done
