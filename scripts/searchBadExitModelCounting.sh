#!/bin/bash

# $1 the command
# $2 timeout

ROOT_PATH="."
CNF_GENERATOR="$ROOT_PATH/cnfuzz"
SOLVER="$ROOT_PATH/minisat"

TIMEOUT=$2
if [ "$TIMEOUT" == "" ]; then TIMEOUT=1; fi

LIMIT_SIZE=50

isExecutableReady()
{
    if ! [ -f $CNF_GENERATOR ]; then make -C $ROOT_PATH; fi
    if ! [ -f $SOLVER ]; then make -C $ROOT_PATH; fi       
}


generateSatisfiableCNF()
{
    ret=20
    while [ $ret -ne 10 ]
    do
        $CNF_GENERATOR | grep -v "^c " > /tmp/test.cnf

        nbVar=$(grep "p cnf" /tmp/test.cnf | cut -d ' ' -f3)
        if [ $nbVar -gt $LIMIT_SIZE ]; then continue; fi
        
        $SOLVER /tmp/test.cnf > /dev/null
        ret=$?
    done


    rdm=$(($RANDOM % 30))
    nbVar=$(grep "p cnf" /tmp/test.cnf | cut -d ' ' -f3)
    tab=$(seq 1 $nbVar)
    

    cpt=3
    while [ $rdm -lt 30 ]
    do
        rdm=$((rdm + 10))
        tab=$(echo $tab | tr ' ' '\n' | shuf | tr '\n' ' ')        
        xor=""
                
        for l in $(echo $tab | cut -d ' ' -f1-$cpt)
        do
            sign=""
            if [ $((RANDOM % 2)) -eq 0 ]; then sign="-"; fi

            xor="$xor $sign$l"
        done

        nbClause=$(((2 ** cpt) - 1))
        for i in $(seq 0 $nbClause)
        do
            tmp=$i
            parity=0
            while [ $tmp -ne 0 ]
            do
                if [ $((tmp % 2)) -eq 1 ]; then parity=$((parity + 1)); fi
                tmp=$((tmp / 2))
            done

            if [ $((parity % 2)) -ne 0 ]; then continue; fi

            tmp=$i
            for l in $xor
            do
                if [ $((tmp % 2)) -eq 1 ]
                then
                    echo -n "$((l * -1)) " >> /tmp/test.cnf
                else
                    echo -n "$l " >> /tmp/test.cnf
                fi                
                tmp=$((tmp / 2))
            done
            echo "0" >> /tmp/test.cnf
        done
        cpt=$((cpt + 1))

    done

    echo "/tmp/test.cnf"
}

# prepare the executable
isExecutableReady

# the main
nbInst=1
cpt=0

while [ true ]
do
    printf "number of instances tested %d\r" "$cpt" 

    benchName=$(generateSatisfiableCNF)
    timeout $TIMEOUT $1 $benchName > /dev/null 2>/dev/null

    code=$?

    if [ $code -ne 124 ]
    then
        cpt=`expr $cpt + 1` 
    fi

    if [ $code -ne 124 -a $code -ne 0 ]
    then
        echo "/tmp/${nbInst}test.cnf" `cat /tmp/test.cnf | grep "^p cnf" | cut -d ' ' -f3-`
        cp $benchName /tmp/${nbInst}test.cnf
        nbInst=`expr $nbInst + 1`        
        if [ $nbInst -gt 10 ]
        then
            echo "OK, there is a problem"
            exit 0
        fi
    fi
done




