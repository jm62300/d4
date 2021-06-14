#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

$SOLVER $1 > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

MODEL_COUNTER="../build/d4_debug -m counting -i"
TESTED_METHOD="../build/d4_debug -m max#sat -i"


# get the max variables.
maxVar=$(grep "c max" $1 | cut -d ' ' -f3- | awk 'NF{NF-=1};1')

# get the ind variables.
indVar=$(grep "c ind" $1 | cut -d ' ' -f3-)

# generate used projected formula
fileTmp=$(tempfile)
grep -v "^c " $1 > $fileTmp
echo "c p show $indVar" >> $fileTmp

nbByte=$(echo $maxVar | wc -w)
nbByte=$((nbByte - 1))

a=($maxVar)
counter=()
for i in "${a[@]}"; do counter+=(0); done

# run counter.
max=0
while [ ${#counter[@]} -le ${#a[@]} ]
do
    # add one
    counter[0]=$((counter[0] + 1))    

    # propagate
    pos=0
    while [ ${counter[$pos]} -eq 2 ]
    do
        counter[$pos]=0
        pos=$((pos+1))
        
        if [ $pos -le ${#counter[@]} ]
        then
            counter[$pos]=$((counter[$pos] + 1))
        fi
    done

    # get the interpretation and call the projected counter.
    fileTmpCouter=$(tempfile)
    cp $fileTmp $fileTmpCouter

    if [ ${#counter[@]} -le ${#a[@]} ]
    then
        for i in $(seq 0 $nbByte)
        do
            if [ ${counter[$i]} -eq 0 ]
            then
                echo "-${a[$i]} 0" >> $fileTmpCouter
            else   
                echo "${a[$i]} 0" >> $fileTmpCouter
            fi
        done
    fi

    c=$($MODEL_COUNTER $fileTmpCouter 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g')    
    if [ $c -gt $max ]; then max=$c; fi
    rm $fileTmpCouter    
done
rm $fileTmp

echo $max # > /tmp/sol1.txt
$TESTED_METHOD $1 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' # > /tmp/sol2.txt

# rm $fileTmp
diff /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
