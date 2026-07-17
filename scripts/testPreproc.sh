#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

$SOLVER $1 > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

MODEL_COUNTER="./build/counter -m counting -i"
PREPROC="./build/counter -m counting --preproc-timeout 1 -i"



$PREPROC $1 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol1.txt
$MODEL_COUNTER $1 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol2.txt

diff /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
