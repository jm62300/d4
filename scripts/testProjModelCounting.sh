#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH=".."
SOLVER="$ROOT_PATH/minisat"

BENCH="/tmp/temp.cnf"
cp $1 $BENCH
# grep "^c p show" /tmp/2test.cnf >> $BENCH


MODEL_COUNTER="./d4_static -m counting -i"
TESTED_METHOD="../demo/compiler/build/compiler_debug --occurrence-manager dynamicBlockedSimp -i"

$TESTED_METHOD $BENCH 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol1.txt
$MODEL_COUNTER $BENCH 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol2.txt

diff /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
