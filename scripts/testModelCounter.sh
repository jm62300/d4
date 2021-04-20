#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

echo "test " >> /tmp/file

$SOLVER $1 > /dev/null
if [ $? -ne 10 ]; then exit 0; fi



MODEL_COUNTER="/home/lagniez/Works/DeMoniaC/core/DeMoniaC -mc"
TESTED_METHOD="/home/lagniez/Works/d4/build/d4_debug -m counting -s glucose -p backbone -i"

$TESTED_METHOD $1 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol1.txt
$MODEL_COUNTER $1 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > /tmp/sol2.txt

diff /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
