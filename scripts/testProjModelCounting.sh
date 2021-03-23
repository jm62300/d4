#!/bin/bash

# $1, bench
# $2, query

QUERIES=$2 # /tmp/1queries.cnf

ROOT_PATH=".."
SOLVER="$ROOT_PATH/minisat"

# echo "test " >> /tmp/file

# $SOLVER $1 > /dev/null
# if [ $? -ne 10 ]; then return 0; fi


MODEL_COUNTER="/home/lagniez/Works/d4/build/d4_debug -m counting --float 0 -i"
TESTED_METHOD="/home/lagniez/Works/d4/build/d4_debug -m projMC --float 0 -i"

$TESTED_METHOD $1  | grep "^s "  > /tmp/sol1.txt        
$MODEL_COUNTER $1  | grep "^s "  2>/dev/null > /tmp/sol2.txt

diff /tmp/sol2.txt /tmp/sol1.txt > /dev/null
exit $?
