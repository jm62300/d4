#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

$SOLVER $1 > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

MODEL_COUNTER="../demo/counter/build/counter_debug --branching-heuristic classic --cache-clause-representation clause -i"
TESTED_METHOD="../demo/counter/build/counter_debug --branching-heuristic classic --cache-clause-representation sym -i"
# TESTED_METHOD="../demo/counter/build/counter_debug -i"
#TESTED_METHOD="./starexec_run_ds_preprocSharpEquiv.sh"

v1=$($TESTED_METHOD $1 2>/dev/null | grep "\-\-\->" | cut -d ' ' -f2)
v2=$($MODEL_COUNTER $1 2>/dev/null | grep "\-\-\->" | cut -d ' ' -f2)

if [ "$v1" == "" ]; then exit 0; fi
if [ "$v2" == "" ]; then exit 0; fi


if [ $v1 -gt $v2 ]; then exit 1; else exit 0; fi
