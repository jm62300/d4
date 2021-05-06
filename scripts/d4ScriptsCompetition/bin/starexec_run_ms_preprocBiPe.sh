#!/bin/bash

# $1, path to the bench

echo "c [COMMAND LINE]: $@"

CURR_PATH=.
OPTIONS="--partitioning-heuristic decomposition-static-multi"

BENCH=$2

PBENCH=$(tempfile)
timeout 60 ./BiPe_static -useGate -preproc $BENCH > $PBENCH
ret=$?

if [ $ret -ne 124 ]; then BENCH=$PBENCH; fi

while getopts ":m:w:p" option; do
    case "${option}" in
        m)
	    shift 
	    $CURR_PATH/d4_static -m counting -i $BENCH --keyword-output-format-solution "s mc" $OPTIONS
        ;;
        w)
	    shift 
	    $CURR_PATH/d4_static -m counting -i $BENCH --keyword-output-format-solution "s wmc" --float 1 $OPTIONS
		;;
        p)
        shift 
	    $CURR_PATH/d4_static -m counting -i $BENCH --keyword-output-format-solution "s pmc" $OPTIONS
        ;;
    esac
done


rm $PBENCH
