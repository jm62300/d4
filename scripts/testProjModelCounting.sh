#!/bin/bash

# $1, bench
# Exit 0 when both counters agree, non-zero when they differ (bug present).

if [ ! -f "$1" ]; then
    echo "usage: $0 bench.cnf" >&2
    echo "error: missing or unreadable CNF instance argument" >&2
    exit 2
fi

BENCH=$(mktemp /tmp/projmc.XXXXXX.cnf)
SOL1=$(mktemp /tmp/projmc.sol1.XXXXXX)
SOL2=$(mktemp /tmp/projmc.sol2.XXXXXX)
trap 'rm -f "$BENCH" "$SOL1" "$SOL2"' EXIT

cp "$1" "$BENCH"
# grep "^c p show" /tmp/2test.cnf >> $BENCH

MODEL_COUNTER="${MODEL_COUNTER:-./d4_static -m counting -i}"
TESTED_METHOD="${TESTED_METHOD:-../c++/counter/build/counter --counter.outFormat classic -i}"

$TESTED_METHOD "$BENCH" 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > "$SOL1"
$MODEL_COUNTER "$BENCH" 2>/dev/null | grep "^s " | cut -d ' ' -f2 | sed 's/ //g' > "$SOL2"

diff "$SOL2" "$SOL1" > /dev/null
exit $?
