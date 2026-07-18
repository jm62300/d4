#!/bin/bash
# Correctness oracle for weighted model counting with negative weights.
# Reference: ganak --mode 6 (exact arbitrary-precision weighted counting) —
# unlike d4_static it shares no code with d4, and its quadruple-precision
# output pitfalls (cancellation with negative weights) do not apply.
# Values are compared numerically with a relative tolerance of 1e-6.
#
# $1, bench

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"
GANAK="${GANAK:-../tools/fuzz/ganak --mode 6}"

$SOLVER $1 > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

TESTED_METHOD="${TESTED_METHOD:-../c++/counter/build/counter --counter.outFormat classic -i}"

# ganak: "c s exact arb cpx -2.00000000e+00 + 0.00000000e+00i" — field 6 is the
# real part (the imaginary part is always 0 for real weights).
ref=$($GANAK $1 2>/dev/null | grep "^c s exact arb cpx" | cut -d ' ' -f6)
val=$($TESTED_METHOD $1 2>/dev/null | grep "^s " | cut -d ' ' -f2)

[ -z "$ref" ] && exit 0     # reference inconclusive: not a bug
[ -z "$val" ] && exit 1     # tested method produced no count

python3 - "$ref" "$val" <<'EOF'
import sys
a, b = float(sys.argv[1]), float(sys.argv[2])
sys.exit(0 if abs(a - b) <= 1e-6 * max(abs(a), abs(b), 1e-300) else 1)
EOF
exit $?
