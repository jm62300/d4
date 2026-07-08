#!/bin/bash
# Fuzz complex-weighted model counting for crashes.
# Correctness can then be checked manually: bash testComplexModelCounter.sh /tmp/fail_N.cnf
# Usage: TESTED_METHOD="..." searchBadExitComplexModelCounting.sh [timeout=2] [max_bugs=10]
source "$(dirname "$0")/lib.sh"
is_executable_ready
export TESTED_METHOD="${TESTED_METHOD:-../c++/counter/build/counter -i}"
fuzz_loop "gen_complex_cnf 200" "$(dirname "$0")/testComplexModelCounter.sh" "${1:-2}" "${2:-10}"
