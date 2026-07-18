#!/bin/bash
# Fuzz weighted model counting with integer weights in [-10, 10] (negative and
# zero allowed) for correctness and crashes; reference: ganak --mode 6.
# Usage: [TESTED_METHOD="..."] searchBadExitNegWModelCounting.sh [timeout=5] [max_bugs=10]
source "$(dirname "$0")/lib.sh"
is_executable_ready
fuzz_loop "gen_neg_weighted_cnf 200" "$(dirname "$0")/testNegWModelCounter.sh" "${1:-5}" "${2:-10}"
