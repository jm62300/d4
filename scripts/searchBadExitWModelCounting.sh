#!/bin/bash
# Fuzz weighted model counting for crashes (no correctness oracle available).
# Usage: TESTED_METHOD="./my_counter -i" searchBadExitWModelCounting.sh [timeout=2] [max_bugs=10]
source "$(dirname "$0")/lib.sh"
is_executable_ready
export TESTED_METHOD="${TESTED_METHOD:-../c++/counter/build/counter -i}"
fuzz_loop "gen_weighted_cnf 50" "$(dirname "$0")/crash_check.sh" "${1:-2}" "${2:-10}"
