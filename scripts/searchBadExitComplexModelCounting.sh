#!/bin/bash
# Fuzz complex-weighted model counting for correctness and crashes.
# Usage: [TESTED_METHOD="..."] searchBadExitComplexModelCounting.sh [timeout=2] [max_bugs=10]
source "$(dirname "$0")/lib.sh"
is_executable_ready
fuzz_loop "gen_complex_cnf 200" "$(dirname "$0")/testComplexModelCounter.sh" "${1:-2}" "${2:-10}"
