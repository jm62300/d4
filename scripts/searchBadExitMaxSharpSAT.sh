#!/bin/bash
# Fuzz Max#SAT for correctness and crashes.
# Usage: [TESTED_METHOD="..."] searchBadExitMaxSharpSAT.sh [timeout=1] [max_bugs=10]
source "$(dirname "$0")/lib.sh"
is_executable_ready
fuzz_loop "gen_maxsharpsat_cnf 30" "$(dirname "$0")/testMaxSharpSAT.sh" "${1:-1}" "${2:-10}"
