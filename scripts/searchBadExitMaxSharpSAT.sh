#!/bin/bash
# Fuzz Max#SAT for correctness and crashes (oracle runs 3 d4 calls, use generous timeout).
# Usage: [TESTED_METHOD="..."] searchBadExitMaxSharpSAT.sh [timeout=5] [max_bugs=10]
source "$(dirname "$0")/lib.sh"
is_executable_ready
fuzz_loop "gen_maxsharpsat_cnf 30" "$(dirname "$0")/testMaxSharpSAT.sh" "${1:-5}" "${2:-10}"
