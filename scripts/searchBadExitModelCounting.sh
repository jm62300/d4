#!/bin/bash
# Fuzz plain model counting for correctness and crashes.
# Usage: [TESTED_METHOD="..."] searchBadExitModelCounting.sh [timeout=2] [max_bugs=10]
source "$(dirname "$0")/lib.sh"
is_executable_ready
fuzz_loop "gen_plain_cnf 100" "$(dirname "$0")/testModelCounter.sh" "${1:-2}" "${2:-10}"
