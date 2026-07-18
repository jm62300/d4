#!/bin/bash
# Fuzz projected model counting for correctness and crashes.
# Usage: [TESTED_METHOD="..."] searchBadExitProjected.sh [timeout=3] [max_bugs=10]
source "$(dirname "$0")/lib.sh"
is_executable_ready
fuzz_loop "gen_projected_cnf 500" "$(dirname "$0")/testProjModelCounting.sh" "${1:-3}" "${2:-10}"