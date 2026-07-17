#!/bin/bash
# Fuzz model counting with queries for correctness and crashes.
# Usage: [TESTED_METHOD="..."] searchBadExitModelCountingQueries.sh [timeout=2] [max_bugs=10]
# Query file is passed as $2 to the oracle (default: /tmp/fuzz_queries.txt).
source "$(dirname "$0")/lib.sh"
is_executable_ready
fuzz_loop "gen_cnf_with_queries 50 20" "$(dirname "$0")/testModelCountingQuery.sh" \
    "${1:-2}" "${2:-10}" /tmp/fuzz_queries.txt
