#!/bin/bash
# Generic crash fuzzer: tests any tool for crashes on plain CNF.
# Usage: searchBadExitQuick.sh <tool_cmd> [max_vars=100] [timeout=2] [max_bugs=10]
# Example: searchBadExitQuick.sh "../c++/counter/build/counter -i" 50 3
source "$(dirname "$0")/lib.sh"
is_executable_ready
export TESTED_METHOD="${1:?Usage: $0 <tool_cmd> [max_vars] [timeout] [max_bugs]}"
fuzz_loop "gen_plain_cnf ${2:-100}" "$(dirname "$0")/crash_check.sh" "${3:-2}" "${4:-10}"
