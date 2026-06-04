#!/bin/bash
# Minimal oracle: verify that $TESTED_METHOD exits cleanly on $1.
# Use this when no correctness oracle exists (crash detection only).
# Set TESTED_METHOD in the environment before calling, e.g.:
#   TESTED_METHOD="../c++/counter/build/counter -i" bash crash_check.sh bench.cnf
${TESTED_METHOD:?'TESTED_METHOD must be set'} "$1" > /dev/null 2>/dev/null
