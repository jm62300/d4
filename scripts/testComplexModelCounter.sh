#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

$SOLVER $1 > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

MODEL_COUNTER="/home/lagniez/Works/Softs/solvers/ganak2.4.6/ganak --mode 6"
TESTED_METHOD="../c++/counter/build/counter --branching-heuristic classic -i"
# TESTED_METHOD="../demo/counter/build/counter_debug -i"
#TESTED_METHOD="./starexec_run_ds_preprocSharpEquiv.sh"

$TESTED_METHOD $1 2>/dev/null | grep "^s " | cut -d ' ' -f2- > /tmp/sol1.txt
$MODEL_COUNTER $1 2>/dev/null | grep "^c s exact" | cut -d ' ' -f6-  > /tmp/sol2.txt

# ==============================================================================
# Function: compare_complex_files
# Usage: compare_complex_files <file1> <file2> [tolerance]
# Returns: 0 (True) if they match, 1 (False) if they mismatch
# ==============================================================================
compare_complex_files() {
    local file1="$1"
    local file2="$2"
    local tol="${3:-1e-8}" # Default to 1e-8, which easily covers your 4.7e-11 difference

    # Notice the LC_ALL=C right before awk! This forces it to understand the '.' as a decimal.
    LC_ALL=C awk -v tol="$tol" '
    function abs(x) { return x < 0 ? -x : x }
    {
        real = $1 + 0  
        sign = $2
        
        imag_str = $3
        sub(/i$/, "", imag_str)
        imag = imag_str + 0 
        
        if (sign == "-") { imag = -imag }

        if (NR == 1) { r1 = real; i1 = imag } 
        else if (NR == 2) { r2 = real; i2 = imag }
    }
    END {
        diff_r = abs(r1 - r2)
        diff_i = abs(i1 - i2)

        if (diff_r <= tol && diff_i <= tol) {
            exit 0
        } else {
            # This will print EXACTLY what awk sees, so we can debug it!
            printf "Mismatch Debug -> Real diff: %e | Imag diff: %e (Tolerance: %e)\n", diff_r, diff_i, tol > "/dev/stderr"
            exit 1
        }
    }' "$file1" "$file2"
}

# ==============================================================================
# Example Usage inside your script
# ==============================================================================

F1="/tmp/sol1.txt"
F2="/tmp/sol2.txt"

# You can call the function directly inside an if-statement!
if compare_complex_files "$F1" "$F2" "1e-6"; then
    echo "✅ Success: The parser outputs match exactly."
    exit 0
    # Do your next steps here
else
    echo "❌ Error: The solutions are mathematically different."
    exit 1
fi