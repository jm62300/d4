#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

$SOLVER $1 > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

MODEL_COUNTER="${MODEL_COUNTER:-/home/lagniez/Works/Softs/solvers/ganak2.4.6/ganak --mode 6}"
TESTED_METHOD="${TESTED_METHOD:-../c++/counter/build/counter --branching-heuristic classic -i}"

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
    local tol="${3:-1e-8}" 

    LC_ALL=C awk -v tol="$tol" -v f1="$file1" -v f2="$file2" '
    function abs(x) { return x < 0 ? -x : x }
    
    BEGIN {
        # CRITICAL FOR UBUNTU/MAWK: Force tol to be a number!
        # This prevents "9.4e-07" > "1e-6" alphabetical string comparison bugs.
        tol = tol + 0 
        
        # Initialize flags so we know if a file was unexpectedly empty
        file1_seen = 0; file2_seen = 0
    }

    NF == 0 { next } 

    {
        # Strip hidden \r characters just in case the parser outputs DOS line endings
        sub(/\r$/, "", $NF)

        real = $1 + 0; sign = $2; imag_str = $3
        sub(/i$/, "", imag_str); imag = imag_str + 0 
        if (sign == "-") { imag = -imag }
        
        # Explicitly check filename, avoiding the NR==FNR empty-file trap
        if (FILENAME == f1) { 
            r1 = real; i1 = imag; file1_seen = 1 
        } 
        if (FILENAME == f2) { 
            r2 = real; i2 = imag; file2_seen = 1 
        }
    }
    
    END {
        # Failsafe: Did grep actually output data to both files?
        if (!file1_seen || !file2_seen) {
            printf "Mismatch Debug -> Error: One or both files are empty!\n" > "/dev/stderr"
            exit 1
        }

        diff_r = abs(r1 - r2)
        diff_i = abs(i1 - i2)

        if (diff_r <= tol && diff_i <= tol) {
            exit 0
        } else {
            printf "Mismatch Debug -> Real diff: %e | Imag diff: %e (Tolerance: %e)\n", diff_r, diff_i, tol > "/dev/stderr"
            exit 1
        }
    }' "$file1" "$file2"
}


F1="/tmp/sol1.txt"
F2="/tmp/sol2.txt"

if compare_complex_files "$F1" "$F2" "1e-9"; then
    echo "✅ Success: The parser outputs match exactly."
    exit 0
else
    echo "❌ Error: The solutions are mathematically different."
    exit 1
fi