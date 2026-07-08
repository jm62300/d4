#!/bin/bash

# $1, bench
# $2, query

ROOT_PATH="."
SOLVER="$ROOT_PATH/minisat"

$SOLVER $1 > /dev/null
if [ $? -ne 10 ]; then exit 0; fi

MODEL_COUNTER="${MODEL_COUNTER:-/home/lagniez/Works/Softs/solvers/ganakSolver/ganak --mode 6}"
TESTED_METHOD="${TESTED_METHOD:-../c++/counter/build/counter -i}"

$TESTED_METHOD $1 2>/dev/null | grep "^c s exact" | cut -d ' ' -f6- > /tmp/sol1.txt
$MODEL_COUNTER $1 2>/dev/null | grep "^c s exact" | cut -d ' ' -f6-  > /tmp/sol2.txt

# ==============================================================================
# Function: compare_complex_files
# Usage: compare_complex_files <file1> <file2> [rel_tol] [abs_tol]
# Match rule mirrors Python's math.isclose:
#   |a-b| <= max(rel_tol * max(|a|,|b|), abs_tol)
# rel_tol absorbs ganak's fixed ~9-significant-digit precision (its absolute
# rounding error scales with the magnitude of the count); abs_tol is a floor
# so tiny/near-zero components still get a meaningful check.
# Returns: 0 (True) if they match, 1 (False) if they mismatch
# ==============================================================================
compare_complex_files() {
    local file1="$1"
    local file2="$2"
    local rel_tol="${3:-1e-6}"
    local abs_tol="${4:-1e-9}"

    LC_ALL=C awk -v rel_tol="$rel_tol" -v abs_tol="$abs_tol" -v f1="$file1" -v f2="$file2" '
    function abs(x) { return x < 0 ? -x : x }
    function max(x, y) { return x > y ? x : y }
    function isclose(a, b,    limit) {
        limit = max(rel_tol * max(abs(a), abs(b)), abs_tol)
        return abs(a - b) <= limit
    }

    # Parse a complex-number token with all whitespace already stripped,
    # e.g. "11.003-2.482i" (d4, no separator) or "1.10e+01+-2.48e+00i"
    # (ganak, "+" separator followed by imaginarys own sign). Scientific
    # notation exponent signs are masked out first so only the real/imag
    # boundary sign is found when scanning from the right. Result is left
    # in out_real / out_imag.
    function parse_complex(tok,    s, sign_pos, i, c, real_str, imag_str) {
        s = tok
        gsub(/e\+/, "eP", s)
        gsub(/e-/, "eM", s)
        sub(/i$/, "", s)

        sign_pos = 0
        for (i = length(s); i >= 1; i--) {
            c = substr(s, i, 1)
            if (c == "+" || c == "-") { sign_pos = i; break }
        }

        if (sign_pos <= 1) {
            # No explicit imaginary sign found (should not happen for valid
            # "a+bi"/"a-bi" output); treat the whole token as real.
            real_str = s
            imag_str = "0"
        } else {
            real_str = substr(s, 1, sign_pos - 1)
            imag_str = substr(s, sign_pos)
        }
        # Drop a stray separator "+" left dangling before the imaginary sign
        # (from formats like "01+-2.48...").
        sub(/\+$/, "", real_str)

        gsub(/eP/, "e+", real_str); gsub(/eM/, "e-", real_str)
        gsub(/eP/, "e+", imag_str); gsub(/eM/, "e-", imag_str)

        out_real = real_str + 0
        out_imag = imag_str + 0
    }

    BEGIN {
        # CRITICAL FOR UBUNTU/MAWK: Force these to be numbers!
        # This prevents "9.4e-07" > "1e-6" alphabetical string comparison bugs.
        rel_tol = rel_tol + 0
        abs_tol = abs_tol + 0

        # Initialize flags so we know if a file was unexpectedly empty
        file1_seen = 0; file2_seen = 0
    }

    {
        # Strip whitespace (spaces, tabs, \r) so both the space-separated
        # ganak format and the no-space d4 competition format normalize to
        # the same shape before parsing.
        line = $0
        gsub(/[ \t\r]/, "", line)
        if (line == "") next

        parse_complex(line)

        # Explicitly check filename, avoiding the NR==FNR empty-file trap
        if (FILENAME == f1) {
            r1 = out_real; i1 = out_imag; file1_seen = 1
        }
        if (FILENAME == f2) {
            r2 = out_real; i2 = out_imag; file2_seen = 1
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

        if (isclose(r1, r2) && isclose(i1, i2)) {
            exit 0
        } else {
            printf "Mismatch Debug -> Real diff: %e (limit %e) | Imag diff: %e (limit %e)\n", \
                diff_r, max(rel_tol * max(abs(r1), abs(r2)), abs_tol), \
                diff_i, max(rel_tol * max(abs(i1), abs(i2)), abs_tol) > "/dev/stderr"
            exit 1
        }
    }' "$file1" "$file2"
}


F1="/tmp/sol1.txt"
F2="/tmp/sol2.txt"

if compare_complex_files "$F1" "$F2" "1e-6" "1e-9"; then
    echo "✅ Success: The parser outputs match exactly."
    exit 0
else
    echo "❌ Error: The solutions are mathematically different."
    exit 1
fi