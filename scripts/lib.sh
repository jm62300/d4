#!/bin/bash
# Shared fuzzing library.  Source this file from any searchBadExit*.sh script.
#
# Public API:
#   is_executable_ready
#   gen_plain_cnf        [max_vars=100]
#   gen_projected_cnf    [max_vars=50]
#   gen_weighted_cnf     [max_vars=20]      -- real-valued weights
#   gen_complex_cnf      [max_vars=200]     -- complex-valued weights
#   gen_maxsharpsat_cnf  [max_vars=30]      -- c max / c ind partition + real weights
#   gen_cnf_with_queries [max_vars=50] [n_queries=20]  -- also writes /tmp/fuzz_queries.txt
#   fuzz_loop <generator_cmd> <oracle_script> [timeout=2] [max_bugs=10] [oracle_extra_args...]
#
# All generators write the main instance to /tmp/fuzz_test.cnf and set FUZZ_NBVAR.
# fuzz_loop saves failing instances to /tmp/fail_N.cnf (and /tmp/fail_N.queries when present).

ROOT_PATH="${ROOT_PATH:-.}"
CNF_GENERATOR="${ROOT_PATH}/cnfuzz"
SOLVER="${ROOT_PATH}/minisat"

is_executable_ready() {
    if [[ ! -f "$CNF_GENERATOR" ]] || [[ ! -f "$SOLVER" ]]; then
        make -C "$ROOT_PATH"
    fi
}

# ---------------------------------------------------------------------------
# Generators — each follows the same pattern as the original searchBadExit scripts:
# write header (if any) first, append cnfuzz output, loop until SAT (exit 10).
# ---------------------------------------------------------------------------

# Plain satisfiable CNF with no comment lines (≤ max_vars, default 100).
gen_plain_cnf() {
    local max_vars="${1:-100}" ret=20
    while [ $ret -ne 10 ]; do
        $CNF_GENERATOR | grep -v "^c " > /tmp/fuzz_test.cnf
        FUZZ_NBVAR=$(grep "p cnf" /tmp/fuzz_test.cnf | cut -d ' ' -f3)
        [ "${FUZZ_NBVAR:-0}" -gt "$max_vars" ] && continue
        $SOLVER /tmp/fuzz_test.cnf > /dev/null 2>/dev/null
        ret=$?
    done
}

# CNF with a random projected variable subset (≤ max_vars, default 50).
gen_projected_cnf() {
    local max_vars="${1:-50}" ret=20
    while [ $ret -ne 10 ]; do
        $CNF_GENERATOR | grep -v "^c " > /tmp/fuzz_test.cnf
        FUZZ_NBVAR=$(grep "p cnf" /tmp/fuzz_test.cnf | cut -d ' ' -f3)
        [ "${FUZZ_NBVAR:-0}" -gt "$max_vars" ] && continue
        $SOLVER /tmp/fuzz_test.cnf > /dev/null 2>/dev/null
        ret=$?
    done
    local size rdm=$RANDOM
    case $((rdm % 4)) in
        0) size=$((FUZZ_NBVAR / 7)) ;;
        1) size=$((FUZZ_NBVAR / 5)) ;;
        2) size=$((FUZZ_NBVAR / 3)) ;;
        3) size=$((FUZZ_NBVAR / 2)) ;;
    esac
    [ "${size:-0}" -lt 1 ] && size=1
    local projected header
    header=$(grep "p cnf" /tmp/fuzz_test.cnf)
    projected=$(seq 1 "$FUZZ_NBVAR" | sort -R | head -n "$size" | tr '\n' ' ')
    { echo "$header"; echo "c p show ${projected}0"; grep -v "^p " /tmp/fuzz_test.cnf; } \
        > /tmp/fuzz_test.pcnf
    mv /tmp/fuzz_test.pcnf /tmp/fuzz_test.cnf
}

# CNF with real-valued weights (≤ max_vars, default 20).
gen_weighted_cnf() {
    local max_vars="${1:-20}" ret=20
    while [ $ret -ne 10 ]; do
        echo "c t wmc" > /tmp/fuzz_test.cnf
        $CNF_GENERATOR | grep -v "c max " >> /tmp/fuzz_test.cnf
        FUZZ_NBVAR=$(grep "p cnf" /tmp/fuzz_test.cnf | cut -d ' ' -f3)
        [ "${FUZZ_NBVAR:-0}" -gt "$max_vars" ] && continue
        $SOLVER /tmp/fuzz_test.cnf > /dev/null 2>/dev/null
        ret=$?
    done
    local i p r
    for i in $(seq 1 "$FUZZ_NBVAR"); do
        p=$((RANDOM % 100)); r=$((100 - p))
        [ $p -lt 10 ] && p="0$p"
        [ $r -lt 10 ] && r="0$r"
        echo "c p weight $i 0.$p 0"   >> /tmp/fuzz_test.cnf
        echo "c p weight -$i 0.$r 0"  >> /tmp/fuzz_test.cnf
    done
}

# CNF with complex-valued weights (≤ max_vars, default 200).
gen_complex_cnf() {
    local max_vars="${1:-200}" ret=20
    while [ $ret -ne 10 ]; do
        echo "c t wmc" > /tmp/fuzz_test.cnf
        $CNF_GENERATOR | grep -v "c max " >> /tmp/fuzz_test.cnf
        FUZZ_NBVAR=$(grep "p cnf" /tmp/fuzz_test.cnf | cut -d ' ' -f3)
        [ "${FUZZ_NBVAR:-0}" -gt "$max_vars" ] && continue
        $SOLVER /tmp/fuzz_test.cnf > /dev/null 2>/dev/null
        ret=$?
    done
    local i pw1 pw2 nw1 nw2
    for i in $(seq 1 "$FUZZ_NBVAR"); do
        if [ $((RANDOM % 2)) -eq 0 ]; then
            echo "c p weight $i 1.00 0.00 0"  >> /tmp/fuzz_test.cnf
            echo "c p weight -$i 1.00 0.00 0" >> /tmp/fuzz_test.cnf
        else
            pw1=$((RANDOM % 100)); pw2=$((RANDOM % 100))
            nw1=$((RANDOM % 100)); nw2=$((RANDOM % 100))
            [ $pw1 -lt 10 ] && pw1="0$pw1"; [ $pw2 -lt 10 ] && pw2="0$pw2"
            [ $nw1 -lt 10 ] && nw1="0$nw1"; [ $nw2 -lt 10 ] && nw2="0$nw2"
            echo "c p weight $i 0.$pw1 0.$pw2 0"  >> /tmp/fuzz_test.cnf
            echo "c p weight -$i 0.$nw1 0.$nw2 0" >> /tmp/fuzz_test.cnf
        fi
    done
}

# CNF with max/ind partition and real-valued weights (≤ max_vars, default 30).
gen_maxsharpsat_cnf() {
    local max_vars="${1:-30}" ret=20
    while [ $ret -ne 10 ]; do
        $CNF_GENERATOR | grep -v "^c " > /tmp/fuzz_test.cnf
        FUZZ_NBVAR=$(grep "p cnf" /tmp/fuzz_test.cnf | cut -d ' ' -f3)
        [ "${FUZZ_NBVAR:-0}" -gt "$max_vars" ] && continue
        $SOLVER /tmp/fuzz_test.cnf > /dev/null 2>/dev/null
        ret=$?
    done
    local r=$((FUZZ_NBVAR / 3)); [ $r -lt 1 ] && r=1
    local s; s=$(seq 1 "$FUZZ_NBVAR" | shuf | tr '\n' ' ')
    local max_part; max_part=$(echo "$s" | cut -d ' ' -f1-$r)
    local ind_part; ind_part=$(echo "$s" | cut -d ' ' -f$((r + 1))-)
    local hdr; hdr=$(mktemp)
    {
        echo "c max $max_part 0"
        echo "c ind $ind_part 0"
        local i seed
        for i in $(seq 1 "$FUZZ_NBVAR"); do
            seed=$((RANDOM % 10000))
            printf "c p weight -%d 0.%04d 0\n" "$i" "$seed"
            printf "c p weight %d 0.%04d 0\n"  "$i" "$((10000 - seed))"
        done
    } > "$hdr"
    cat "$hdr" /tmp/fuzz_test.cnf > /tmp/fuzz_test.tmp
    mv /tmp/fuzz_test.tmp /tmp/fuzz_test.cnf
    rm -f "$hdr"
}

# Plain CNF plus random queries at /tmp/fuzz_queries.txt (≤ max_vars, default 50).
gen_cnf_with_queries() {
    local max_vars="${1:-50}" n_queries="${2:-20}" ret=20
    while [ $ret -ne 10 ]; do
        $CNF_GENERATOR | grep -v "^c " > /tmp/fuzz_test.cnf
        FUZZ_NBVAR=$(grep "p cnf" /tmp/fuzz_test.cnf | cut -d ' ' -f3)
        [ "${FUZZ_NBVAR:-0}" -gt "$max_vars" ] && continue
        $SOLVER /tmp/fuzz_test.cnf > /dev/null 2>/dev/null
        ret=$?
    done
    local i ratio tab tmp v
    {
        for i in $(seq 1 "$n_queries"); do
            tab=$(seq 1 "$FUZZ_NBVAR" | shuf | tr '\n' ' ')
            ratio=$(echo "$FUZZ_NBVAR / 100 * $((RANDOM % 10)) + 1" | bc -l | cut -d '.' -f1)
            [ "${ratio:-0}" -lt 1 ] && ratio=1
            tmp=$(echo "$tab" | cut -d ' ' -f1-"$ratio")
            [ $((RANDOM % 2)) -eq 0 ] && printf "m " || printf "d "
            for v in $tmp; do
                [ $((RANDOM % 2)) -eq 0 ] && printf -- "-%d " "$v" || printf "%d " "$v"
            done
            echo "0"
        done
    } > /tmp/fuzz_queries.txt
}

# ---------------------------------------------------------------------------
# Main fuzzing driver
# ---------------------------------------------------------------------------
fuzz_loop() {
    local generator="$1" oracle="$2" timeout="${3:-2}" max_bugs="${4:-10}"
    shift 4
    local oracle_args=("$@")
    local nbBugs=1 cpt=0 code hdr

    while true; do
        printf "tested: %-6d  bugs: %d\r" "$cpt" "$((nbBugs - 1))"
        eval "$generator"        
        timeout "$timeout" bash "$oracle" /tmp/fuzz_test.cnf "${oracle_args[@]}" \
            > /dev/null 2>/dev/null
        code=$?

        [ $code -ne 124 ] && ((cpt++))

        if [ $code -ne 0 ] && [ $code -ne 124 ]; then
            hdr=$(grep "p cnf" /tmp/fuzz_test.cnf | cut -d ' ' -f3-)
            [ -z "$hdr" ] && continue
            echo ""
            echo "Bug #$((nbBugs)): /tmp/fail_${nbBugs}.cnf  ($hdr)"
            cp /tmp/fuzz_test.cnf "/tmp/fail_${nbBugs}.cnf"
            [ -s /tmp/fuzz_queries.txt ] && cp /tmp/fuzz_queries.txt "/tmp/fail_${nbBugs}.queries"
            ((nbBugs++))
            if [ $nbBugs -gt $max_bugs ]; then
                echo "Reached $max_bugs bugs — stopping."
                exit 1
            fi
        fi
    done
}
