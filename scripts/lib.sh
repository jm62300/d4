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
# All generators write the main instance to /tmp/fuzz_test.cnf and export FUZZ_NBVAR.
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
# Internal helper: write a satisfiable plain CNF (no comment lines) with at
# most $1 variables to /tmp/fuzz_test.cnf.  Sets FUZZ_NBVAR.
# ---------------------------------------------------------------------------
_gen_sat_cnf() {
    local max_vars="${1:-100}"
    while true; do
        $CNF_GENERATOR | grep -v "^c " > /tmp/fuzz_test.cnf
        FUZZ_NBVAR=$(grep "^p cnf" /tmp/fuzz_test.cnf | cut -d' ' -f3)
        [[ $FUZZ_NBVAR -gt $max_vars ]] && continue
        $SOLVER /tmp/fuzz_test.cnf > /dev/null 2>/dev/null && break
    done
}

# ---------------------------------------------------------------------------
# Generators
# ---------------------------------------------------------------------------

# Plain satisfiable CNF (≤ max_vars variables, default 100).
gen_plain_cnf() {
    _gen_sat_cnf "${1:-100}"
}

# CNF with a random projected variable subset via "c p show" (≤ max_vars, default 50).
# Projection size is chosen uniformly from {N/7, N/5, N/3, N/2}.
gen_projected_cnf() {
    _gen_sat_cnf "${1:-50}"
    local size projected
    case $((RANDOM % 4)) in
        0) size=$((FUZZ_NBVAR / 7)) ;;
        1) size=$((FUZZ_NBVAR / 5)) ;;
        2) size=$((FUZZ_NBVAR / 3)) ;;
        3) size=$((FUZZ_NBVAR / 2)) ;;
    esac
    [[ ${size:-0} -lt 1 ]] && size=1
    projected=$(seq 1 "$FUZZ_NBVAR" | sort -R | head -n "$size" | tr '\n' ' ')
    {
        grep "^p cnf" /tmp/fuzz_test.cnf
        echo "c p show ${projected}0"
        grep -v "^p " /tmp/fuzz_test.cnf
    } > /tmp/fuzz_test.pcnf
    mv /tmp/fuzz_test.pcnf /tmp/fuzz_test.cnf
}

# CNF with real-valued weights: each variable gets w and (1-w) for pos/neg literal
# (≤ max_vars, default 20).
gen_weighted_cnf() {
    _gen_sat_cnf "${1:-20}"
    sed -i '/^p cnf/i c t wmc' /tmp/fuzz_test.cnf
    local i p r
    for i in $(seq 1 "$FUZZ_NBVAR"); do
        p=$((RANDOM % 100)); r=$((100 - p))
        [[ $p -lt 10 ]] && p="0$p"
        [[ $r -lt 10 ]] && r="0$r"
        echo "c p weight $i 0.$p 0"   >> /tmp/fuzz_test.cnf
        echo "c p weight -$i 0.$r 0"  >> /tmp/fuzz_test.cnf
    done
}

# CNF with independent complex-valued weights for each literal (≤ max_vars, default 200).
gen_complex_cnf() {
    _gen_sat_cnf "${1:-200}"
    sed -i '/^p cnf/i c t wmc' /tmp/fuzz_test.cnf
    local i pw1 pw2 nw1 nw2
    for i in $(seq 1 "$FUZZ_NBVAR"); do
        pw1=$((RANDOM % 100)); pw2=$((RANDOM % 100))
        nw1=$((RANDOM % 100)); nw2=$((RANDOM % 100))
        printf "c p weight %d 0.%02d 0.%02d 0\n"  "$i"  "$pw1" "$pw2" >> /tmp/fuzz_test.cnf
        printf "c p weight -%d 0.%02d 0.%02d 0\n" "$i"  "$nw1" "$nw2" >> /tmp/fuzz_test.cnf
    done
}

# CNF with max/ind variable partition and real-valued weights, suitable for
# Max#SAT / ERE testing (≤ max_vars, default 30).
gen_maxsharpsat_cnf() {
    _gen_sat_cnf "${1:-30}"
    local r=$((FUZZ_NBVAR / 3)); [[ $r -lt 1 ]] && r=1
    local shuffled max_part ind_part i seed pbp pbn
    shuffled=$(seq 1 "$FUZZ_NBVAR" | shuf)
    max_part=$(echo "$shuffled" | head -n "$r"          | tr '\n' ' ')
    ind_part=$(echo "$shuffled" | tail -n "+$((r + 1))" | tr '\n' ' ')
    {
        echo "c max ${max_part}0"
        echo "c ind ${ind_part}0"
        for i in $(seq 1 "$FUZZ_NBVAR"); do
            seed=$((RANDOM % 10000))
            pbp=$(bc -l <<< "scale=4; $seed / 10000")
            pbn=$(bc -l <<< "scale=4; (10000 - $seed) / 10000")
            echo "c p weight -$i $pbp 0"
            echo "c p weight $i $pbn 0"
        done
        cat /tmp/fuzz_test.cnf
    } > /tmp/fuzz_test.tmp
    mv /tmp/fuzz_test.tmp /tmp/fuzz_test.cnf
}

# Plain CNF plus a random query file at /tmp/fuzz_queries.txt.
# Queries are randomly typed "m" (model count) or "d" (decision) with signed literals.
gen_cnf_with_queries() {
    _gen_sat_cnf "${1:-50}"
    local n_queries="${2:-20}" i ratio tab tmp v
    {
        for i in $(seq 1 "$n_queries"); do
            tab=$(seq 1 "$FUZZ_NBVAR" | shuf)
            ratio=$(echo "$FUZZ_NBVAR / 100 * $((RANDOM % 10)) + 1" | bc -l | cut -d'.' -f1)
            [[ -z "$ratio" || $ratio -lt 1 ]] && ratio=1
            tmp=$(echo "$tab" | head -n "$ratio")
            [[ $((RANDOM % 2)) -eq 0 ]] && printf "m " || printf "d "
            for v in $tmp; do
                [[ $((RANDOM % 2)) -eq 0 ]] && printf -- "-%d " "$v" || printf "%d " "$v"
            done
            echo "0"
        done
    } > /tmp/fuzz_queries.txt
}

# ---------------------------------------------------------------------------
# Main fuzzing driver
# ---------------------------------------------------------------------------
# Usage: fuzz_loop <generator_cmd> <oracle_script> [timeout=2] [max_bugs=10] [oracle_extra_args...]
#
# generator_cmd   : shell command that calls one of the gen_* functions above
# oracle_script   : path to a test*.sh script; receives /tmp/fuzz_test.cnf as $1;
#                   exit 0 = correct, non-0 = bug (crash or wrong answer)
# oracle_extra_args : forwarded verbatim after the bench path (e.g. query file path)
#
# Environment:
#   TESTED_METHOD   used by crash_check.sh and optionally by oracle scripts
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

        [[ $code -ne 124 ]] && ((cpt++))

        if [[ $code -ne 0 && $code -ne 124 ]]; then
            hdr=$(grep "^p cnf" /tmp/fuzz_test.cnf | cut -d' ' -f3-)
            echo ""
            echo "Bug #$((nbBugs)): /tmp/fail_${nbBugs}.cnf  ($hdr)"
            cp /tmp/fuzz_test.cnf "/tmp/fail_${nbBugs}.cnf"
            [[ -s /tmp/fuzz_queries.txt ]] && cp /tmp/fuzz_queries.txt "/tmp/fail_${nbBugs}.queries"
            ((nbBugs++))
            if [[ $nbBugs -gt $max_bugs ]]; then
                echo "Reached $max_bugs bugs — stopping."
                exit 1
            fi
        fi
    done
}
