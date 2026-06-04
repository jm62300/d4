# Testing scripts

All scripts must be run from the `scripts/` directory.  
The first time you run any script it will build `cnfuzz` and `minisat` via `make`.

---

## Quick reference

| What you want to test | Script | Detects |
|---|---|---|
| Plain model counting | `searchBadExitModelCounting.sh` | crashes + wrong count |
| Projected model counting | `searchBadExitProjected.sh` | crashes + wrong count |
| Weighted counting (real weights) | `searchBadExitWModelCounting.sh` | crashes only |
| Weighted counting (complex weights) | `searchBadExitComplexModelCounting.sh` | crashes + wrong value |
| Max#SAT / ERE | `searchBadExitMaxSharpSAT.sh` | crashes + wrong optimum |
| Model counting with queries | `searchBadExitModelCountingQueries.sh` | crashes + wrong answer |
| Any binary, quickly | `searchBadExitQuick.sh <cmd>` | crashes only |

---

## Fuzz-testing scenarios

### Plain model counting

```bash
bash searchBadExitModelCounting.sh             # default: 2s timeout, stop after 10 bugs
bash searchBadExitModelCounting.sh 5           # 5s timeout
bash searchBadExitModelCounting.sh 5 20        # 5s timeout, stop after 20 bugs
```

Oracle used: `testModelCounter.sh`  
Reference: `./d4_static -m counting -i`  
Tested: `../c++/counter/build/counter -i`

---

### Projected model counting

```bash
bash searchBadExitProjected.sh
bash searchBadExitProjected.sh 3 10
```

Oracle used: `testProjModelCounting.sh`  
The generator adds a random `c p show` line selecting a random subset of variables.

---

### Weighted model counting — real weights

```bash
bash searchBadExitWModelCounting.sh
bash searchBadExitWModelCounting.sh 3
```

Crash detection only (no correctness oracle).  
The generator adds `c t wmc` and one `c p weight` line per literal.  
Override the tested binary:

```bash
TESTED_METHOD="../c++/counter/build/counter --float 1 -i" bash searchBadExitWModelCounting.sh
```

---

### Weighted model counting — complex weights

```bash
bash searchBadExitComplexModelCounting.sh
bash searchBadExitComplexModelCounting.sh 5 20
```

Oracle used: `testComplexModelCounter.sh`  
Reference: ganak (`/home/lagniez/Works/Softs/solvers/ganak2.4.6/ganak --mode 6`)  
Weights are random pairs `(re, im)` in `[0.00, 0.99]²`.  
Comparison uses a tolerance of `1e-9`.

---

### Max#SAT

```bash
bash searchBadExitMaxSharpSAT.sh
bash searchBadExitMaxSharpSAT.sh 2 10
```

Oracle used: `testMaxSharpSAT.sh`  
The generator partitions variables into `c max` (1/3) and `c ind` (2/3) with random real weights.  
The oracle checks:
1. The claimed optimum equals the count under the returned valuation.
2. The result matches a second Max#SAT run with cuts disabled.

---

### Model counting with queries

```bash
bash searchBadExitModelCountingQueries.sh
bash searchBadExitModelCountingQueries.sh 3
```

Oracle used: `testModelCountingQuery.sh`  
Generates 20 random queries (mix of `m`-count and `d`-decision) per instance.  
Failures are saved as `/tmp/fail_N.cnf` + `/tmp/fail_N.queries`.

---

### Generic crash fuzzer (any binary)

Use this when you have a new binary and just want to know if it crashes:

```bash
bash searchBadExitQuick.sh "../c++/counter/build/counter -i"
bash searchBadExitQuick.sh "../c++/counter/build/counter -i" 50      # max 50 variables
bash searchBadExitQuick.sh "../c++/counter/build/counter -i" 50 3    # 3s timeout
bash searchBadExitQuick.sh "../c++/counter/build/counter -i" 50 3 20 # stop after 20 bugs
```

---

## Testing a single known instance

Use the `test*.sh` scripts directly when you already have a failing file:

```bash
# Plain model counting
bash testModelCounter.sh /tmp/fail_1.cnf

# Projected model counting
bash testProjModelCounting.sh /tmp/fail_1.cnf

# Complex weighted counting
bash testComplexModelCounter.sh /tmp/fail_1.cnf

# Max#SAT / ERE
bash testMaxSharpSAT.sh /tmp/fail_1.cnf
bash testEre.sh         /tmp/fail_1.cnf

# MaxT (weighted Max#SAT with float semiring)
bash testMaxT.sh /tmp/fail_1.cnf

# Min#SAT  (brute-force oracle — slow, keep instances tiny)
bash testMinSharpSAT.sh /tmp/fail_1.cnf

# Threshold Max#SAT
bash testMaxSharpSATThreshold.sh /tmp/fail_1.cnf

# Enumerative Max#SAT  (brute-force oracle — keep instances tiny)
bash testMaxSharpSATEnum.sh /tmp/fail_1.cnf

# Preprocessor correctness
bash testPreproc.sh /tmp/fail_1.cnf

# Cache symmetry efficiency (checks that sym cache hits ≥ classic cache hits)
bash testCacheSym.sh /tmp/fail_1.cnf

# Erosion
bash testErosion.sh /tmp/fail_1.cnf

# Model counting queries
bash testModelCountingQuery.sh /tmp/fail_1.cnf /tmp/fail_1.queries
```

Exit code: `0` = correct, `1` = bug found, `124` = timeout.

---

## Overriding the tested binary

All oracle scripts read `TESTED_METHOD` and `MODEL_COUNTER` from the environment,
falling back to their compiled-in defaults.  You can override without editing files:

```bash
# Test a debug build instead of the default release build
TESTED_METHOD="../build/d4_debug -m counting -i" bash testModelCounter.sh /tmp/fail_1.cnf

# Wire a non-default binary into the full fuzzing loop
TESTED_METHOD="../build/d4_debug -m counting -i" bash searchBadExitModelCounting.sh 5
```

---

## Where failing instances are saved

The fuzzing loops save up to 10 (default) failing instances:

| File | Contents |
|---|---|
| `/tmp/fail_1.cnf` … `/tmp/fail_N.cnf` | The CNF that triggered the bug |
| `/tmp/fail_1.queries` … | Query file (only for the query-based fuzzer) |

To reproduce bug #3 found by the Max#SAT fuzzer:
```bash
bash testMaxSharpSAT.sh /tmp/fail_3.cnf
```

---

## Building the test tools

```bash
make          # builds cnfuzz and minisat into scripts/
make clean    # removes them
```

The `d4_static` reference binary must already be present in `scripts/`.  
Demo binaries (`counter`, `compiler`, …) are built from their own `c++/*/build.sh`.
