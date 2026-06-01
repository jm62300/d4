# Cube-counter session notes (2026-06-01)

## Architecture

`c++/cube-counter/` is a standalone demo linking against `libd4.a`.
Entry point: `cubeCounter()` in `src/CubeCounterDemo.cpp`.

Flow:
1. Build a `ProblemManager` for the full formula.
2. Use a `ClauseSelector` to pick an initial subset of clauses → **F_easy**.
3. If `--extendEasy` is on (default): absorb every remaining clause whose
   variables are all already in `vars(F_easy)` — does not grow the variable
   set, strengthens F_easy for free.
4. Optionally strengthen F_easy with implied V_easy clauses (see below).
5. `cubeAndCount()` compiles F_easy into a decision-DNNF, enumerates its
   partial models (cubes), and for each cube counts the full formula
   conditioned on that cube via `DpllStyleMethod<mpz_int, MpzIntSemiring>`.

Empty F_easy is valid: produces one empty cube → plain direct count.

## Selectors (`src/selectors/`)

| `--selectorStrategy` | Class | Description |
|---|---|---|
| `primal-cut` | `PrimalCutSelector` | Primal hypergraph bisection (PaToH); returns cut clauses |
| `iterative-primal-cut` | `IterativePrimalCutSelector` | Recursive bisection up to `--maxDepth` levels |
| `high-degree` *(default)* | `HighDegreeVariableSelector` | Gain-based greedy — see below |

### high-degree strategy — REWORKED 2026-06-01 (cube-aware), VALIDATION PENDING

**Objective (clarified with user):** minimise **end-to-end runtime**, keeping
the variable-ratio budget. Runtime ≈ (#cubes) × (cost of counting F | cube),
and **#cubes == number of partial models of F_easy**.

**Key model:** `log2(#cubes) ≈ |V_easy| − Σ_{c∈F_easy} −log2(1 − 2^−|c|)`.
Adding a variable to V_easy adds **one free dimension (+1 bit ≈ ×2 cubes)**
unless it also *completes* clauses that constrain it back down. A completed
binary clause is worth only `−log2(1−1/4) = 0.415` bits, ternary `0.193`, etc.
So a variable that completes a single binary clause STILL grows the cube count.

**Why the old version was useless:** it always spent the *entire* variable
budget, adding variables even when they completed no clause (tie-broken on raw
`degree`). Every such free variable ~doubled the cube count. Measured on
`test.cnf` (460 var / 794 cl) at ratio 0.1: 46 vars → 68 clauses but
**162903 cubes**.

**New algorithm (`HighDegreeVariableSelector.cpp`):**
- `constraintBits[s] = −log2(1 − 2^−s)` precomputed (short clauses dominate).
- `completion[v]` = Σ constraintBits over clauses `v` would complete now
  (clauses where `v` is the sole missing variable); maintained incrementally.
- `seedScore[v]` = Σ `2^−remaining[c]` — closeness potential, used ONLY while
  F_easy is still empty, to pick the first seed variable(s).
- Loop each step: if any var can complete a clause, pick max `completion`
  (degree tiebreak); else seeding move pick max `seedScore`.
  - Seeding moves (complete nothing, pure +1 bit) allowed only to assemble the
    FIRST clause, capped at `kMaxSeed = min(budget, max(2, maxClauseSize))`.
  - After seeding, **reject any variable with `completion[best] < kMinBits`**
    and stop — this is the gate that prevents cube explosion. `kMinBits`
    defaults to 1.0 (net non-increasing cube estimate).
- Reports `~cubes` estimate in the `c [HIGH-DEGREE] ...` log line.

`m_minBits` is a new constructor param (default 1.0), wired to the new
`--hdMinBits` option.

**STATUS: NOT YET VALIDATED.** The `--hdMinBits 1.0` gate was just built but the
confirming run on `test.cnf` was interrupted before results. **First thing to do
next session:** run and confirm the gate actually crushes the cube count:

```bash
cd c++/cube-counter && ./build.sh -j
for r in 0.1 0.2 0.3 0.5; do
  ./build/cube-counter -i test.cnf --targetRatio $r 2>&1 \
    | grep -E 'HIGH-DEGREE|cubes is'
done
```

Expectation: `~cubes` (estimate) and the real "the number of cubes is N" should
now both be small (single/low double digits) instead of 160k–360k, because the
gate stops adding variables once they no longer pay for their dimension.

**Open risk / next tuning:** the gate may make F_easy *too* small on some
instances (few clauses → each conditioned count ≈ full formula → no
decomposition speedup, but never worse than ~direct count). The trade-off dial
is `--hdMinBits`: lower (<1.0) admits more variables/clauses (better
decomposition, more cubes), higher is stricter. This can only be tuned against
**real end-to-end runtime**, which requires removing the `#if 1` debug guard in
`cubeAndCount()` (see Pending) so the actual counting loop runs.

**Earlier dead-end (do not repeat):** an α-decay "partial credit / clustering"
potential was tried first — it grows a dense variable community and *maximised
clauses*, but that is the WRONG objective: at fixed 46 vars it gave 71 clauses
yet **359992 cubes** (worse than old). Clause count is not the right proxy;
cube count (model count of F_easy) is.

## Options (`OptionCubeCounter`)

| Option | Default | Meaning |
|---|---|---|
| `--selectorStrategy` | `high-degree` | Which selector to use |
| `--maxDepth` | 3 | Recursion depth for iterative-primal-cut |
| `--targetRatio` | 0.1 | Variable budget fraction for high-degree (hard cap) |
| `--hdMinBits` | 1.0 | Min model-count reduction (bits) to add a var in high-degree; 1.0 = non-increasing cube estimate, lower = more clauses/more cubes |
| `--extendEasy` | `true` | After selection, absorb clauses that don't grow `vars(F_easy)` |
| `--strengthen` | `none` | Derive implied clauses over `vars(F_easy)` before compilation (`none`\|`resolution`\|`sat`) |
| `--strengthenTime` | `30.0` | Wall-clock time limit in seconds for the `sat` strengthen phase |
| `--strengthenSteps` | `10` | Max resolution steps per derivation path (resolution strategy) |
| `--strengthenMaxClauses` | `100` | Max new V_easy clauses to collect (resolution strategy) |

### Strengthen strategies

**`resolution`** — multi-step BFS over resolution derivations.
Starting from every clause in F that has at least one variable outside V_easy,
repeatedly resolves on outside-variable pivots using original F clauses.
A path succeeds when the derived clause lands fully within V_easy.

Bounds that prevent blowup:
- `--strengthenSteps`: max resolution steps per path (default 10).
- `--strengthenMaxClauses`: stop after collecting this many new clauses (default 100).
- `kMaxClauseSize = 20`: discard resolvents larger than this.
- Visited set on intermediate clause content: each distinct intermediate is
  explored at most once.

Previous single-step version added zero clauses in practice because it required
exactly 1 outside variable; the multi-step BFS handles the common case where
multiple outside variables must be resolved away in sequence.

**`sat`** — enumerate models of F_easy with a dedicated CaDiCaL instance.
For each model UNSAT w.r.t. F, extract the minimal failing core via
`failed()` and add its negation as a new clause to F_easy (implied by F →
correctness preserved). SAT models are blocked in the enumeration solver only.
Stops after `--strengthenTime` seconds.

## Pending / in-progress

### Debug scaffolding in `cubeAndCount()`
There is a `#if 1` / `return` block (around line 292) that short-circuits
into a cube-count-only loop for testing enumeration. The real counting loop
below it is **currently dead code**. Remove the guard once cube enumeration
is validated.

### Cleanup in the counting loop
Debug prints to remove before production:
- `"the size of sigma is ..."`
- `"it is SAT then we have to count ..."`
- `"it is unsat\n"`
- The per-clause dump of F_easy (lines ~274–279 in `cubeAndCount()`)

### Validate resolution strengthening
The multi-step resolution strategy was just implemented (2026-06-01).
Next step: run on a test instance and check how many clauses it actually adds.

```bash
cd c++/cube-counter
./build/cube-counter -i test.cnf --strengthen resolution
# or try scripts/1test_reduced.cnf / scripts/bug.cnf
```

Tune `--strengthenSteps` and `--strengthenMaxClauses` based on results.
If still few clauses, consider adding a `"probing"` strategy (failed literal
detection over V_easy vars using a CaDiCaL backbone check).

### Validation (end-to-end)
Once the `#if 1` guard is removed:
- Run on a small known instance; compare result against the direct counter.
- Tune `--targetRatio` on real benchmarks to balance cube count vs. per-cube cost.

### Future selector strategies
The user plans to add more variable-selection heuristics beyond `high-degree`.

## Build

```bash
cd c++/cube-counter && ./build.sh -j
```

Test instances: `test.cnf`, `scripts/1test_reduced.cnf`, `scripts/bug.cnf`,
`instancesTest/cnfs/`.
