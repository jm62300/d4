# Cube-counter session notes (updated 2026-06-02)

## Architecture

`c++/cube-counter/` is a standalone demo linking against `libd4.a`.
Entry point: `cubeCounter()` in `src/CubeCounterDemo.cpp`.

Flow:
1. Build a `ProblemManager` for the full formula.
2. Use a `ClauseSelector` to pick an initial subset of clauses → **F_easy**.
3. If `--extendEasy` is on (default): absorb every remaining clause whose
   variables are all already in `vars(F_easy)` — does not grow the variable
   set, strengthens F_easy for free.
4. Optionally strengthen F_easy (see below).
5. Build **F' = F \ F_easy** — used for SAT checking and counting, since every
   cube already satisfies F_easy by construction.
6. `cubeAndCount()` compiles F_easy into a decision-DNNF, does a fast
   first-pass cube count (polynomial DAG traversal), and either:
   - Falls back to direct counting if `cube_count > --maxCubes`, OR
   - Enumerates cubes and for each cube counts F' conditioned on that cube
     via `DpllStyleMethod<mpz_int, MpzIntSemiring>`.

Empty F_easy is valid: produces one empty cube → plain direct count.

## Selectors (`src/selectors/`)

| `--selectorStrategy` | Class | Description |
|---|---|---|
| `primal-cut` | `PrimalCutSelector` | Primal hypergraph bisection (PaToH); returns cut clauses |
| `iterative-primal-cut` | `IterativePrimalCutSelector` | Recursive bisection up to `--maxDepth` levels |
| `high-degree` *(default)* | `HighDegreeVariableSelector` | Iterative neighbourhood expansion — see below |

### high-degree strategy (rewritten 2026-06-02)

**Algorithm:**
1. V = {}
2. While |V| < `targetRatio * nbVar`:
   - Pick v* = arg max_{v ∉ V} occ[v] (occurrence in original F)
   - V ← V ∪ (all variables in every clause containing v*)
   - F_easy ← {c ∈ F : var(c) ⊆ V}  ← clauses *fully* inside V
3. Return F_easy

Key: one-hop neighbourhood expansion around each seed. The `--targetRatio`
(default 0.15 = 15%) bounds |V|. The old cube-aware gain-based code is gone.

**Key observation on `mc2025_track1_033.cnf`:**
- At targetRatio=0.15 → ~193,880 cubes, 99%+ pruning rate.
- Root cause: max-occurrence vars are hub nodes; conditioning on them does NOT
  decompose the primal graph. F_easy is structurally thin.
- The `primal-cut` selector is the theoretically correct choice for
  decomposition (it finds actual graph separators via PaToH), but has not been
  tested on this instance yet.

## Options (`OptionCubeCounter`)

| Option | Default | Meaning |
|---|---|---|
| `--selectorStrategy` | `high-degree` | Which selector to use |
| `--maxDepth` | 3 | Recursion depth for iterative-primal-cut |
| `--targetRatio` | 0.15 | Variable budget fraction for high-degree |
| `--hdMinBits` | 1.0 | (unused in current selector — kept for compat) |
| `--maxCubes` | 1000 | Max F_easy cube count before falling back to direct counting |
| `--extendEasy` | `true` | After selection, absorb clauses that don't grow `vars(F_easy)` |
| `--strengthen` | `none` | Derive implied clauses over `vars(F_easy)` before compilation (`none`\|`resolution`\|`elimination`\|`sat`) |
| `--strengthenTime` | `30.0` | Time limit for `sat` strengthen phase |
| `--strengthenSteps` | `10` | Max resolution steps per derivation path |
| `--strengthenMaxClauses` | `100` | Max new V_easy clauses to collect |
| `--strengthenMaxProduct` | `20` | Max \|P\|×\|N\| resolvent budget per variable (elimination) |
| `--preStrengthen` | `0` | SAT-based pre-strengthening iterations before DNNF compilation (0=off) |
| `--externSolver` | `""` | Path to external SAT solver binary for per-cube checks (empty = CaDiCaL) |

## F' optimisation (2026-06-02)

Every cube σ produced by the DNNF enumerator satisfies F_easy by construction.
Therefore **F ∧ σ ≡ F' ∧ σ** where F' = F \ F_easy.

Both the per-cube SAT checker (CaDiCaL) and the full counter
(`DpllStyleMethod`) are built from F' only — fewer clauses, faster
propagation, correct counts.

## Per-cube SAT checker — CaDiCaL with core learning

The checker uses `simplify=0` (pure CDCL) so `failed()` returns a genuine
subset of the assumed cube literals. On each UNSAT cube the negated core is
added back as a learned clause, so future cubes that extend the same UNSAT
sub-assignment are pruned by unit propagation without a full solve.

**`--externSolver` (Kissat test, 2026-06-02):**
Kissat was tested as an external process (`fork`+`execvp`, DIMACS piped to
`/dev/stdin`). **CaDiCaL is faster** for this workload: the incremental clause
learning accumulated across 4000+ cube checks more than compensates for
Kissat's per-solve speed. External-process overhead (~0.5 ms/call × 4000
cubes) is secondary. Kissat path kept for completeness but not recommended.

## Tseitin DNNF pruning (implemented 2026-06-02, left for future work)

`DecDNNFSemiring` has two new public methods:

- **`buildTseitin(root, tsOffset, addClause)`**: traverses the DNNF like
  `printNNF` but instead of printing, emits `(-x_n ∨ x_ci)` implication
  clauses for every AND node. Node `n` maps to variable `tsOffset + n`. This
  allows CaDiCaL to propagate through AND-chains when a sub-circuit node is
  assumed.

- **`enumeratePartialModelsWithPruning(root, assums, nbVar, shouldPrune, onModel)`**:
  identical to `enumeratePartialModels` but calls `shouldPrune(target_node,
  accumulated_partial_model)` before recursing into each OR branch. If the
  pruner returns true, the entire subtree is skipped.

In `cubeAndCount` this was wired up so the SAT check happens at every OR branch
(not just complete cubes), allowing large subtrees to be pruned early. **The
improvement was modest on the tested instance** — the fundamental bottleneck is
not the number of SAT calls but the formula structure. Left in the code but
**not the default path** — `enumeratePartialModels` is used.

The `declare_more_variables(formula.nbVar)` call must come **before** loading
F' clauses into CaDiCaL because the bundled bipe build has `factor` +
`factorcheck` enabled (requires explicit variable declaration).

## `cubeAndCount()` flow (as of 2026-06-02)

1. Compiles F_easy (+ extra clauses) → DNNF (DecDNNFSemiring).
2. Fast first-pass cube count — polynomial DNNF traversal.
3. If `cube_count > maxCubes` → direct count of F' and return.
4. Pre-serialises F' as a string (reused for all per-cube checks).
5. Builds CaDiCaL checker on F' (`simplify=0`).
6. `enumeratePartialModels`: for each cube σ:
   - CaDiCaL check F' ∧ σ. If UNSAT: extract core, add negated core to
     CaDiCaL, prune.
   - If SAT: `fullCounter->count(allVars, σ, nullStream)`.
7. Prints `s <total>`.

## Build

```bash
cd c++/cube-counter && ./build.sh -j
```

Test instances: `test.cnf`, `scripts/1test_reduced.cnf`, `scripts/bug.cnf`,
`instancesTest/cnfs/`.

Correctness tested with `scripts/searchBadExitModelCounting.sh` +
`scripts/testModelCounter.sh` — 440+ random SAT instances passed, 0 failures.

## Hard benchmark

`/data/Benchmarks/counting/MC2025_all/mc2025_track1_all/mc2025_track1_033.cnf`

At `--targetRatio 0.15` this produces 193,880 cubes with 99%+ pruning rate.
All approaches tried this session (SAT pre-strengthen, Tseitin pruning, core
learning, Kissat) failed to make a dent on cube count. Root cause: the
high-degree selector picks hub variables that don't decompose the primal graph.

## Next session (TODO)

### 1. Try `primal-cut` selector on the hard benchmark
The `primal-cut` / `iterative-primal-cut` strategies find genuine graph
separators via PaToH, which is the correct decomposition for cube-and-count.
After conditioning on cut variables, the two sub-problems become independent
and the full counter exploits component analysis.

```bash
./build/cube-counter -i <hard.cnf> --selectorStrategy primal-cut
./build/cube-counter -i <hard.cnf> --selectorStrategy iterative-primal-cut \
  --maxDepth 4
```

### 2. Revisit Tseitin pruning
The `enumeratePartialModelsWithPruning` infrastructure is in place. The
pruning check currently happens at every OR branch (expensive). A better
strategy: only prune at OR branches near the root (depth-bounded), where one
SAT call can eliminate the largest subtrees. Add a depth parameter to
`shouldPrune`.

### 3. Integrate Kissat as a library (if CaDiCaL remains a bottleneck)
Link against `libkissat.a` and use the IPASIR interface for incremental
solving with assumptions. Only worth doing if Kissat solves the hard instance
sub-problems faster than CaDiCaL.

### 4. Better variable selection heuristic
Instead of max-occurrence (hub variables), try selecting variables that
maximise the number of clauses that become *fully contained* in V after
expansion — i.e. maximise |F_easy| per variable added. This is a one-step
look-ahead that targets decomposability rather than connectivity.
