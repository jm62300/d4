# Cube-counter session notes (2026-05-31)

## Architecture

`c++/cube-counter/` is a standalone demo linking against `libd4.a`.
Entry point: `cubeCounter()` in `src/CubeCounterDemo.cpp`.

Flow:
1. Build a `ProblemManager` for the full formula.
2. Use a `ClauseSelector` to pick an initial subset of clauses → **F_easy**.
3. If `--extendEasy` is on (default): absorb every remaining clause whose
   variables are all already in `vars(F_easy)` — does not grow the variable
   set, strengthens F_easy for free.
4. `cubeAndCount()` compiles F_easy into a decision-DNNF, enumerates its
   partial models (cubes), and for each cube counts the full formula
   conditioned on that cube via `DpllStyleMethod<mpz_int, MpzIntSemiring>`.

Empty F_easy is valid: produces one empty cube → plain direct count.

## Selectors (`src/selectors/`)

| `--selectorStrategy` | Class | Description |
|---|---|---|
| `primal-cut` | `PrimalCutSelector` | Primal hypergraph bisection (PaToH); returns cut clauses |
| `iterative-primal-cut` | `IterativePrimalCutSelector` | Recursive bisection up to `--maxDepth` levels |
| `high-degree` *(default)* | `HighDegreeVariableSelector` | Gain-based greedy — see below |

### high-degree strategy

Grows V_easy one variable at a time up to `--targetRatio * nbVar` variables
(default 0.3). At each step picks the variable that immediately covers the
most new clauses (gain = clauses whose last missing variable is this one);
degree breaks ties. Gain is maintained incrementally: when `remaining[c]`
drops to 1, the sole missing variable gets its gain incremented.

Goal: maximise |F_easy| (→ fewer cubes) while keeping |vars(F_easy)| small
(→ small DNNF).

## Options (`OptionCubeCounter`)

| Option | Default | Meaning |
|---|---|---|
| `--selectorStrategy` | `high-degree` | Which selector to use |
| `--maxDepth` | 3 | Recursion depth for iterative-primal-cut |
| `--targetRatio` | 0.3 | Variable budget fraction for high-degree |
| `--extendEasy` | `true` | After selection, absorb clauses that don't grow `vars(F_easy)` |

## Pending / in-progress

### Debug scaffolding in `cubeAndCount()`
There is a `#if 1` / `return` block (around line 116) that short-circuits
into a cube-count-only loop for testing enumeration. The real counting loop
below it is **currently dead code**. Remove the guard once cube enumeration
is validated.

### Cleanup in the counting loop
Debug prints to remove before production:
- `"the size of sigma is ..."`
- `"it is SAT then we have to count ..."`

### Validation
- Run on a small known instance; compare result against the direct counter.
- Tune `--targetRatio` on real benchmarks to balance cube count vs. per-cube
  cost.

### Future selector strategies
The user plans to add more variable-selection heuristics beyond `high-degree`.

## Build

```bash
cd c++/cube-counter && ./build.sh -j
```

Test instances: `scripts/1test_reduced.cnf`, `scripts/bug.cnf`.
