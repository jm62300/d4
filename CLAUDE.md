# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What is d4

**d4** is a C++20 static library (`libd4.a`) for solving problems beyond NP. It supports model counting (weighted/projected), compiling CNF formulas into decision-DNNF, Max#SAT/Min#SAT, and QBF counting. It is not a standalone binary — consumers link against `libd4.a`.

## Build Commands

### Building the library

```bash
./build.sh          # standard release (O3)
./build.sh -d       # debug (O0 + symbols)
./build.sh -s       # static linking
./build.sh -p       # profiling (O3 + pg)
./build.sh -j       # parallel compilation
```

Output: `build/libd4.a`

Dependencies: CMake ≥ 3.14, C++20 compiler, `libgmp`, `libgmpxx`, `libz`, Boost. On Linux, `patoh` is also required (bundled in `3rdParty/patoh/`). The `optree` library is fetched automatically via CMake `FetchContent` from GitLab.

### Building demo applications

Each demo in `c++/` builds independently and will rebuild `libd4.a` as a dependency:

```bash
cd c++/compiler && ./build.sh    # decision-DNNF compiler
cd c++/counter  && ./build.sh    # model counter
cd c++/maxT     && ./build.sh    # Max#SAT
cd c++/qbfCounter && ./build.sh  # QBF counter
cd c++/server   && ./build.sh    # server mode
```

Build flags (`-d`, `-s`, `-p`, `-j`) are passed through to the library build.

### Running tests

```bash
./test.sh                        # builds then runs ninja test
ninja -C build/ test             # run tests directly after build
```

Integration test scripts are in `scripts/` (e.g., `testModelCounter.sh`, `testCompiler.sh`). These rely on the compiled binary from a demo.

## Architecture

### Library core (`src/`)

The library is templated on a **semiring** — the same DPLL search engine can count models, build decision-DNNFs, or compute weighted counts by swapping the semiring.

| Directory | Role |
|---|---|
| `src/methods/` | Top-level algorithms: `DpllStyleMethod<T,O>`, `MaxSharpSAT`, `MinSharpSAT`, `QbfCounter`, `ExistRandomExist` |
| `src/formulaManager/` | In-memory formula representation for CNF (`CnfManagerDyn`) and circuits (`CircuitManager`) |
| `src/problem/` | `ProblemManager` and `ProblemTypes` — input type enum (`PB_CNF`, `PB_CIRC`, `PB_QBF`) |
| `src/caching/` | Bucket-based component cache; cleaning strategies in `cleaning/` |
| `src/heuristics/` | Branching (`BranchingHeuristic`), phase selection, variable scoring (VSIDS, VSADS, DLCS, JWTS, MOM), partial order |
| `src/solvers/` | Wrappers around Glucose and MiniSAT; `WrapperSolver` is the common interface |
| `src/options/` | Hierarchical options system (`Option<T>` → `OptionGroup` → `OptionRoot` → `OptionDpllStyleMethod`) |
| `src/partitioner/` | Hypergraph partitioning interface (delegates to PaToH on Linux) |
| `src/treeDecompositioner/` | Tree decomposition via FlowCutter |
| `src/representation/` | Graph/hypergraph extractors used by partitioner and tree decomposer |
| `src/preprocs/` | BiPe preprocessor integration |
| `src/utils/` | Buffer reading, equivalence extraction, AT-MOST-1 detection |

### Central algorithm: `DpllStyleMethod<T, O>`

`DpllStyleMethod` (`src/methods/DpllStyleMethod.hpp`) inherits both `MethodManager` and `Counter<T>`. It implements a DPLL-style component search parameterized by:
- `T` — the value type (e.g., `mpz::mpz_int`, `double`, `std::complex<double>`)
- `O` — a semiring policy satisfying the `SemiringPolicy<T>` concept (`src/methods/SemiringConcept.hpp`)

The semiring must provide: `add`, `mul`, `zero`, `one`, `presetSum`, `presetMul`.

### Semiring implementations (`c++/semirings/`)

Shared across demos, not part of the library itself:
- `MpzIntSemiring` — exact integer model counting
- `MpzFloatSemiring` — floating-point weighted counting
- `MpzComplexSemiring` — complex-weight counting
- `DecDNNFSemiring` — builds a decision-DNNF node structure

### Options system

Options are declared as `Option<T>` members of `OptionGroup` subclasses. `OptionDpllStyleMethod` composes sub-groups: `OptionCacheManager`, `OptionSolver`, `OptionBranchingHeuristic`, `OptionSpecManager`. Each demo defines its own extra option group (e.g., `OptionCompiler` adds `--dump-file` and `--query-file`).

### Parser (`c++/parser/`)

`ParserDimacs` is shared by all demos. Produces a `parser::Formula` containing variables, clauses, and literal weights. The compiler and counter demos include parser sources directly via CMake glob.

### Third-party bundled libraries (`3rdParty/`)

- `glucose-3.0` — primary SAT solver (compiled as an OBJECT library and merged into `libd4.a`)
- `flowCutter` — tree decomposition (same merging strategy)
- `patoh` — hypergraph partitioner (Linux only, linked as pre-built `.a`)
- `cadical` — alternative SAT solver (available but optional)

## Input formats

- **CNF**: standard DIMACS (`.cnf`); weighted extensions follow MC 2024 competition format
- **Circuit**: custom BC-S1.2 format; gates declared with `G name := A/O/I literals`, inputs with `I name`, required-true literals with `T literal`
- **QBF**: quantified DIMACS

## Key conventions

- All library code is in the `d4` namespace; semiring implementations use `semiring` namespace.
- `BUILD_MODE` CMake variable controls optimization: 0=release, 1=debug, 2=static, 3=profiling.
- On Linux, `USE_PATOH` is defined automatically enabling hypergraph partitioning; on other platforms it is disabled silently.
- Demo `build.sh` scripts propagate all flags (e.g., `-s -j`) to the parent `libd4.a` build before building the demo executable.
