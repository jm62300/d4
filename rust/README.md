# d4 Rust binding

Rust bindings for the counting part of **d4** (exact, projected and weighted
model counting). Other parts of the library (compilation, Max#SAT, QBF) will
be exposed later through the same C shim (`shim/d4_shim.{h,cpp}`).

## Layout

- `shim/` — C API over the C++ core (`ProblemManager` + `DpllStyleMethod`),
  mirroring the wiring of `c++/counter`. No preprocessing (bipe/arjun) is
  involved; the raw formula is counted as given.
- `src/ffi.rs` — raw `extern "C"` declarations.
- `src/lib.rs` — safe API (`Problem`).

## Building

```sh
cd rust
cargo build --release
cargo test --release
cargo run --release --example count -- ../instancesTest/cnfs/cnf10.cnf
```

`build.rs` looks for `../build/libd4.a` and runs the repo's `./build.sh -j`
automatically if it is missing. Set `D4_LIB_DIR` to link against a `libd4.a`
located elsewhere. System dependencies are the same as for the library:
`libgmp`, `libgmpxx`, `libz` (and the bundled PaToH on Linux).

The DPLL search engine is header-template C++ instantiated inside the shim,
so the shim is always compiled with `-O3` even for debug Rust builds.

## Usage

```rust
use d4::Problem;

// #SAT
let mut p = Problem::new(3);
p.add_clause(&[1, 2, 3])?;         // DIMACS literals, no trailing 0
println!("{}", p.count()?);        // "7" (arbitrary-precision decimal string)

// Projected counting
p.set_projected_vars(&[1, 2])?;

// Weighted counting (unset literals keep weight 1)
p.set_lit_weight(1, "0.3")?;
p.set_lit_weight(-1, "0.7")?;
println!("{}", p.count_weighted()?);

// From a DIMACS file or string (weighted / projected extensions supported)
let p = Problem::from_dimacs_file("formula.cnf")?;
let p = Problem::from_dimacs("p cnf 2 1\n1 2 0\n")?;
```

Counts are returned as decimal strings since they routinely exceed any
machine integer; parse them with a bignum crate if arithmetic is needed.

`Problem` holds a raw pointer and is deliberately neither `Send` nor `Sync`.
