//! Count the models of a DIMACS CNF file:
//!
//! ```sh
//! cargo run --release --example count -- ../instancesTest/cnfs/cnf10.cnf
//! ```

use d4::{Problem, WeightType};

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: count <file.cnf> [-v]");
    let verbose = std::env::args().any(|a| a == "-v");

    let mut problem = Problem::from_dimacs_file(&path).expect("parsing failed");
    if verbose {
        problem.set_verbosity(1);
    }

    let count = match problem.weight_type() {
        WeightType::Int => problem.count(),
        _ => problem.count_weighted(),
    }
    .expect("counting failed");

    println!("s {count}");
}
