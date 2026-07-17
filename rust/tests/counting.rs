use d4::{Problem, WeightType};

#[test]
fn count_single_clause() {
    // (1 ∨ 2 ∨ 3) over 3 vars: 2^3 - 1 = 7 models.
    let mut p = Problem::new(3);
    p.add_clause(&[1, 2, 3]).unwrap();
    assert_eq!(p.count().unwrap(), "7");
}

#[test]
fn count_free_variables() {
    // Unit clause (1) over 3 vars: vars 2 and 3 are free -> 4 models.
    let mut p = Problem::new(3);
    p.add_clause(&[1]).unwrap();
    assert_eq!(p.count().unwrap(), "4");
}

#[test]
fn count_unsat() {
    let mut p = Problem::new(2);
    p.add_clause(&[1]).unwrap();
    p.add_clause(&[-1]).unwrap();
    assert_eq!(p.count().unwrap(), "0");
}

#[test]
fn count_projected() {
    // (1 ∨ 2) projected on {1}: both values of 1 extend to a model -> 2.
    let mut p = Problem::new(2);
    p.add_clause(&[1, 2]).unwrap();
    p.set_projected_vars(&[1]).unwrap();
    assert_eq!(p.count().unwrap(), "2");
}

#[test]
fn count_from_dimacs_string() {
    let p = Problem::from_dimacs("p cnf 3 1\n1 2 3 0\n").unwrap();
    assert_eq!(p.nb_var(), 3);
    assert_eq!(p.weight_type(), WeightType::Int);
    assert_eq!(p.count().unwrap(), "7");
}

#[test]
fn count_weighted() {
    // Two independent vars with weights 0.5/0.5 and 0.25/0.25:
    // total weight = (0.5 + 0.5) * (0.25 + 0.25) = 0.5.
    let mut p = Problem::new(2);
    p.add_clause(&[1, -1]).unwrap();
    p.add_clause(&[2, -2]).unwrap();
    p.set_lit_weight(1, "0.5").unwrap();
    p.set_lit_weight(-1, "0.5").unwrap();
    p.set_lit_weight(2, "0.25").unwrap();
    p.set_lit_weight(-2, "0.25").unwrap();
    assert_eq!(p.weight_type(), WeightType::Float);
    let w: f64 = p.count_weighted().unwrap().parse().unwrap();
    assert!((w - 0.5).abs() < 1e-9);
}

#[test]
fn count_bigger_instance() {
    // 20 unconstrained vars plus one tautological clause: 2^20 models.
    let mut p = Problem::new(20);
    p.add_clause(&[1, -1]).unwrap();
    assert_eq!(p.count().unwrap(), (1u64 << 20).to_string());
}

#[test]
fn invalid_inputs_are_rejected() {
    let mut p = Problem::new(2);
    assert!(p.add_clause(&[0]).is_err());
    assert!(p.add_clause(&[3]).is_err());
    assert!(p.add_clause(&[]).is_err());
    assert!(p.set_projected_vars(&[5]).is_err());
    assert!(p.set_lit_weight(1, "not-a-number").is_err());
    assert!(Problem::from_dimacs_file("/no/such/file.cnf").is_err());
}
