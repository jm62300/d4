//! Raw FFI declarations matching `shim/d4_shim.h`.

use std::os::raw::{c_char, c_int, c_uint};

#[repr(C)]
pub struct D4Problem {
    _private: [u8; 0],
}

#[allow(dead_code)] // matched via the catch-all arm in WeightType conversion
pub const D4_WEIGHT_INT: c_int = 0;
pub const D4_WEIGHT_FLOAT: c_int = 1;
pub const D4_WEIGHT_COMPLEX: c_int = 2;

extern "C" {
    pub fn d4_problem_new(nb_var: c_uint) -> *mut D4Problem;
    pub fn d4_problem_free(p: *mut D4Problem);

    pub fn d4_problem_from_dimacs_file(path: *const c_char) -> *mut D4Problem;
    pub fn d4_problem_from_dimacs(data: *const c_char, len: usize) -> *mut D4Problem;

    pub fn d4_problem_add_clause(p: *mut D4Problem, lits: *const c_int, len: usize) -> c_int;
    pub fn d4_problem_set_projected_vars(
        p: *mut D4Problem,
        vars: *const c_int,
        len: usize,
    ) -> c_int;
    pub fn d4_problem_set_lit_weight(p: *mut D4Problem, lit: c_int, weight: *const c_char)
        -> c_int;

    pub fn d4_problem_nb_var(p: *const D4Problem) -> c_uint;
    pub fn d4_problem_weight_type(p: *const D4Problem) -> c_int;

    pub fn d4_count(p: *const D4Problem, verbosity: c_int) -> *mut c_char;
    pub fn d4_count_weighted(p: *const D4Problem, verbosity: c_int) -> *mut c_char;
    pub fn d4_free_string(s: *mut c_char);

    pub fn d4_last_error() -> *const c_char;
}
