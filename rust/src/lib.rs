//! Safe Rust bindings for the counting part of the d4 library.
//!
//! d4 solves problems beyond NP; this crate currently exposes exact model
//! counting (standard, projected, weighted) over CNF formulas. Counts are
//! arbitrary precision and returned as decimal strings.
//!
//! ```no_run
//! use d4::Problem;
//!
//! // #SAT over 3 variables with the single clause (1 ∨ 2 ∨ 3).
//! let mut p = Problem::new(3);
//! p.add_clause(&[1, 2, 3]).unwrap();
//! assert_eq!(p.count().unwrap(), "7");
//! ```

mod ffi;

use std::ffi::{CStr, CString};
use std::fmt;
use std::os::raw::c_char;
use std::path::Path;
use std::sync::{Mutex, MutexGuard};

// The search engine keeps global state (solver statics, GMP default
// precision, ...) and crashes when two counts run concurrently, so counting
// is serialized process-wide.
static COUNT_LOCK: Mutex<()> = Mutex::new(());

fn count_lock() -> MutexGuard<'static, ()> {
    COUNT_LOCK.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Error raised by the underlying d4 library or by input validation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error(String);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "d4: {}", self.0)
    }
}

impl std::error::Error for Error {}

fn last_error() -> Error {
    unsafe {
        let msg = ffi::d4_last_error();
        if msg.is_null() {
            Error("unknown error".into())
        } else {
            Error(CStr::from_ptr(msg).to_string_lossy().into_owned())
        }
    }
}

fn take_string(ptr: *mut c_char) -> Result<String, Error> {
    if ptr.is_null() {
        return Err(last_error());
    }
    let s = unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned();
    unsafe { ffi::d4_free_string(ptr) };
    Ok(s)
}

/// Kind of literal weights attached to a problem (DIMACS `c p weight` lines
/// or [`Problem::set_lit_weight`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WeightType {
    Int,
    Float,
    Complex,
}

/// A CNF problem to count models of.
///
/// Variables are numbered `1..=nb_var`; literals follow the DIMACS
/// convention (`v` positive, `-v` negative).
pub struct Problem {
    ptr: *mut ffi::D4Problem,
    verbosity: i32,
}

impl Problem {
    /// Creates an empty problem over `nb_var` variables.
    pub fn new(nb_var: u32) -> Self {
        let ptr = unsafe { ffi::d4_problem_new(nb_var) };
        assert!(!ptr.is_null(), "d4_problem_new failed: {}", last_error());
        Problem { ptr, verbosity: 0 }
    }

    /// Parses a DIMACS CNF file (weighted/projected extensions of the MC
    /// competition format are supported).
    pub fn from_dimacs_file(path: impl AsRef<Path>) -> Result<Self, Error> {
        let path = path.as_ref();
        let cpath = CString::new(path.to_str().ok_or_else(|| {
            Error(format!("path is not valid UTF-8: {}", path.display()))
        })?)
        .map_err(|_| Error("path contains a NUL byte".into()))?;
        let ptr = unsafe { ffi::d4_problem_from_dimacs_file(cpath.as_ptr()) };
        if ptr.is_null() {
            Err(last_error())
        } else {
            Ok(Problem { ptr, verbosity: 0 })
        }
    }

    /// Parses a DIMACS CNF held in memory.
    pub fn from_dimacs(data: impl AsRef<[u8]>) -> Result<Self, Error> {
        let data = data.as_ref();
        let ptr =
            unsafe { ffi::d4_problem_from_dimacs(data.as_ptr() as *const c_char, data.len()) };
        if ptr.is_null() {
            Err(last_error())
        } else {
            Ok(Problem { ptr, verbosity: 0 })
        }
    }

    /// Adds a clause given as non-zero DIMACS literals (no trailing 0).
    pub fn add_clause(&mut self, lits: &[i32]) -> Result<(), Error> {
        let rc = unsafe { ffi::d4_problem_add_clause(self.ptr, lits.as_ptr(), lits.len()) };
        if rc == 0 { Ok(()) } else { Err(last_error()) }
    }

    /// Restricts counting to the given variables (projected model counting).
    /// An empty slice resets to standard counting over all variables.
    pub fn set_projected_vars(&mut self, vars: &[i32]) -> Result<(), Error> {
        let rc =
            unsafe { ffi::d4_problem_set_projected_vars(self.ptr, vars.as_ptr(), vars.len()) };
        if rc == 0 { Ok(()) } else { Err(last_error()) }
    }

    /// Attaches a weight (decimal string such as `"0.25"`) to a literal and
    /// switches the problem to weighted mode. Unset literals keep weight 1.
    pub fn set_lit_weight(&mut self, lit: i32, weight: &str) -> Result<(), Error> {
        let cweight =
            CString::new(weight).map_err(|_| Error("weight contains a NUL byte".into()))?;
        let rc = unsafe { ffi::d4_problem_set_lit_weight(self.ptr, lit, cweight.as_ptr()) };
        if rc == 0 { Ok(()) } else { Err(last_error()) }
    }

    /// Number of variables of the problem.
    pub fn nb_var(&self) -> u32 {
        unsafe { ffi::d4_problem_nb_var(self.ptr) }
    }

    /// Kind of weights attached to the problem.
    pub fn weight_type(&self) -> WeightType {
        match unsafe { ffi::d4_problem_weight_type(self.ptr) } {
            ffi::D4_WEIGHT_FLOAT => WeightType::Float,
            ffi::D4_WEIGHT_COMPLEX => WeightType::Complex,
            _ => WeightType::Int,
        }
    }

    /// Verbosity of the underlying solver (0 = silent, higher = more logs on
    /// stdout).
    pub fn set_verbosity(&mut self, level: i32) {
        self.verbosity = level;
    }

    /// Exact (projected) model count, ignoring weights. The result is an
    /// arbitrary-precision integer rendered in decimal ("0" when UNSAT).
    ///
    /// Counting is serialized process-wide: concurrent calls from other
    /// threads wait their turn.
    pub fn count(&self) -> Result<String, Error> {
        let _guard = count_lock();
        take_string(unsafe { ffi::d4_count(self.ptr, self.verbosity) })
    }

    /// Weighted model count using the literal weights (~128 decimal digits of
    /// working precision, rendered with up to 50 significant digits).
    ///
    /// Counting is serialized process-wide: concurrent calls from other
    /// threads wait their turn.
    pub fn count_weighted(&self) -> Result<String, Error> {
        let _guard = count_lock();
        take_string(unsafe { ffi::d4_count_weighted(self.ptr, self.verbosity) })
    }
}

impl Drop for Problem {
    fn drop(&mut self) {
        unsafe { ffi::d4_problem_free(self.ptr) };
    }
}
