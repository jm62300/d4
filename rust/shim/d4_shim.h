/*
 * d4 — C API shim for the Rust binding.
 *
 * Covers the counting part of the library: exact (projected) model counting
 * and weighted model counting. Other methods (compilation, Max#SAT, QBF)
 * will be added here later.
 *
 * Conventions:
 *   - functions returning int: 0 on success, -1 on error;
 *   - functions returning a pointer: NULL on error;
 *   - on error, d4_last_error() returns a thread-local message.
 */
#ifndef D4_SHIM_H
#define D4_SHIM_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct D4Problem D4Problem;

/* Mirrors parser::WeightType. */
#define D4_WEIGHT_INT 0
#define D4_WEIGHT_FLOAT 1
#define D4_WEIGHT_COMPLEX 2

/* Create an empty CNF problem over nb_var variables (numbered 1..nb_var). */
D4Problem* d4_problem_new(unsigned nb_var);
void d4_problem_free(D4Problem* p);

/* Parse a DIMACS CNF (weighted/projected extensions supported). */
D4Problem* d4_problem_from_dimacs_file(const char* path);
D4Problem* d4_problem_from_dimacs(const char* data, size_t len);

/* Literals use the DIMACS convention: v > 0 positive, -v negative. */
int d4_problem_add_clause(D4Problem* p, const int* lits, size_t len);

/* Restrict counting to these variables (projected model counting).
   Passing len == 0 resets to standard counting over all variables. */
int d4_problem_set_projected_vars(D4Problem* p, const int* vars, size_t len);

/* Attach a weight (decimal string, e.g. "0.25") to a literal; switches the
   problem to weighted mode. */
int d4_problem_set_lit_weight(D4Problem* p, int lit, const char* weight);

unsigned d4_problem_nb_var(const D4Problem* p);
int d4_problem_weight_type(const D4Problem* p);

/* Exact (projected) model count, ignoring weights. Returns the count as a
   malloc'ed decimal string (arbitrary precision). */
char* d4_count(const D4Problem* p, int verbosity);

/* Weighted model count using the literal weights (default weight 1 per
   literal). Returns a malloc'ed decimal string. */
char* d4_count_weighted(const D4Problem* p, int verbosity);

void d4_free_string(char* s);

/* Message of the last error on this thread, or NULL if none. The pointer is
   invalidated by the next d4_* call on the same thread. */
const char* d4_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* D4_SHIM_H */
