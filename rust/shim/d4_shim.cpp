/*
 * d4 — C API shim implementation for the Rust binding.
 *
 * Follows the wiring of c++/counter/src/Counter.cpp: build a
 * d4::ProblemManager from the clauses/weights/projection, then run
 * d4::DpllStyleMethod with the semiring matching the requested count.
 * No exception may cross the C boundary: every entry point catches and
 * stores the message in a thread-local slot exposed by d4_last_error().
 */
#include "d4_shim.h"

#include <gmp.h>
#include <gmpxx.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ParserDimacs.hpp"
#include "semirings/MpzFloatSemiring.hpp"
#include "semirings/MpzIntSemiring.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/problem/ProblemManager.hpp"

namespace {

namespace mpz = d4MpzTypes;

thread_local std::string g_lastError;

void setError(const std::string& msg) { g_lastError = msg; }

// Swallows library logging when verbosity is 0.
struct NullBuffer : std::streambuf {
  int overflow(int c) override { return c; }
};

char* dupString(const std::string& s) {
  char* out = static_cast<char*>(std::malloc(s.size() + 1));
  if (out) std::memcpy(out, s.c_str(), s.size() + 1);
  return out;
}

}  // namespace

struct D4Problem {
  unsigned nbVar = 0;
  std::vector<std::vector<int>> clauses;
  std::vector<int> projected;
  std::map<int, std::string> weights;
  int weightType = D4_WEIGHT_INT;
};

namespace {

D4Problem* fromFormula(const parser::Formula& formula) {
  // maxVars/indVars blocks belong to Max#SAT; the counting API handles a
  // single (possibly empty) projection block only.
  if (formula.quantifications.size() != 1)
    throw std::runtime_error(
        "input declares max/ind quantifier blocks, which the counting API "
        "does not support");

  auto p = std::make_unique<D4Problem>();
  p->nbVar = formula.nbVar;
  p->clauses = formula.clauses;
  p->projected = formula.quantifications[0];
  p->weights = formula.weightMap;
  switch (formula.weightType) {
    case parser::WeightType::INT:
      p->weightType = D4_WEIGHT_INT;
      break;
    case parser::WeightType::FLOAT:
      p->weightType = D4_WEIGHT_FLOAT;
      break;
    case parser::WeightType::COMPLEX:
      p->weightType = D4_WEIGHT_COMPLEX;
      break;
  }
  return p.release();
}

template <typename T, typename O>
std::string runCount(const D4Problem& p, int verbosity) {
  d4::OptionDpllStyleMethod options;
  options.verbosity.set(verbosity);
  options.operationType.set(d4::OP_COUNTING);
  if (options.optionCacheManager.optionBucketManager.clauseRepresentation ==
      d4::CACHE_INDEX)
    options.optionSpecManager.needFastNotSatisfied = true;
  if (!p.projected.empty())
    options.optionSpecManager.specUpdateType = d4::SPEC_DYNAMIC_BLOCKED_SIMP;

  std::vector<d4::BcGate> gates;
  gates.reserve(p.clauses.size());
  for (const auto& cl : p.clauses) {
    std::vector<d4::Lit> lits;
    lits.reserve(cl.size());
    for (int l : cl) lits.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
    gates.push_back({lits, d4::lit_Undef, d4::BcGateType::CLAUSE});
  }

  std::map<d4::Lit, std::string> weightMap;
  for (const auto& [lit, w] : p.weights)
    weightMap[d4::Lit::makeLit(std::abs(lit), lit < 0)] = w;

  std::vector<std::vector<d4::Var>> quantifications(1);
  quantifications[0].assign(p.projected.begin(), p.projected.end());

  NullBuffer nullBuffer;
  std::ostream nullStream(&nullBuffer);
  std::ostream& out = verbosity > 0 ? std::cout : nullStream;

  // A few spots in the library log straight to std::cout instead of the
  // stream they were given (e.g. CnfManagerDynBlockedCl); mute those too
  // when the caller asked for quiet. Restored on scope exit.
  struct CoutSilencer {
    std::streambuf* saved = nullptr;
    ~CoutSilencer() {
      if (saved) std::cout.rdbuf(saved);
    }
  } silencer;
  if (verbosity <= 0) silencer.saved = std::cout.rdbuf(&nullBuffer);

  d4::ProblemManager problem("cnf", p.nbVar, quantifications, weightMap, gates,
                             out);

  d4::DpllStyleMethod<T, O> counter(options, problem, out);
  T result = counter.run();

  std::ostringstream os;
  os.precision(50);
  os << result;
  return os.str();
}

int checkLits(const D4Problem* p, const int* lits, size_t len,
              const char* what) {
  for (size_t i = 0; i < len; i++) {
    int l = lits[i];
    if (l == 0) {
      setError(std::string(what) + ": literal 0 is not allowed");
      return -1;
    }
    if (static_cast<unsigned>(std::abs(l)) > p->nbVar) {
      setError(std::string(what) + ": variable " +
               std::to_string(std::abs(l)) + " is out of range (nbVar = " +
               std::to_string(p->nbVar) + ")");
      return -1;
    }
  }
  return 0;
}

}  // namespace

extern "C" {

D4Problem* d4_problem_new(unsigned nb_var) {
  g_lastError.clear();
  try {
    auto* p = new D4Problem();
    p->nbVar = nb_var;
    return p;
  } catch (const std::exception& e) {
    setError(e.what());
    return nullptr;
  }
}

void d4_problem_free(D4Problem* p) { delete p; }

D4Problem* d4_problem_from_dimacs_file(const char* path) {
  g_lastError.clear();
  try {
    if (!path) {
      setError("path is null");
      return nullptr;
    }
    if (!std::ifstream(path).good()) {
      setError(std::string("cannot open file: ") + path);
      return nullptr;
    }
    parser::Formula formula;
    parser::ParserDimacs parser;
    parser.parse_DIMACS(std::string(path), formula);
    return fromFormula(formula);
  } catch (const std::exception& e) {
    setError(e.what());
    return nullptr;
  } catch (...) {
    setError("unknown error while parsing DIMACS file");
    return nullptr;
  }
}

D4Problem* d4_problem_from_dimacs(const char* data, size_t len) {
  g_lastError.clear();
  try {
    if (!data) {
      setError("data is null");
      return nullptr;
    }
    parser::Formula formula;
    parser::ParserDimacs parser;
    parser.parse_DIMACS(data, len, formula);
    return fromFormula(formula);
  } catch (const std::exception& e) {
    setError(e.what());
    return nullptr;
  } catch (...) {
    setError("unknown error while parsing DIMACS data");
    return nullptr;
  }
}

int d4_problem_add_clause(D4Problem* p, const int* lits, size_t len) {
  g_lastError.clear();
  try {
    if (!p || (!lits && len > 0)) {
      setError("add_clause: null argument");
      return -1;
    }
    if (len == 0) {
      setError("add_clause: empty clause is not allowed");
      return -1;
    }
    if (checkLits(p, lits, len, "add_clause")) return -1;
    p->clauses.emplace_back(lits, lits + len);
    return 0;
  } catch (const std::exception& e) {
    setError(e.what());
    return -1;
  }
}

int d4_problem_set_projected_vars(D4Problem* p, const int* vars, size_t len) {
  g_lastError.clear();
  try {
    if (!p || (!vars && len > 0)) {
      setError("set_projected_vars: null argument");
      return -1;
    }
    for (size_t i = 0; i < len; i++)
      if (vars[i] <= 0 || static_cast<unsigned>(vars[i]) > p->nbVar) {
        setError("set_projected_vars: variable " + std::to_string(vars[i]) +
                 " is out of range (nbVar = " + std::to_string(p->nbVar) +
                 ")");
        return -1;
      }
    p->projected.assign(vars, vars + len);
    return 0;
  } catch (const std::exception& e) {
    setError(e.what());
    return -1;
  }
}

int d4_problem_set_lit_weight(D4Problem* p, int lit, const char* weight) {
  g_lastError.clear();
  try {
    if (!p || !weight) {
      setError("set_lit_weight: null argument");
      return -1;
    }
    if (checkLits(p, &lit, 1, "set_lit_weight")) return -1;
    // Validate eagerly: mpf_class throws std::invalid_argument on garbage,
    // better here than in the middle of a count.
    mpf_class check(weight);
    (void)check;
    p->weights[lit] = weight;
    if (p->weightType == D4_WEIGHT_INT) p->weightType = D4_WEIGHT_FLOAT;
    return 0;
  } catch (const std::exception& e) {
    setError(std::string("set_lit_weight: invalid weight: ") + e.what());
    return -1;
  }
}

unsigned d4_problem_nb_var(const D4Problem* p) { return p ? p->nbVar : 0; }

int d4_problem_weight_type(const D4Problem* p) {
  return p ? p->weightType : D4_WEIGHT_INT;
}

char* d4_count(const D4Problem* p, int verbosity) {
  g_lastError.clear();
  try {
    if (!p) {
      setError("count: problem is null");
      return nullptr;
    }
    return dupString(
        runCount<mpz::mpz_int, semiring::MpzIntSemiring>(*p, verbosity));
  } catch (const std::exception& e) {
    setError(e.what());
    return nullptr;
  } catch (...) {
    setError("unknown error during counting");
    return nullptr;
  }
}

char* d4_count_weighted(const D4Problem* p, int verbosity) {
  g_lastError.clear();
  try {
    if (!p) {
      setError("count_weighted: problem is null");
      return nullptr;
    }
    if (p->weightType == D4_WEIGHT_COMPLEX) {
      setError("count_weighted: complex weights are not supported yet");
      return nullptr;
    }
    mpf_set_default_prec(426);  // ~128 decimal digits, as in the counter demo
    return dupString(
        runCount<mpz::mpf_float, semiring::MpzFloatSemiring>(*p, verbosity));
  } catch (const std::exception& e) {
    setError(e.what());
    return nullptr;
  } catch (...) {
    setError("unknown error during weighted counting");
    return nullptr;
  }
}

void d4_free_string(char* s) { std::free(s); }

const char* d4_last_error(void) {
  return g_lastError.empty() ? nullptr : g_lastError.c_str();
}

}  // extern "C"
