/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include "Counter.hpp"

#include <gmpxx.h>
#include <signal.h>

#include <cassert>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>

#include "../semirings/MpzComplexSemiring.hpp"
#include "../semirings/MpzFloatSemiring.hpp"
#include "../semirings/MpzIntSemiring.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

extern d4::MethodManager* methodRun;

using namespace d4;

namespace {

// log10(|x|) as a double, robust to arbitrarily large/small magnitudes: GMP
// has no native log, so we decompose x = d * 2^e (0.5 <= |d| < 1) and compute
// log10(|x|) = log10(|d|) + e * log10(2). This keeps double precision (the ~15
// significant digits the competition format expects for the estimate) even
// when x is far beyond the range of a double.
inline double log10Abs(const mpz::mpz_int& x) {
  signed long int e;
  double d = mpz_get_d_2exp(&e, x.get_mpz_t());
  if (d == 0.0) return -std::numeric_limits<double>::infinity();
  return std::log10(std::fabs(d)) + e * std::log10(2.0);
}

inline double log10Abs(const mpz::mpf_float& x) {
  signed long int e;
  double d = mpf_get_d_2exp(&e, x.get_mpf_t());
  if (d == 0.0) return -std::numeric_limits<double>::infinity();
  return std::log10(std::fabs(d)) + e * std::log10(2.0);
}

// Emit a mandatory "c s [neg]log10-estimate<suffix> VALUE" line for a real
// quantity, following the competition sign rules: VALUE := log10(|cnt|);
// cnt > 0 -> log10-estimate, cnt < 0 -> neglog10-estimate, cnt = 0 -> -inf.
template <typename R>
inline void emitLog10Estimate(std::ostream& out, const R& v,
                              const std::string& suffix) {
  int s = sgn(v);
  if (s == 0)
    out << "c s log10-estimate" << suffix << " -inf\n";
  else if (s > 0)
    out << "c s log10-estimate" << suffix << " " << log10Abs(v) << "\n";
  else
    out << "c s neglog10-estimate" << suffix << " " << log10Abs(v) << "\n";
}

// Render a complex value as "a+bi" / "a-bi" with no spaces, as required by the
// algebraic-model-counting track.
inline std::string complexToString(const semiring::Complex& c) {
  std::ostringstream os;
  os.precision(50);
  os << c.real.get_d();
  if (sgn(c.im) < 0)
    os << c.im.get_d() << "i";  // c.im already carries its leading '-'
  else
    os << "+" << c.im.get_d() << "i";
  return os.str();
}

}  // namespace

template <typename T, typename O>
void countModels(const OptionDpllStyleMethod& options,
                 const ProblemManager& problem, const std::string& format,
                 const std::string& outFormat, bool isFloat) {
  std::cout << "c [FORMAT] Input/Output format:" << " output-symbol(" << format
            << ")" << " output-format(" << outFormat << ")" << " is-float("
            << isFloat << ")\n";

  auto counter = new DpllStyleMethod<T, O>(options, problem, std::cout);
  T result = counter->run();

  if (outFormat == "competition") {
    mpf_set_default_prec(426);  // ~128 decimal digits
    std::cout.precision(50);
    std::cout << (counter->isProblemUnsat() ? "s UNSATISFIABLE\n"
                                            : "s SATISFIABLE\n");
    std::cout << "c s type " << format << "\n";

    if constexpr (std::is_same_v<T, semiring::Complex>) {
      // Algebraic counting: announce the log10 estimate of the real and the
      // imaginary parts separately, then the exact result as "a+bi".
      emitLog10Estimate(std::cout, result.real, "-real");
      emitLog10Estimate(std::cout, result.im, "-imag");
      std::cout << "c s exact arb float " << complexToString(result) << "\n";
    } else {
      emitLog10Estimate(std::cout, result, "");
      if constexpr (std::is_same_v<T, mpz::mpf_float>)
        std::cout << "c s exact arb float " << result.get_d() << "\n";
      else
        std::cout << "c s exact arb int " << result.get_str() << "\n";
    }
    exit(0);  // stop faster than cleaning the memory!
  } else {
    assert(outFormat == "classic");
    mpf_set_default_prec(426);  // ~128 decimal digits
    std::cout.precision(50);
    if constexpr (std::is_same_v<T, mpz::mpf_float>)
        std::cout << "s " << result.get_d() << "\n";
    else if constexpr (std::is_same_v<T, semiring::Complex>)
        std::cout << "s " << result << "\n";
    else
        std::cout << "s " << result.get_str() << "\n";
  }

  delete counter;
}  // count

/**
 * @brief counterDemo implementation.
 *
 * Runs the counter using the provided configuration.
 */
void counter(const d4::OptionDpllStyleMethod& inputConfig,
             const d4::OptionCounter& optionCounter,
             const parser::Formula& formula) {
  // Use the provided configuration.
  d4::OptionDpllStyleMethod options = inputConfig;

  // Force counting operation.
  options.operationType = d4::OperationTypeManager::getOperatorType("counting");

  if (options.optionCacheManager.optionBucketManager.clauseRepresentation ==
      CACHE_INDEX)
    options.optionSpecManager.needFastNotSatisfied = true;

  const std::string format = optionCounter.format.get();
  const std::string outFormat = optionCounter.outFormat.get();

  // Build the problem from the parsed formula.
  std::vector<d4::BcGate> gates;
  if (formula.type == "circuit") {
    gates.reserve(formula.gates.size());
    for (auto& g : formula.gates) {
      d4::BcGateType t;
      switch (g.gateType) {
        case parser::GateType::AND:
          t = d4::BcGateType::AND;
          break;
        case parser::GateType::OR:
          t = d4::BcGateType::OR;
          break;
        case parser::GateType::IDENTITY:
          t = d4::BcGateType::IDENTITY;
          break;
        case parser::GateType::CLAUSE:
          t = d4::BcGateType::CLAUSE;
          break;
        default:
          std::cerr << "ERROR: unsupported gate type for counting\n";
          exit(1);
      }
      std::vector<d4::Lit> lits;
      lits.reserve(g.inputs.size());
      for (int l : g.inputs)
        lits.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
      d4::Lit out = (g.output == 0)
                        ? d4::lit_Undef
                        : d4::Lit::makeLit(std::abs(g.output), g.output < 0);
      gates.push_back({lits, out, t});
    }
  } else {
    gates.reserve(formula.clauses.size());
    for (auto& cl : formula.clauses) {
      std::vector<d4::Lit> d4Clause;
      for (auto& l : cl)
        d4Clause.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
      gates.push_back({d4Clause, d4::lit_Undef, BcGateType::CLAUSE});
    }
  }

  std::map<d4::Lit, std::string> weightMap;
  for (const auto& [lit, weight] : formula.weightMap)
    weightMap[d4::Lit::makeLit(std::abs(lit), lit < 0)] = weight;

  d4::ProblemManager problem(formula.type, formula.nbVar,
                             formula.quantifications, weightMap, gates,
                             std::cout);

  switch (formula.weightType) {
    case parser::WeightType::INT:
      countModels<mpz::mpz_int, semiring::MpzIntSemiring>(
          options, problem, formula.typeCompet, outFormat, false);
      break;
    case parser::WeightType::FLOAT:
      countModels<mpz::mpf_float, semiring::MpzFloatSemiring>(
          options, problem, formula.typeCompet, outFormat, true);
      break;
    case parser::WeightType::COMPLEX:
      countModels<semiring::Complex, semiring::MpzComplexSemiring>(
          options, problem, formula.typeCompet, outFormat, true);
      break;
  }
}  // counter