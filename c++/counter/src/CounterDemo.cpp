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

#include "CounterDemo.hpp"

#include <signal.h>

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/integer.hpp>
#include <cassert>
#include <iomanip>

#include "../semirings/MpzComplexSemiring.hpp"
#include "../semirings/MpzFloatSemiring.hpp"
#include "../semirings/MpzIntSemiring.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

extern d4::MethodManager* methodRun;

using namespace d4;

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
    boost::multiprecision::mpf_float::default_precision(128);
    std::cout.precision(
        std::numeric_limits<boost::multiprecision::cpp_dec_float_50>::digits10);

    if (result == T(0)) {
      std::cout << "s UNSATISFIABLE\n";
      std::cout << "c " << format << "\n";
      std::cout << "c s log10-estimate -inf\n";
      std::cout << "c s exact quadruple int 0\n";
    } else {
      std::cout << "s SATISFIABLE\n";
      std::cout << "c " << format << "\n";
      std::cout << "c s log10-estimate " << result << "\n";

      if (isFloat)
        std::cout << "c s exact quadruple int " << result << "\n";
      else
        std::cout << "c s exact arb int " << result << "\n";
    }
    exit(0);  // stop faster than cleaning the memory!
  } else {
    assert(outFormat == "classic");
    boost::multiprecision::mpf_float::default_precision(128);
    std::cout.precision(
        std::numeric_limits<boost::multiprecision::cpp_dec_float_50>::digits10);

    std::cout << format << " ";
    std::cout << result << "\n";
  }

  delete counter;
}  // count

/**
 * @brief counterDemo implementation.
 *
 * Runs the counter using the provided configuration.
 */
void counterDemo(const d4::OptionDpllStyleMethod& inputConfig,
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
  gates.reserve(formula.clauses.size());
  for (auto& cl : formula.clauses) {
    std::vector<d4::Lit> d4Clause;
    for (auto& l : cl) d4Clause.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
    gates.push_back({d4Clause, d4::lit_Undef, BcGateType::CLAUSE});
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
          options, problem, format, outFormat, false);
      break;
    case parser::WeightType::FLOAT:
      countModels<mpz::mpf_float, semiring::MpzFloatSemiring>(
          options, problem, format, outFormat, false);
      break;
    case parser::WeightType::COMPLEX:
      countModels<semiring::Complex, semiring::MpzComplexSemiring>(
          options, problem, format, outFormat, false);
      break;
  }
}  // counterDemo