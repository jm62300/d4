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

#include "ProjMcCounter.hpp"

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/integer.hpp>
#include <cassert>
#include <iomanip>
#include <limits>

#include "../semirings/MpzFloatSemiring.hpp"
#include "../semirings/MpzIntSemiring.hpp"
#include "src/methods/ProjMCMethod.hpp"

using namespace d4;

template <typename T, typename O>
void runProjMc(const OptionProjMcMethod& options, const ProblemManager& problem,
               const std::string& format, const std::string& outFormat,
               bool isFloat) {
  std::cout << "c [FORMAT] Input/Output format:"
            << " output-symbol(" << format << ")"
            << " output-format(" << outFormat << ")"
            << " is-float(" << isFloat << ")\n";

  auto* counter = new ProjMCMethod<T, O>(options, problem, std::cout);
  T result = counter->run();

  boost::multiprecision::mpf_float::default_precision(128);
  std::cout.precision(
      std::numeric_limits<boost::multiprecision::cpp_dec_float_50>::digits10);

  if (outFormat == "competition") {
    if (result == T(0)) {
      std::cout << "s UNSATISFIABLE\n";
      std::cout << "c " << format << "\n";
      std::cout << "c s log10-estimate -inf\n";
      std::cout << "c s exact quadruple int 0\n";
    } else {
      std::cout << "s SATISFIABLE\n";
      std::cout << "c " << format << "\n";
      std::cout << "c s log10-estimate "
                << boost::multiprecision::log10(
                       boost::multiprecision::cpp_dec_float_100(result))
                << "\n";
      if (isFloat)
        std::cout << "c s exact quadruple int " << result << "\n";
      else
        std::cout << "c s exact arb int " << result << "\n";
    }
  } else {
    assert(outFormat == "classic");
    std::cout << format << " " << result << "\n";
  }

  delete counter;
}  // runProjMc

/**
 * @brief Run the projected model counter on the parsed formula.
 */
void projMcCounter(const d4::OptionProjMcMethod& options,
                   const d4::OptionProjMcCounter& optionCounter,
                   const parser::Formula& formula) {
  // Build the ProblemManager from the parsed formula.
  std::vector<d4::BcGate> gates;
  gates.reserve(formula.clauses.size());
  for (auto& cl : formula.clauses) {
    std::vector<d4::Lit> d4Clause;
    for (auto& l : cl) d4Clause.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
    gates.push_back({d4Clause, d4::lit_Undef, d4::BcGateType::CLAUSE});
  }

  std::map<d4::Lit, std::string> weightMap;
  for (const auto& [lit, weight] : formula.weightMap)
    weightMap[d4::Lit::makeLit(std::abs(lit), lit < 0)] = weight;

  d4::ProblemManager problem(formula.type, formula.nbVar,
                             formula.quantifications, weightMap, gates,
                             std::cout);

  const std::string format = optionCounter.format.get();
  const std::string outFormat = optionCounter.outFormat.get();

  switch (formula.weightType) {
    case parser::WeightType::INT:
      runProjMc<mpz::mpz_int, semiring::MpzIntSemiring>(
          options, problem, format, outFormat, false);
      break;
    case parser::WeightType::FLOAT:
      runProjMc<mpz::mpf_float, semiring::MpzFloatSemiring>(
          options, problem, format, outFormat, true);
      break;
    default:
      std::cerr << "ERROR: Complex weights are not supported for ProjMC.\n";
      exit(1);
  }
}  // projMcCounter
