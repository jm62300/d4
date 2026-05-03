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

#include "../semirings/MpzFloatSemiring.hpp"
#include "../semirings/MpzIntSemiring.hpp"
#include "ParseOption.hpp"
#include "src/configurations/ConfigurationDpllStyleMethod.hpp"
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

  methodRun = counter;
  T result = counter->run();

  if (outFormat == "competition") {
    boost::multiprecision::mpf_float::default_precision(128);
    std::cout.precision(
        std::numeric_limits<boost::multiprecision::cpp_dec_float_50>::digits10);

    if (result == 0) {
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
    exit(0);  // stop faster than cleaning the memory!
  } else {
    assert(outFormat == "classic");
    std::cout << format << " ";
    std::cout << std::fixed << std::setprecision(50) << result << "\n";
  }

  methodRun = nullptr;
  delete counter;
}  // count

/**
 * @brief couterDemo implementation.
 */
void counterDemo(const po::variables_map& vm, const parser::Formula& formula) {
  // get the configuration.
  ConfigurationDpllStyleMethod config;

  config.methodName = d4::MethodNameManager::getMethodName("counting");

  config.inputName = vm["input"].as<std::string>();
  config.problemInputType = d4::ProblemInputTypeManager::getInputType(
      vm["input-type"].as<std::string>());

  config.cache = parseCacheConfiguration(vm);
  config.branchingHeuristic = parseBranchingHeuristicConfiguration(vm);
  config.solver.solverName =
      d4::SolverNameManager::getSolverName(vm["solver"].as<std::string>());

  config.spec.specUpdateType = d4::SpecUpdateManager::getSpecUpdate(
      vm["occurrence-manager"].as<std::string>());
  config.spec.removeGates = vm["remove-gates"].as<bool>();

  config.exploitModel = vm["exploit-model-activated"].as<bool>();
  config.operationType = d4::OperationTypeManager::getOperatorType("counting");

  // init the options.
  if (config.cache.clauseRepresentation == CACHE_INDEX)
    config.spec.needFastNotSatisfied = true;
  OptionDpllStyleMethod options(config);

  // construct and call the counter regarding if it is MC or WMC.
  std::string format = vm["keyword-output-format-solution"].as<std::string>();
  std::string outFormat = vm["output-format"].as<std::string>();

  // get the clauses.
  std::vector<d4::BcGate> gates;
  gates.reserve(formula.clauses.size());
  for (auto& cl : formula.clauses) {
    std::vector<d4::Lit> d4Clause;
    for (auto& l : cl) d4Clause.push_back(d4::Lit::makeLit(std::abs(l), l < 0));

    gates.push_back({d4Clause, d4::lit_Undef, BcGateType::CLAUSE});
  }

  // get the weights.
  std::map<d4::Lit, std::string> weightMap;
  // transform the map.
  for (const auto& [lit, weight] : formula.weightMap)
    weightMap[d4::Lit::makeLit(std::abs(lit), lit < 0)] = weight;

  d4::ProblemManager problem(formula.type, formula.nbVar,
                             formula.quantifications, weightMap, gates,
                             std::cout);

  if (!formula.weighted)
    countModels<mpz::mpz_int, semiring::MpzIntSemiring>(
        options, problem, format, outFormat, false);
  else
    countModels<mpz::mpf_float, semiring::MpzFloatSemiring>(
        options, problem, format, outFormat, false);
}  // counterDemo