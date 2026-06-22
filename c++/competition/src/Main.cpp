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
#include <signal.h>

#include <gmpxx.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>

#include "../semirings/MpzFloatSemiring.hpp"
#include "../semirings/MpzIntSemiring.hpp"
#include "src/options/Option.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "Formula.hpp"
#include "ParserDimacs.hpp"

using namespace d4;

namespace {
// log10 of a (possibly huge) arbitrary-precision value, computed without
// transcendental support from GMP by decomposing the value as d*2^exp first.
double log10Estimate(const mpz_class& n) {
  signed long int exp2;
  double d = mpz_get_d_2exp(&exp2, n.get_mpz_t());
  return std::log10(d) + exp2 * std::log10(2.0);
}

double log10Estimate(const mpf_class& n) {
  signed long int exp2;
  double d = mpf_get_d_2exp(&exp2, n.get_mpf_t());
  return std::log10(d) + exp2 * std::log10(2.0);
}
}  // namespace

#define SOLVER "minisat"

MethodManager *methodRun = nullptr;

/**
 * @brief Catch the signal that ask for stopping the method which is running.
 *
 * @param signum is the signal.
 */
static void signalHandler(int signum) {
  std::cout << "c [MAIN] Method stop\n";
  if (methodRun != nullptr) methodRun->interrupt();
  exit(signum);
}  // signalHandler

void wmc(d4::ProblemManager *problem) {
  std::cout << "c [D4] Run the weigted model counter\n";

  // cache.
  d4::OptionCacheManager cache;
  cache.isActivated = true;
  cache.cachingMethod.setFromString("list");
  cache.optionCacheCleaningManager.cacheCleaningStrategy.setFromString("none");
  cache.optionBucketManager.modeStore.setFromString("not-touched");
  cache.optionBucketManager.clauseRepresentation.setFromString("clause");
  cache.optionBucketManager.sizeFirstPage = 1UL << 32;
  cache.optionBucketManager.sizeAdditionalPage = 1UL << 29;

  // branching heuristic.
  d4::OptionBranchingHeuristic branchingHeuristic;
  branchingHeuristic.freqDecay = 2048;
  branchingHeuristic.scoringMethodType.setFromString("vsads");
  branchingHeuristic.branchingHeuristicType.setFromString("hybrid-partial-classic");
  branchingHeuristic.phaseHeuristicType.setFromString("polarity");
  branchingHeuristic.reversePhase = false;
  branchingHeuristic.optionPartialOrderHeuristic.partialOrderMethod.setFromString("tree-decomposition");
  branchingHeuristic.optionPartialOrderHeuristic.treeDecompositionMethod.setFromString("tree-width");
  branchingHeuristic.optionPartialOrderHeuristic.graphExtractorMethod.setFromString("primal");
  branchingHeuristic.optionPartialOrderHeuristic.treeDecompositionerMethod.setFromString("flow-cutter");
  branchingHeuristic.optionPartialOrderHeuristic.useSimpGraphExtractor = true;

  // configuration of the dpll counter.
  d4::OptionDpllStyleMethod options;
  options.optionCacheManager = cache;
  options.optionBranchingHeuristic = branchingHeuristic;
  options.optionSolver.solverName.setFromString(SOLVER);
  options.optionSpecManager.specUpdateType.setFromString("dynamic");
  options.operationType.setFromString("counting");

  d4::DpllStyleMethod<mpz::mpf_float, semiring::MpzFloatSemiring> *counter =
      new DpllStyleMethod<mpz::mpf_float, semiring::MpzFloatSemiring>(
          options, *problem, std::cout);
  mpz::mpf_float result = counter->run();

  mpf_set_default_prec(426);  // ~128 decimal digits
  std::cout.precision(50);

  if (result == 0) {
    std::cout << "s UNSATISFIABLE\n";
    std::cout << "c s type mc\n";
    std::cout << "c s log10-estimate -inf\n";
    std::cout << "c s exact quadruple int 0\n";
  } else {
    std::cout << "s SATISFIABLE\n";
    std::cout << "c s type mc\n";
    std::cout << "c s log10-estimate "
              << log10Estimate(result) << "\n";
    std::cout << "c s exact quadruple int " << result << "\n";
  }
  delete counter;
}  // wmc

void pwmc(d4::ProblemManager *problem) {
  std::cout << "c [D4] Run the weighted projected model counter\n";

  // cache.
  d4::OptionCacheManager cache;
  cache.isActivated = true;
  cache.cachingMethod.setFromString("list");
  cache.optionCacheCleaningManager.cacheCleaningStrategy.setFromString("none");
  cache.optionBucketManager.modeStore.setFromString("not-touched");
  cache.optionBucketManager.clauseRepresentation.setFromString("clause");
  cache.optionBucketManager.sizeFirstPage = 1UL << 32;
  cache.optionBucketManager.sizeAdditionalPage = 1UL << 29;

  // branching heuristic.
  d4::OptionBranchingHeuristic branchingHeuristic;
  branchingHeuristic.freqDecay = 2048;
  branchingHeuristic.scoringMethodType.setFromString("vsads");
  branchingHeuristic.branchingHeuristicType.setFromString("classic");
  branchingHeuristic.phaseHeuristicType.setFromString("polarity");
  branchingHeuristic.reversePhase = false;

  // configuration of the dpll counter.
  d4::OptionDpllStyleMethod options;
  options.optionCacheManager = cache;
  options.optionBranchingHeuristic = branchingHeuristic;
  options.optionSolver.solverName.setFromString(SOLVER);
  options.optionSpecManager.specUpdateType.setFromString("dynamicBlockedSimp");
  options.operationType.setFromString("counting");

  d4::DpllStyleMethod<mpz::mpf_float, semiring::MpzFloatSemiring> *counter =
      new DpllStyleMethod<mpz::mpf_float, semiring::MpzFloatSemiring>(
          options, *problem, std::cout);
  mpz::mpf_float result = counter->run();

  mpf_set_default_prec(426);  // ~128 decimal digits
  std::cout.precision(50);

  if (result == 0) {
    std::cout << "s UNSATISFIABLE\n";
    std::cout << "c s type mc\n";
    std::cout << "c s log10-estimate -inf\n";
    std::cout << "c s exact quadruple int 0\n";
  } else {
    std::cout << "s SATISFIABLE\n";
    std::cout << "c s type mc\n";
    std::cout << "c s log10-estimate "
              << log10Estimate(result) << "\n";
    std::cout << "c s exact quadruple int " << result << "\n";
  }
  delete counter;
}  // pwmc

void pmc(d4::ProblemManager *problem) {
  std::cout << "c [D4] Run the projected model counter\n";

  // cache.
  d4::OptionCacheManager cache;
  cache.isActivated = true;
  cache.cachingMethod.setFromString("list");
  cache.optionCacheCleaningManager.cacheCleaningStrategy.setFromString("none");
  cache.optionBucketManager.modeStore.setFromString("not-touched");
  cache.optionBucketManager.clauseRepresentation.setFromString("clause");
  cache.optionBucketManager.sizeFirstPage = 1UL << 32;
  cache.optionBucketManager.sizeAdditionalPage = 1UL << 29;

  // branching heuristic.
  d4::OptionBranchingHeuristic branchingHeuristic;
  branchingHeuristic.freqDecay = 2048;
  branchingHeuristic.scoringMethodType.setFromString("vsads");
  branchingHeuristic.branchingHeuristicType.setFromString("classic");
  branchingHeuristic.phaseHeuristicType.setFromString("polarity");
  branchingHeuristic.reversePhase = false;

  // configuration of the dpll counter.
  d4::OptionDpllStyleMethod options;
  options.optionCacheManager = cache;
  options.optionBranchingHeuristic = branchingHeuristic;
  options.optionSolver.solverName.setFromString(SOLVER);
  options.optionSpecManager.specUpdateType.setFromString("dynamicBlockedSimp");
  options.operationType.setFromString("counting");

  d4::DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring> *counter =
      new DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring>(
          options, *problem, std::cout);
  mpz::mpz_int result = counter->run();

  mpf_set_default_prec(426);  // ~128 decimal digits
  std::cout.precision(50);

  if (result == 0) {
    std::cout << "s UNSATISFIABLE\n";
    std::cout << "c s type mc\n";
    std::cout << "c s log10-estimate -inf\n";
    std::cout << "c s exact quadruple int 0\n";
  } else {
    std::cout << "s SATISFIABLE\n";
    std::cout << "c s type mc\n";
    std::cout << "c s log10-estimate "
              << log10Estimate(result) << "\n";
    std::cout << "c s exact arb int " << result << "\n";
  }
  delete counter;
}  // pmc

void mc(d4::ProblemManager *problem) {
  std::cout << "c [D4] Run the model counter\n";

  // cache.
  d4::OptionCacheManager cache;
  cache.isActivated = true;
  cache.cachingMethod.setFromString("list");
  cache.optionCacheCleaningManager.cacheCleaningStrategy.setFromString("none");
  cache.optionBucketManager.modeStore.setFromString("not-touched");
  cache.optionBucketManager.clauseRepresentation.setFromString("clause");
  cache.optionBucketManager.sizeFirstPage = 1UL << 32;
  cache.optionBucketManager.sizeAdditionalPage = 1UL << 29;

  // branching heuristic.
  d4::OptionBranchingHeuristic branchingHeuristic;
  branchingHeuristic.freqDecay = 2048;
  branchingHeuristic.scoringMethodType.setFromString("vsads");
  branchingHeuristic.branchingHeuristicType.setFromString("hybrid-partial-classic");
  branchingHeuristic.phaseHeuristicType.setFromString("polarity");
  branchingHeuristic.reversePhase = false;
  branchingHeuristic.optionPartialOrderHeuristic.partialOrderMethod.setFromString("tree-decomposition");
  branchingHeuristic.optionPartialOrderHeuristic.treeDecompositionMethod.setFromString("tree-width");
  branchingHeuristic.optionPartialOrderHeuristic.graphExtractorMethod.setFromString("primal");
  branchingHeuristic.optionPartialOrderHeuristic.treeDecompositionerMethod.setFromString("flow-cutter");
  branchingHeuristic.optionPartialOrderHeuristic.useSimpGraphExtractor = true;

  // configuration of the dpll counter.
  d4::OptionDpllStyleMethod options;
  options.optionCacheManager = cache;
  options.optionBranchingHeuristic = branchingHeuristic;
  options.optionSolver.solverName.setFromString(SOLVER);
  options.optionSpecManager.specUpdateType.setFromString("dynamic");
  options.operationType.setFromString("counting");

  d4::DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring> *counter =
      new DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring>(
          options, *problem, std::cout);
  mpz::mpz_int result = counter->run();

  mpf_set_default_prec(426);  // ~128 decimal digits
  std::cout.precision(50);

  if (result == 0) {
    std::cout << "s UNSATISFIABLE\n";
    std::cout << "c s type mc\n";
    std::cout << "c s log10-estimate -inf\n";
    std::cout << "c s exact quadruple int 0\n";
  } else {
    std::cout << "s SATISFIABLE\n";
    std::cout << "c s type mc\n";
    std::cout << "c s log10-estimate "
              << log10Estimate(result) << "\n";
    std::cout << "c s exact arb int " << result << "\n";
  }
  delete counter;
}  // mc

void run(const parser::Formula& formula, d4::ProblemManager *initProblem) {
  bool isFloat = (formula.weightType == parser::WeightType::FLOAT);
  bool isProjected = (formula.quantifications.size() > 0 && formula.quantifications[0].size() > 0);

  if (isFloat && isProjected) return pwmc(initProblem);
  if (isFloat) return wmc(initProblem);
  if (isProjected) return pmc(initProblem);
  mc(initProblem);
}  // run

/**
   The main function!
*/
int main(int argc, char **argv) {
  std::cout << "c [D4] Competition version\n";
  auto start = std::chrono::system_clock::now();

  signal(SIGINT, signalHandler);
  const char *inputFile = (argc == 1) ? "/dev/stdin" : argv[1];

  parser::Formula formula;
  parser::ParserDimacs parserDimacs;
  parserDimacs.parse_DIMACS(inputFile, formula);

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

  std::cout << "c [INITIAL INPUT] \033[4m\033[32mStatistics about the input "
                "formula\033[0m\n";
  problem.display(std::cout);
  std::cout << "c\n";

  run(formula, &problem);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "c [COUNTER] Elapsed time: " << elapsed_seconds.count()
            << " seconds\n";

  return EXIT_SUCCESS;
}  // main