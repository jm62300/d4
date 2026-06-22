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
#include "src/options/preprocs/OptionPreprocManager.hpp"
#include "src/preprocs/PreprocManager.hpp"

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

void wmc(d4::ProblemManager *initProblem) {
  std::cout << "c [D4] Run the weigted model counter\n";

  ProblemManager *problem = initProblem;
  MethodManager::displayInfoVariables(*problem, std::cout);

  // count.

  // cache.
  d4::OptionCacheManager cache;
  cache.isActivated = true;
  cache.cachingMethod = d4::CachingMethodManager::getCachingMethod("list");
  cache.optionCacheCleaningManager.cacheCleaningStrategy =
      d4::CacheCleaningStrategyManager::getCacheCleaningStrategy("none");
  cache.optionBucketManager.modeStore = d4::ModeStoreManager::getModeStore("not-touched");
  cache.optionBucketManager.clauseRepresentation =
      d4::ClauseRepresentationManager::getClauseRepresentation("clause");
  cache.optionBucketManager.sizeFirstPage = 1UL << 32;
  cache.optionBucketManager.sizeAdditionalPage = 1UL << 29;

  // branching heuristic.
  d4::OptionBranchingHeuristic branchingHeuristic;
  branchingHeuristic.freqDecay = 2048;
  branchingHeuristic.scoringMethodType =
      d4::ScoringMethodTypeManager::getScoringMethodType("vsads");
  branchingHeuristic.branchingHeuristicType =
      d4::BranchingHeuristicTypeManager::getBranchingHeuristicType(
          "hybrid-partial-classic");
  branchingHeuristic.phaseHeuristicType =
      d4::PhaseHeuristicTypeManager::getPhaseHeuristicType("polarity");
  branchingHeuristic.reversePhase = false;
  branchingHeuristic.optionPartialOrderHeuristic.partialOrderMethod =
      d4::PartialOrderMethodManager::getPartialOrderMethod(
          "tree-decomposition");
  branchingHeuristic.optionPartialOrderHeuristic
      .treeDecompositionMethod =
      d4::TreeDecompositionMethodManager::getTreeDecompositionMethod(
          "tree-width");
  branchingHeuristic.optionPartialOrderHeuristic.graphExtractorMethod =
      d4::GraphExtractorMethodManager::getGraphExtractorMethodManager("primal");
  branchingHeuristic.optionPartialOrderHeuristic
      .treeDecompositionerMethod =
      d4::TreeDecompositionerMethodManager::getTreeDecompositionerMethodManager(
          "flow-cutter");
  branchingHeuristic.optionPartialOrderHeuristic.useSimpGraphExtractor =
      true;

  // configuration of the dpll counter.
  d4::OptionDpllStyleMethod options;
  options.methodName = d4::MethodNameManager::getMethodName("counting");
  options.problemInputType =
      d4::ProblemInputTypeManager::getInputType("cnf");
  options.optionCacheManager = cache;
  options.optionBranchingHeuristic = branchingHeuristic;
  options.optionSolver.solverName =
      d4::SolverNameManager::getSolverName(SOLVER);
  options.optionSpecManager.specUpdateType =
      d4::SpecUpdateManager::getSpecUpdate("dynamic");
  options.operationType =
      d4::OperationTypeManager::getOperatorType("counting");

  d4::DpllStyleMethod<mpz::mpf_float, semiring::MpzFloatSemiring> *counter =
      new DpllStyleMethod<mpz::mpf_float, semiring::MpzFloatSemiring>(
          options, problem, std::cout);
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
}  // wmc

void pwmc(d4::ProblemManager *initProblem) {
  std::cout << "c [D4] Run the weighted projected model counter\n";

  // preproc.
  /*
  d4::ConfigurationPeproc configPreproc;
  configPreproc.inputType = d4::ProblemInputTypeManager::getInputType("cnf");
  configPreproc.preprocMethod =
      d4::PreprocMethodManager::getPreprocMethod("basic");
  configPreproc.nbIteration = 5;
  configPreproc.timeout = 60;

  ProblemManager *problem =
      d4::MethodManager::runPreproc(configPreproc, initProblem, std::cout);
  */
  ProblemManager *problem = initProblem;
  MethodManager::displayInfoVariables(*problem, std::cout);

  // count.

  // cache.
  d4::OptionCacheManager cache;
  cache.isActivated = true;
  cache.cachingMethod = d4::CachingMethodManager::getCachingMethod("list");
  cache.optionCacheCleaningManager.cacheCleaningStrategy =
      d4::CacheCleaningStrategyManager::getCacheCleaningStrategy("none");
  cache.optionBucketManager.modeStore = d4::ModeStoreManager::getModeStore("not-touched");
  cache.optionBucketManager.clauseRepresentation =
      d4::ClauseRepresentationManager::getClauseRepresentation("clause");
  cache.optionBucketManager.sizeFirstPage = 1UL << 32;
  cache.optionBucketManager.sizeAdditionalPage = 1UL << 29;

  // branching heuristic.
  d4::OptionBranchingHeuristic branchingHeuristic;
  branchingHeuristic.freqDecay = 2048;
  branchingHeuristic.scoringMethodType =
      d4::ScoringMethodTypeManager::getScoringMethodType("vsads");
  branchingHeuristic.branchingHeuristicType =
      d4::BranchingHeuristicTypeManager::getBranchingHeuristicType("classic");
  branchingHeuristic.phaseHeuristicType =
      d4::PhaseHeuristicTypeManager::getPhaseHeuristicType("polarity");
  branchingHeuristic.reversePhase = false;

  // configuration of the dpll counter.
  d4::OptionDpllStyleMethod options;
  options.methodName = d4::MethodNameManager::getMethodName("counting");
  options.problemInputType =
      d4::ProblemInputTypeManager::getInputType("cnf");
  options.optionCacheManager = cache;
  options.optionBranchingHeuristic = branchingHeuristic;
  options.optionSolver.solverName =
      d4::SolverNameManager::getSolverName(SOLVER);
  options.optionSpecManager.specUpdateType =
      d4::SpecUpdateManager::getSpecUpdate("dynamicBlockedSimp");
  options.operationType =
      d4::OperationTypeManager::getOperatorType("counting");

  d4::DpllStyleMethod<mpz::mpf_float, semiring::MpzFloatSemiring> *counter =
      new DpllStyleMethod<mpz::mpf_float, semiring::MpzFloatSemiring>(
          options, problem, std::cout);
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
}  // pwmc

void pmc(d4::ProblemManager *initProblem) {
  std::cout << "c [D4] Run the projected model counter\n";

  // preproc.
  /*
  d4::ConfigurationPeproc configPreproc;
  configPreproc.inputType = d4::ProblemInputTypeManager::getInputType("cnf");
  configPreproc.preprocMethod =
      d4::PreprocMethodManager::getPreprocMethod("basic");
  configPreproc.nbIteration = 5;
  configPreproc.timeout = 60;

  ProblemManager *problem =
      d4::MethodManager::runPreproc(configPreproc, initProblem, std::cout);
  */
  ProblemManager *problem = initProblem;
  MethodManager::displayInfoVariables(*problem, std::cout);

  // count.

  // cache.
  d4::OptionCacheManager cache;
  cache.isActivated = true;
  cache.cachingMethod = d4::CachingMethodManager::getCachingMethod("list");
  cache.optionCacheCleaningManager.cacheCleaningStrategy =
      d4::CacheCleaningStrategyManager::getCacheCleaningStrategy("none");
  cache.optionBucketManager.modeStore = d4::ModeStoreManager::getModeStore("not-touched");
  cache.optionBucketManager.clauseRepresentation =
      d4::ClauseRepresentationManager::getClauseRepresentation("clause");
  cache.optionBucketManager.sizeFirstPage = 1UL << 32;
  cache.optionBucketManager.sizeAdditionalPage = 1UL << 29;

  // branching heuristic.
  d4::OptionBranchingHeuristic branchingHeuristic;
  branchingHeuristic.freqDecay = 2048;
  branchingHeuristic.scoringMethodType =
      d4::ScoringMethodTypeManager::getScoringMethodType("vsads");
  branchingHeuristic.branchingHeuristicType =
      d4::BranchingHeuristicTypeManager::getBranchingHeuristicType("classic");
  branchingHeuristic.phaseHeuristicType =
      d4::PhaseHeuristicTypeManager::getPhaseHeuristicType("polarity");
  branchingHeuristic.reversePhase = false;

  // configuration of the dpll counter.
  d4::OptionDpllStyleMethod options;
  options.methodName = d4::MethodNameManager::getMethodName("counting");
  options.problemInputType =
      d4::ProblemInputTypeManager::getInputType("cnf");
  options.optionCacheManager = cache;
  options.optionBranchingHeuristic = branchingHeuristic;
  options.optionSolver.solverName =
      d4::SolverNameManager::getSolverName(SOLVER);
  options.optionSpecManager.specUpdateType =
      d4::SpecUpdateManager::getSpecUpdate("dynamicBlockedSimp");
  options.operationType =
      d4::OperationTypeManager::getOperatorType("counting");

  d4::DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring> *counter =
      new DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring>(
          options, problem, std::cout);
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
}  // pmc

void mc(d4::ProblemManager *initProblem) {
  std::cout << "c [D4] Run the model counter\n";

  // preproc.
  /*
  d4::ConfigurationPeproc configPreproc;
  configPreproc.inputType = d4::ProblemInputTypeManager::getInputType("cnf");
  configPreproc.preprocMethod =
      d4::PreprocMethodManager::getPreprocMethod("sharp-equiv");
  configPreproc.nbIteration = 5;
  configPreproc.timeout = 60;

  ProblemManager *problem =
      d4::MethodManager::runPreproc(configPreproc, initProblem, std::cout);
  */
  ProblemManager *problem = initProblem;
  MethodManager::displayInfoVariables(*problem, std::cout);

  // count.

  // cache.
  d4::OptionCacheManager cache;
  cache.isActivated = true;
  cache.cachingMethod = d4::CachingMethodManager::getCachingMethod("list");
  cache.optionCacheCleaningManager.cacheCleaningStrategy =
      d4::CacheCleaningStrategyManager::getCacheCleaningStrategy("none");
  cache.optionBucketManager.modeStore = d4::ModeStoreManager::getModeStore("not-touched");
  cache.optionBucketManager.clauseRepresentation =
      d4::ClauseRepresentationManager::getClauseRepresentation("clause");
  cache.optionBucketManager.sizeFirstPage = 1UL << 32;
  cache.optionBucketManager.sizeAdditionalPage = 1UL << 29;

  // branching heuristic.
  d4::OptionBranchingHeuristic branchingHeuristic;
  branchingHeuristic.freqDecay = 2048;
  branchingHeuristic.scoringMethodType =
      d4::ScoringMethodTypeManager::getScoringMethodType("vsads");
  branchingHeuristic.branchingHeuristicType =
      d4::BranchingHeuristicTypeManager::getBranchingHeuristicType(
          "hybrid-partial-classic");
  branchingHeuristic.phaseHeuristicType =
      d4::PhaseHeuristicTypeManager::getPhaseHeuristicType("polarity");
  branchingHeuristic.reversePhase = false;
  branchingHeuristic.optionPartialOrderHeuristic.partialOrderMethod =
      d4::PartialOrderMethodManager::getPartialOrderMethod(
          "tree-decomposition");
  branchingHeuristic.optionPartialOrderHeuristic
      .treeDecompositionMethod =
      d4::TreeDecompositionMethodManager::getTreeDecompositionMethod(
          "tree-width");
  branchingHeuristic.optionPartialOrderHeuristic.graphExtractorMethod =
      d4::GraphExtractorMethodManager::getGraphExtractorMethodManager("primal");
  branchingHeuristic.optionPartialOrderHeuristic
      .treeDecompositionerMethod =
      d4::TreeDecompositionerMethodManager::getTreeDecompositionerMethodManager(
          "flow-cutter");
  branchingHeuristic.optionPartialOrderHeuristic.useSimpGraphExtractor =
      true;

  // configuration of the dpll counter.
  d4::OptionDpllStyleMethod options;
  options.methodName = d4::MethodNameManager::getMethodName("counting");
  options.problemInputType =
      d4::ProblemInputTypeManager::getInputType("cnf");
  options.optionCacheManager = cache;
  options.optionBranchingHeuristic = branchingHeuristic;
  options.optionSolver.solverName =
      d4::SolverNameManager::getSolverName(SOLVER);
  options.optionSpecManager.specUpdateType =
      d4::SpecUpdateManager::getSpecUpdate("dynamic");
  options.operationType =
      d4::OperationTypeManager::getOperatorType("counting");

  d4::DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring> *counter =
      new DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring>(
          options, problem, std::cout);
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
}  // mc

void run(d4::ProblemManager *initProblem) {
  if (initProblem->isFloat() && initProblem->getSelectedVar().size())
    return pwmc(initProblem);
  if (initProblem->isFloat()) return wmc(initProblem);
  if (initProblem->getSelectedVar().size()) return pmc(initProblem);
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

  // parse the initial problem.
  d4::ProblemManager *initProblem = d4::ProblemManager::makeProblemManager(
      inputFile, d4::ProblemInputTypeManager::getInputType("cnf"), std::cout);
  assert(initProblem);
  std::cout << "c [INITIAL INPUT] \033[4m\033[32mStatistics about the input "
                "formula\033[0m\n";
  initProblem->displayStat(std::cout, "c [INITIAL INPUT] ");
  std::cout << "c\n";

  run(initProblem);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "c [COUNTER] Elapsed time: " << elapsed_seconds.count()
            << " seconds\n";

  delete initProblem;
  return EXIT_SUCCESS;
}  // main