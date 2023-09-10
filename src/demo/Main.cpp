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

#include <boost/multiprecision/gmp.hpp>
#include <boost/program_options.hpp>
#include <cassert>
#include <iostream>
#include <vector>

#include "src/configurations/Configuration.hpp"
#include "src/configurations/ConfigurationDpllStyleMethod.hpp"
#include "src/demo/ParseOption.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/preprocs/OptionPreprocManager.hpp"
#include "src/preprocs/PreprocManager.hpp"

#ifndef NOMAIN

namespace po = boost::program_options;
d4::MethodManager *methodRun = nullptr;

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

/**
 * @brief Get the options from the command line and return the adequate
 * configuration.
 *
 * @param vm are options.
 * @return a suited configuration.
 */
d4::Configuration *parseConfiguration(po::variables_map &vm) {
  std::string meth = vm["method"].as<std::string>();

  d4::ConfigurationDpllStyleMethod *config =
      new d4::ConfigurationDpllStyleMethod();
  config->methodName =
      d4::MethodNameManager::getMethodName(vm["method"].as<std::string>());

  config->inputName = vm["input"].as<std::string>();
  config->problemInputType = d4::ProblemInputTypeManager::getInputType(
      vm["input-type"].as<std::string>());

  config->configurationPreproc = parsePreprocConfiguration(vm);
  config->cache = parseCacheConfiguration(vm);

  config->solver.solverName =
      d4::SolverNameManager::getSolverName(vm["solver"].as<std::string>());

  config->spec.specUpdateType = d4::SpecUpdateManager::getSpecUpdate(
      vm["occurrence-manager"].as<std::string>());

  config->branchingHeuristic.freqDecay =
      vm["scoring-method-freq-decay"].as<unsigned>();

  config->branchingHeuristic.scoringMethodType =
      d4::ScoringMethodTypeManager::getScoringMethodType(
          vm["scoring-method"].as<std::string>());

  config->branchingHeuristic.phaseHeuristicType =
      d4::PhaseHeuristicTypeManager::getPhaseHeuristicType(
          vm["phase-heuristic"].as<std::string>());

  config->branchingHeuristic.reversePhase =
      vm["phase-heuristic-reversed"].as<bool>();

  config->partitioningHeuristic.partitioningMethod =
      d4::PartitioningMethodManager::getPartitioningMethod(
          vm["partitioning-heuristic"].as<std::string>());

  config->partitioningHeuristic.partitionerName =
      d4::PartitionerNameManager::getPartitionerName(
          vm["partitioning-heuristic-partitioner"].as<std::string>());

  config->partitioningHeuristic.reduceFormula =
      vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();

  config->partitioningHeuristic.equivSimp =
      vm["partitioning-heuristic-simplification-equivalence"].as<bool>();

  config->partitioningHeuristic.staticPhase =
      vm["partitioning-heuristic-bipartite-phase-static"].as<int>();

  config->partitioningHeuristic.dynamicPhase =
      vm["partitioning-heuristic-bipartite-phase-dynamic"].as<double>();

  config->operationType =
      d4::OperationTypeManager::getOperatorType(vm["method"].as<std::string>());

  return config;
}  // parseConfiguration

/**
   The main function!
*/
int main(int argc, char **argv) {
  po::options_description desc{"Options"};
  desc.add_options()
#include "option.dsc"
      ;

  signal(SIGINT, signalHandler);
  po::variables_map vm;
  po::store(parse_command_line(argc, argv, desc), vm);

  try {
    po::notify(vm);
  } catch (const po::error &ex) {
    std::cerr << ex.what() << '\n';
    exit(1);
  }

  // help or problem with the command line
  if (vm.count("help") || !vm.count("input")) {
    if (!vm.count("help"))
      std::cout << "Some parameters are missing, please read the README\n";
    std::cout << "USAGE: " << argv[0] << " -i INPUT -m METH [OPTIONS]\n";
    std::cout << desc << '\n';
    exit(!vm.count("help"));
  }

  // parse the initial problem.
  d4::ProblemManager *initProblem = d4::ProblemManager::makeProblemManager(
      vm["input"].as<std::string>(),
      d4::ProblemInputTypeManager::getInputType(
          vm["input-type"].as<std::string>()),
      std::cout);
  assert(initProblem);
  std::cout << "c [INITIAL INPUT] \033[4m\033[32mStatistics about the input "
               "formula\033[0m\n";
  initProblem->displayStat(std::cout, "c [INITIAL INPUT] ");
  std::cout << "c\n";

  // preproc.
  d4::ProblemManager *problem = d4::MethodManager::runPreproc(
      parsePreprocConfiguration(vm), initProblem, std::cout);
  delete initProblem;

  // run the method asked.
  d4::MethodName methodName =
      d4::MethodNameManager::getMethodName(vm["method"].as<std::string>());

  d4::Configuration *config = parseConfiguration(vm);
  methodRun =
      d4::MethodManager::makeMethodManager(vm, problem, *config, std::cout);

  methodRun->run(vm);
  delete methodRun;
  methodRun = nullptr;

  return EXIT_SUCCESS;
}  // main
#endif