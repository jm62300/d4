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

#include "CompilerDemo.hpp"
#include "ParseOption.hpp"
#include "src/configurations/Configuration.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/preprocs/OptionPreprocManager.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/circuit/ProblemManagerCircuit.hpp"

#ifndef NOMAIN

using namespace d4;
namespace po = boost::program_options;
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

  std::vector<d4::Var> projectedVar;

  if (vm["translate"].as<std::string>() != "none") {
    std::cout << "c [TRANSLATION] Translate the input formula: "
              << vm["input-type"].as<std::string>() << " -> "
              << vm["translate"].as<std::string>() << '\n';

    static_cast<d4::ProblemManagerCircuit *>(initProblem)
        ->getInputVar(projectedVar);

    d4::ProblemManager *tmp =
        initProblem->translate(d4::ProblemTranslateTypeManager::getInputType(
            vm["translate"].as<std::string>()));
    delete initProblem;
    initProblem = tmp;
  }

  // run the method asked.
  d4::MethodName methodName =
      d4::MethodNameManager::getMethodName("ddnnf-compiler");

  // preproc.
  d4::ConfigurationPeproc configPreproc = parsePreprocConfiguration(vm);

  bool rewind = false;
  if (vm["translate"].as<std::string>() == "cnf" &&
      vm["preproc"].as<std::string>() == "compile-equiv") {
    rewind = true;

    std::vector<d4::Var> &vars = initProblem->getSelectedVar();
    for (auto &v : projectedVar) vars.push_back(v);
  }

  configPreproc.inputType = initProblem->getProblemType();
  ProblemManager *problem =
      d4::MethodManager::runPreproc(configPreproc, initProblem, std::cout);

  if (rewind) problem->getSelectedVar().clear();

  // compile.
  compilerDemo(vm, problem);

  delete initProblem;
  return EXIT_SUCCESS;
}  // main
#endif