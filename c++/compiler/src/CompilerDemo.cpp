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

#include "CompilerDemo.hpp"

#include <signal.h>

#include <cassert>

#include "ParseOption.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

extern d4::MethodManager *methodRun;

using namespace d4;

template <typename T>
void compiler(const OptionDpllStyleMethod &options, ProblemManager *problem,
              const std::string &dumpFile, const std::string &queryFile) {
  DpllStyleMethod<T, Node<T> *> *comp =
      new DpllStyleMethod<T, Node<T> *>(options, problem, std::cout);

  methodRun = comp;
  Node<T> *result = comp->run();
  NodeManager<T> *nodeManager =
      NodeManager<T>::makeNodeManager(problem->getNbVar() + 1);

  if (dumpFile != "/dev/null") {
    std::ofstream outFile;
    outFile.open(dumpFile);
    nodeManager->printNNF(result, outFile);
    outFile.close();
  } else if (queryFile != "/dev/null") {
    std::vector<Lit> query;
    std::vector<ValueVar> fixedValue(problem->getNbVar() + 1,
                                     ValueVar::isNotAssigned);

    QueryManager queryManager(queryFile);
    TypeQuery typeQuery = TypeQuery::QueryEnd;

    do {
      typeQuery = queryManager.next(query);
      for (auto &l : query) {
        if ((unsigned)l.var() >= fixedValue.size()) continue;
        fixedValue[l.var()] = (l.sign()) ? ValueVar::isFalse : ValueVar::isTrue;
      }

      if (typeQuery == TypeQuery::QueryCounting) {
#if 0
        static unsigned cpt = 0;
        cpt++;
        nodeManager->computeNbModels(result, fixedValue, *problem);
        if (!(cpt % 10000)) std::cout << cpt << '\n';
#else
        std::cout << "s " << std::fixed
                  << nodeManager->computeNbModels(result, fixedValue, *problem)
                  << "\n";
#endif
      } else if (typeQuery == TypeQuery::QueryDecision) {
        bool res = nodeManager->isSAT(result, fixedValue);
        std::cout << "s " << ((res) ? "SAT" : "UNS") << "\n";
      }

      for (auto &l : query) {
        if ((unsigned)l.var() >= fixedValue.size()) continue;
        fixedValue[l.var()] = ValueVar::isNotAssigned;
      }
    } while (typeQuery != TypeQuery::QueryEnd);
  } else {
    std::vector<ValueVar> fixedValue(problem->getNbVar() + 1,
                                     ValueVar::isNotAssigned);
    std::cout << "s " << std::fixed
              << nodeManager->computeNbModels(result, fixedValue, *problem)
              << "\n";
  }

  nodeManager->deallocate(result);

  methodRun = nullptr;
  delete comp;
}  // count

/**
 * @brief couterDemo implementation.
 */
void compilerDemo(const po::variables_map &vm, ProblemManager *problem) {
  // get the configuration.
  OptionDpllStyleMethod options;

  options.methodName = d4::resolve_enum<d4::MethodName>("ddnnf-compiler");

  options.inputName = vm["input"].as<std::string>();
  options.problemInputType = d4::ProblemInputTypeManager::getInputType(
      vm["input-type"].as<std::string>());

  options.optionCacheManager = parseCacheConfiguration(vm);
  options.optionBranchingHeuristic = parseBranchingHeuristicConfiguration(vm);
  options.optionSolver.solverName =
      d4::resolve_enum<d4::SolverName>(vm["solver"].as<std::string>());

  options.optionSpecManager.specUpdateType = d4::resolve_enum<d4::SpecUpdateType>(
      vm["occurrence-manager"].as<std::string>());
  options.optionSpecManager.removeGates = vm["remove-gates"].as<bool>();

  options.operationType =
      d4::OperationTypeManager::getOperatorType("ddnnf-compiler");

  bool isFloat = problem->isFloat();
  MethodManager::displayInfoVariables(problem, std::cout);

  // construct and call the counter regarding if it is MC or WMC.
  std::string dumpFile = vm["dump-file"].as<std::string>();
  std::string queryFile = vm["query"].as<std::string>();

  if (!isFloat)
    compiler<mpz::mpz_int>(options, problem, dumpFile, queryFile);
  else
    compiler<mpz::mpf_float>(options, problem, dumpFile, queryFile);

}  // counterDemo