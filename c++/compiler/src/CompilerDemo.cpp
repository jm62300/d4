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
 *src/methods/QueryManager.cpp src/methods/QueryManager.hpp
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include "CompilerDemo.hpp"

#include <signal.h>

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/integer.hpp>
#include <cassert>
#include <iomanip>

#include "../semirings/DecDNNFSemiring.hpp"
#include "QueryManager.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

using namespace d4;
namespace mpz = boost::multiprecision;

void compileFormula(const OptionDpllStyleMethod& options,
                    const ProblemManager& problem, const std::string& dumpFile,
                    const std::string& queryFile) {
  auto compilerEngine =
      new DpllStyleMethod<semiring::Node, semiring::DecDNNFSemiring>(
          options, problem, std::cout);
  semiring::Node result = compilerEngine->run();
  const semiring::DecDNNFSemiring& semiring = compilerEngine->getSemiring();

  std::cout << "c [COMPILER] #Nodes: " << semiring.getNbNodes() << '\n';
  std::cout << "c [COMPILER] #Edges: " << semiring.getNbEdges() << '\n';

  std::vector<mpz::mpz_int> weight(2 + problem.getNbVar() * 2, 1);
  mpz::mpz_int count = semiring.count<mpz::mpz_int>(result, std::vector<Lit>(),
                                                    weight, problem.getNbVar());

  std::cout << "s " << count << '\n';
#if 0
  NodeManager<T>* nodeManager =
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
      for (auto& l : query) {
        if ((unsigned)l.var() >= fixedValue.size()) continue;
        fixedValue[l.var()] = (l.sign()) ? ValueVar::isFalse : ValueVar::isTrue;
      }

      if (typeQuery == TypeQuery::QueryCounting) {
        std::cout << "s " << std::fixed
                  << nodeManager->computeNbModels(result, fixedValue, *problem)
                  << "\n";
      } else if (typeQuery == TypeQuery::QueryDecision) {
        bool res = nodeManager->isSAT(result, fixedValue);
        std::cout << "s " << ((res) ? "SAT" : "UNS") << "\n";
      }

      for (auto& l : query) {
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
#endif
  delete compilerEngine;
}  // count

/**
 * @brief counterDemo implementation.
 *
 * Runs the counter using the provided configuration.
 */
void compiler(const d4::OptionDpllStyleMethod& inputConfig,
              const d4::OptionCompiler& optionCompiler,
              const parser::Formula& formula) {
  // Use the provided configuration.
  d4::OptionDpllStyleMethod options = inputConfig;

  // Force counting operation.
  options.operationType = d4::OperationTypeManager::getOperatorType("counting");

  if (options.optionCacheManager.optionBucketManager.clauseRepresentation ==
      CACHE_INDEX)
    options.optionSpecManager.needFastNotSatisfied = true;

  const std::string dumpFile = optionCompiler.dumpFile.get();
  const std::string queryFile = optionCompiler.queryFile.get();

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

  compileFormula(options, problem, "/dev/null", "/dev/null");
}  // counterDemo
