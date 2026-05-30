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

  std::map<Lit, std::string> mapWeight = problem.getWeightMap();

  std::vector<mpz::mpz_int> noweight(2 + problem.getNbVar() * 2, 1);
  std::vector<mpz::mpf_float> weight(2 + problem.getNbVar() * 2, 1);
  for (const auto& [lit, w] : mapWeight)
    weight[lit.intern()] = mpz::mpf_float(w);

  if (dumpFile != "/dev/null") {
    std::ofstream outFile;
    outFile.open(dumpFile);
    semiring.printNNF(result, outFile);
    outFile.close();
  } else if (queryFile != "/dev/null") {
    std::vector<Lit> query;
    QueryManager queryManager(queryFile);
    TypeQuery typeQuery = TypeQuery::QueryEnd;

    do {
      mpz::mpf_float count;
      typeQuery = queryManager.next(query);
      switch (typeQuery) {
        case TypeQuery::QueryCounting:
          if (mapWeight.size())
            std::cout << "s "
                      << semiring.count<mpz::mpf_float>(result, query, weight,
                                                        problem.getNbVar())
                      << '\n';
          else
            std::cout << "s "
                      << semiring.count<mpz::mpz_int>(result, query, noweight,
                                                      problem.getNbVar())
                      << '\n';
          break;
        case TypeQuery::QueryEnum:
          break;
        case TypeQuery::QueryPartialEnum: {
          unsigned nbVar = problem.getNbVar();
          mpz::mpz_int total = 0;
          semiring.enumeratePartialModels(
              result, query, nbVar, [&](std::vector<d4::Lit>& cube) {
                total += mpz::mpz_int(1) << (nbVar - cube.size());
              });
          std::cout << "s " << total << '\n';
          break;
        }
        case TypeQuery::QueryDecision:
          std::cout << "s "
                    << ((semiring.isSAT(result, query, problem.getNbVar()))
                            ? "SAT"
                            : "UNS")
                    << "\n";
        case TypeQuery::QueryEnd:
          break;
      }

    } while (typeQuery != TypeQuery::QueryEnd);
  } else {
    mpz::mpf_float count = semiring.count<mpz::mpf_float>(
        result, std::vector<Lit>(), weight, problem.getNbVar());
    std::cout << "s " << count << '\n';
  }

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

  compileFormula(options, problem, optionCompiler.dumpFile,
                 optionCompiler.queryFile);
}  // counterDemo
