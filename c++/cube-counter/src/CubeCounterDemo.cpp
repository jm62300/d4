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

#include "CubeCounterDemo.hpp"

#include <signal.h>

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/integer.hpp>
#include <cassert>
#include <chrono>
#include <iomanip>

#include "../semirings/DecDNNFSemiring.hpp"
#include "../semirings/MpzIntSemiring.hpp"
#include "3rdParty/cadical/src/cadical.hpp"
#include "QueryManager.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

using namespace d4;
namespace mpz = boost::multiprecision;

class ModelAccumulator {
 private:
  mpz::mpz_int m_totalModels;
  unsigned m_nbVar;

  d4::DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring>* counter;
  std::vector<d4::Var> setOfVars;
  CaDiCaL::Solver cadical;

 public:
  /**
   * @brief Constructor
   * @param nbVar The total number of variables in the problem.
   */
  ModelAccumulator(const ProblemManager& problem)
      : m_totalModels(0), m_nbVar(problem.getNbVar()) {
    d4::OptionDpllStyleMethod options;

    parser::Formula formula;
    parser::ParserDimacs parserDimacs;
    parserDimacs.parse_DIMACS(
        "/data/Benchmarks/counting/MC2025_all/mc2025_track1_all/"
        "mc2025_track1_033.cnf",
        formula);
    cadical.declare_more_variables(formula.nbVar + 1);

    // Build the problem from the parsed formula.
    std::vector<d4::BcGate> gates;
    gates.reserve(formula.clauses.size());
    for (auto& cl : formula.clauses) {
      std::vector<d4::Lit> d4Clause;
      for (auto& l : cl) {
        d4Clause.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
        cadical.add(l);
      }
      cadical.add(0);
      gates.push_back({d4Clause, d4::lit_Undef, BcGateType::CLAUSE});
    }

    std::map<d4::Lit, std::string> weightMap;
    for (const auto& [lit, weight] : formula.weightMap)
      weightMap[d4::Lit::makeLit(std::abs(lit), lit < 0)] = weight;

    d4::ProblemManager completeProblem(formula.type, formula.nbVar,
                                       formula.quantifications, weightMap,
                                       gates, std::cout);

    counter = new d4::DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring>(
        options, completeProblem, std::cout);

    for (unsigned i = 1; i <= problem.getNbVar(); i++) setOfVars.push_back(i);
  }

  /**
   * @brief Processes a partial model and adds its complete models to the total.
   * @param partial_model The unit literals assigned in this specific branch.
   */
  void addPartialModel(std::vector<d4::Lit>& partial_model) {
    static int cpt = 0;
    static double cumulative_time_sat = 0.0;  // Accumulator for total time
    static double cumulative_time_uns = 0.0;  // Accumulator for total time
    cpt++;

    // 1. Start the timer
    auto start_time = std::chrono::steady_clock::now();

#if 1
    // first call the SAT solver.
    cadical.reset_assumptions();
    for (auto& l : partial_model) cadical.assume(l.human());

    if (cadical.solve() == 20) {
      std::vector<int> clause;
      for (auto& l : partial_model)
        if (cadical.failed(l.human())) clause.push_back(-l.human());

      // for (auto l : clause) cadical.add(l);
      // cadical.add(0);

      // 2a. Stop timer, update cumulative time, and print for UNSAT
      auto end_time = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = end_time - start_time;
      cumulative_time_uns += elapsed.count();

      std::cout << cpt << " unsat computed in " << elapsed.count()
                << "s (Total: " << cumulative_time_uns << '+'
                << cumulative_time_sat << " = "
                << cumulative_time_uns + cumulative_time_sat << "s)\n";
      return;
    }

    m_totalModels += counter->count(setOfVars, partial_model, std::cout);
#endif

    // 2b. Stop timer, update cumulative time, and print for SAT
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    cumulative_time_sat += elapsed.count();

    std::cout << cpt << " consider a new model: " << m_totalModels
              << " computed in " << elapsed.count()
              << "s (Total: " << cumulative_time_uns << '+'
              << cumulative_time_sat << " = "
              << cumulative_time_uns + cumulative_time_sat << "s)\n";
  }

  /**
   * @brief Retrieves the final accumulated model count.
   */
  mpz::mpz_int getTotal() const { return m_totalModels; }
};

void compileFormula(const OptionDpllStyleMethod& options,
                    const ProblemManager& problem, const std::string& dumpFile,
                    const std::string& queryFile) {
  auto compilerEngine =
      new DpllStyleMethod<semiring::Node, semiring::DecDNNFSemiring>(
          options, problem, std::cout);
  semiring::Node result = compilerEngine->run();
  const semiring::DecDNNFSemiring& semiring = compilerEngine->getSemiring();

  std::cout << "c [CUBE-COUNTER] #Nodes: " << semiring.getNbNodes() << '\n';
  std::cout << "c [CUBE-COUNTER] #Edges: " << semiring.getNbEdges() << '\n';

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

    ModelAccumulator accumulator(problem);
    auto countCallback = [&](std::vector<d4::Lit>& partial_model) {
      accumulator.addPartialModel(partial_model);
    };

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
        case TypeQuery::QueryPartialEnum:
          semiring.enumeratePartialModels(result, query, problem.getNbVar(),
                                          countCallback);
          std::cout << "model count " << accumulator.getTotal() << '\n';
          break;
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
void cubeCounter(const d4::OptionDpllStyleMethod& inputConfig,
                 const d4::OptionCubeCounter& optionCubeCounter,
                 const parser::Formula& formula) {
  // Use the provided configuration.
  d4::OptionDpllStyleMethod options = inputConfig;

  // Force counting operation.
  options.operationType = d4::OperationTypeManager::getOperatorType("counting");

  if (options.optionCacheManager.optionBucketManager.clauseRepresentation ==
      CACHE_INDEX)
    options.optionSpecManager.needFastNotSatisfied = true;

  const std::string dumpFile = optionCubeCounter.dumpFile.get();
  const std::string queryFile = optionCubeCounter.queryFile.get();

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

  compileFormula(options, problem, optionCubeCounter.dumpFile,
                 optionCubeCounter.queryFile);
}  // cubeCounter
