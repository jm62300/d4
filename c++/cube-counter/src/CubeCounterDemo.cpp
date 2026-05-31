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

#include "CubeCounterDemo.hpp"

#include <signal.h>

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/integer.hpp>
#include <cassert>
#include <chrono>
#include <iomanip>
#include <memory>

#include "../semirings/DecDNNFSemiring.hpp"
#include "../semirings/MpzIntSemiring.hpp"
#include "3rdParty/cadical/src/cadical.hpp"
#include "QueryManager.hpp"
#include "selectors/IterativePrimalCutSelector.hpp"
#include "selectors/PrimalCutSelector.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

using namespace d4;
namespace mpz = boost::multiprecision;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * @brief Build a ProblemManager from a subset of formula clauses.
 *
 * @param formula      The full parsed formula.
 * @param clauseIdxs   Indices into formula.clauses; pass all indices for the
 *                     full problem.
 * @param out          Log stream.
 */
static d4::ProblemManager buildProblem(const parser::Formula& formula,
                                       const std::vector<unsigned>& clauseIdxs,
                                       std::ostream& out) {
  std::vector<d4::BcGate> gates;
  gates.reserve(clauseIdxs.size());
  for (unsigned idx : clauseIdxs) {
    const auto& cl = formula.clauses[idx];
    std::vector<d4::Lit> d4Clause;
    for (auto l : cl) d4Clause.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
    gates.push_back({d4Clause, d4::lit_Undef, BcGateType::CLAUSE});
  }

  std::map<d4::Lit, std::string> weightMap;
  for (const auto& [lit, weight] : formula.weightMap)
    weightMap[d4::Lit::makeLit(std::abs(lit), lit < 0)] = weight;

  return d4::ProblemManager(formula.type, formula.nbVar,
                            formula.quantifications, weightMap, gates, out);
}

/**
 * @brief Core cube-and-count implementation.
 *
 * 1. Compiles F_easy (given by easyClauses) into a decision-DNNF.
 * 2. Enumerates its partial models (cubes).
 * 3. For each cube σ: checks satisfiability of F ∧ σ, then adds
 *    count(F | σ) to the total.
 *
 * Correctness: because F_easy ⊆ F and decision-DNNFs are deterministic, the
 * cubes are mutually exclusive and every model of F extends exactly one cube.
 * After conditioning on a cube, the residual formula decomposes into the two
 * independent components created by the primal cut, which the full counter
 * exploits automatically through its component analysis.
 */
static void cubeAndCount(const OptionDpllStyleMethod& inputConfig,
                         const ProblemManager& fullProblem,
                         const parser::Formula& formula,
                         const std::vector<unsigned>& easyClauses) {
  d4::OptionDpllStyleMethod options = inputConfig;
  options.operationType = d4::OperationTypeManager::getOperatorType("counting");
  if (options.optionCacheManager.optionBucketManager.clauseRepresentation ==
      CACHE_INDEX)
    options.optionSpecManager.needFastNotSatisfied = true;

  std::cout << "c [CUBE-COUNTER] F_easy: " << easyClauses.size() << '/'
            << formula.clauses.size() << " clauses\n";

  // --- Compile F_easy ---
  d4::ProblemManager easyProblem =
      buildProblem(formula, easyClauses, std::cout);

  auto* easyCompiler =
      new DpllStyleMethod<semiring::Node, semiring::DecDNNFSemiring>(
          options, easyProblem, std::cout);
  semiring::Node D_easy = easyCompiler->run();
  const semiring::DecDNNFSemiring& sem = easyCompiler->getSemiring();

  std::cout << "c [CUBE-COUNTER] F_easy DNNF nodes=" << sem.getNbNodes()
            << " edges=" << sem.getNbEdges() << '\n';

  // --- Full counter (reused across all cubes) ---
  auto* fullCounter =
      new DpllStyleMethod<mpz::mpz_int, semiring::MpzIntSemiring>(
          options, fullProblem, std::cout);

  std::vector<d4::Var> allVars;
  allVars.reserve(formula.nbVar);
  for (unsigned i = 1; i <= formula.nbVar; i++) allVars.push_back(i);

  // --- SAT checker for full formula (prunes unsatisfiable cubes) ---
  CaDiCaL::Solver cadical;
  cadical.set("quiet", 1);
  cadical.declare_more_variables(formula.nbVar);
  for (const auto& cl : formula.clauses) {
    for (auto l : cl) cadical.add(l);
    cadical.add(0);
  }

  // --- Null stream: suppress per-cube counter log output ---
  struct NullBuf : std::streambuf {
    int overflow(int c) override { return c; }
  } nullBuf;
  std::ostream nullStream(&nullBuf);

  // --- Enumerate and count ---
  mpz::mpz_int total = 0;
  unsigned cubeCount = 0, prunedCount = 0;
  auto wallStart = std::chrono::steady_clock::now();

  sem.enumeratePartialModels(
      D_easy, {}, formula.nbVar, [&](std::vector<d4::Lit>& sigma) {
        cubeCount++;

        cadical.reset_assumptions();
        for (auto& l : sigma) cadical.assume(l.human());

        if (cadical.solve() == 20) {
          prunedCount++;
          return;
        }

        total += fullCounter->count(allVars, sigma, nullStream);

        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - wallStart).count();
        std::cout << "c cube " << cubeCount << " pruned=" << prunedCount
                  << " total=" << total << " t=" << t << "s\n";
      });

  double elapsed = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - wallStart)
                       .count();

  std::cout << "c [CUBE-COUNTER] cubes=" << cubeCount
            << " pruned=" << prunedCount << " time=" << elapsed << "s\n";
  std::cout << "s " << total << '\n';

  delete easyCompiler;
  delete fullCounter;
}

// ---------------------------------------------------------------------------
// cubeCounter — public entry point
// ---------------------------------------------------------------------------

void cubeCounter(const d4::OptionDpllStyleMethod& inputConfig,
                 const d4::OptionCubeCounter& optionCubeCounter,
                 const parser::Formula& formula) {
  d4::OptionDpllStyleMethod options = inputConfig;
  options.operationType = d4::OperationTypeManager::getOperatorType("counting");
  if (options.optionCacheManager.optionBucketManager.clauseRepresentation ==
      CACHE_INDEX)
    options.optionSpecManager.needFastNotSatisfied = true;

  // All clause indices for the full formula.
  std::vector<unsigned> allClauses;
  allClauses.reserve(formula.clauses.size());
  for (unsigned i = 0; i < formula.clauses.size(); i++) allClauses.push_back(i);

  d4::ProblemManager fullProblem = buildProblem(formula, allClauses, std::cout);

  const std::string strategy = optionCubeCounter.selectorStrategy.get();

  std::vector<unsigned> easyClauses;
  if (strategy == "iterative-primal-cut") {
    unsigned depth = (unsigned)optionCubeCounter.maxDepth.get();
    easyClauses =
        IterativePrimalCutSelector(depth, std::cout).select(formula);
  } else {
    if (strategy != "primal-cut")
      std::cerr << "c [CUBE-COUNTER] unknown strategy '" << strategy
                << "'; falling back to primal-cut\n";
    easyClauses = PrimalCutSelector(std::cout).select(formula);
  }

  cubeAndCount(options, fullProblem, formula, easyClauses);
}
