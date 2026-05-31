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
#include "selectors/HighDegreeVariableSelector.hpp"
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
 * @brief Build a ProblemManager from a subset of formula clauses plus
 *        optional extra clauses (expressed as DIMACS literal vectors).
 */
static d4::ProblemManager buildProblem(
    const parser::Formula& formula, const std::vector<unsigned>& clauseIdxs,
    std::ostream& out, const std::vector<std::vector<int>>& extraClauses = {}) {
  std::vector<d4::BcGate> gates;
  gates.reserve(clauseIdxs.size() + extraClauses.size());

  auto addClause = [&](const std::vector<int>& cl) {
    std::vector<d4::Lit> d4Clause;
    for (int l : cl) d4Clause.push_back(d4::Lit::makeLit(std::abs(l), l < 0));
    gates.push_back({d4Clause, d4::lit_Undef, BcGateType::CLAUSE});
  };

  for (unsigned idx : clauseIdxs) addClause(formula.clauses[idx]);
  for (const auto& cl : extraClauses) addClause(cl);

  std::map<d4::Lit, std::string> weightMap;
  for (const auto& [lit, weight] : formula.weightMap)
    weightMap[d4::Lit::makeLit(std::abs(lit), lit < 0)] = weight;

  return d4::ProblemManager(formula.type, formula.nbVar,
                            formula.quantifications, weightMap, gates, out);
}

// ---------------------------------------------------------------------------
// Strengthening helpers
// ---------------------------------------------------------------------------

/**
 * @brief Single-step resolution into V_easy.
 *
 * For each clause in F with exactly one variable outside V_easy, resolve it
 * with every clause containing the negation of that variable. Keep resolvents
 * whose variable support is entirely within V_easy.
 */
static std::vector<std::vector<int>> strengthenByResolution(
    const parser::Formula& formula, const std::vector<bool>& inVeasy,
    std::ostream& out) {
  const unsigned nbVar = formula.nbVar;
  const auto& clauses = formula.clauses;

  // literal → clause indices  (positive lit l → index 2l, negative → 2|l|+1)
  auto litIdx = [](int l) -> unsigned {
    return l > 0 ? (unsigned)(2 * l) : (unsigned)(2 * (-l) + 1);
  };
  std::vector<std::vector<unsigned>> litToClauses(2 * nbVar + 2);
  for (unsigned i = 0; i < clauses.size(); i++)
    for (int l : clauses[i]) litToClauses[litIdx(l)].push_back(i);

  // Existing clauses (to avoid adding duplicates)
  std::set<std::vector<int>> existing;
  for (const auto& cl : clauses) {
    auto s = cl;
    std::sort(s.begin(), s.end());
    existing.insert(s);
  }

  std::set<std::vector<int>> newSet;

  for (unsigned i = 0; i < clauses.size(); i++) {
    int outsideLit = 0;
    bool moreThanOne = false;
    for (int l : clauses[i]) {
      if (!inVeasy[std::abs(l)]) {
        if (outsideLit) {
          moreThanOne = true;
          break;
        }
        outsideLit = l;
      }
    }
    if (moreThanOne || !outsideLit) continue;

    for (unsigned j : litToClauses[litIdx(-outsideLit)]) {
      std::map<int, bool> res;
      bool taut = false;
      for (int l : clauses[i]) {
        if (std::abs(l) == std::abs(outsideLit)) continue;
        if (res.count(-l)) {
          taut = true;
          break;
        }
        res[l] = true;
      }
      if (taut) continue;
      for (int l : clauses[j]) {
        if (std::abs(l) == std::abs(outsideLit)) continue;
        if (res.count(-l)) {
          taut = true;
          break;
        }
        res[l] = true;
      }
      if (taut) continue;

      bool allInVeasy = true;
      for (auto& [l, _] : res)
        if (!inVeasy[std::abs(l)]) {
          allInVeasy = false;
          break;
        }
      if (!allInVeasy) continue;

      std::vector<int> resolvent;
      for (auto& [l, _] : res) resolvent.push_back(l);
      std::sort(resolvent.begin(), resolvent.end());
      if (!existing.count(resolvent)) newSet.insert(resolvent);
    }
  }

  std::vector<std::vector<int>> result(newSet.begin(), newSet.end());
  out << "c [STRENGTHEN-RES] new clauses=" << result.size() << "\n";
  return result;
}

/**
 * @brief SAT-based strengthening: enumerate models of F_easy, and for each
 *        that is UNSAT w.r.t. F, extract the minimal failing core and add
 *        its negation as a new clause. Only these negated cores are returned
 *        (they are implied by F, so adding them to F_easy preserves
 *        correctness). Stops after timeLimitSec seconds.
 */
static std::vector<std::vector<int>> strengthenBySat(
    const parser::Formula& formula, const std::vector<unsigned>& easyClauses,
    const std::vector<bool>& inVeasy, double timeLimitSec, std::ostream& out) {
  const unsigned nbVar = formula.nbVar;
  const auto& clauses = formula.clauses;

  std::vector<unsigned> veasyVars;
  for (unsigned v = 1; v <= nbVar; v++)
    if (inVeasy[v]) veasyVars.push_back(v);

  // Solver for enumerating models of F_easy (gets all blocking clauses).
  CaDiCaL::Solver enumSolver;
  enumSolver.set("quiet", 1);
  enumSolver.declare_more_variables(nbVar);
  for (unsigned idx : easyClauses) {
    for (int l : clauses[idx]) enumSolver.add(l);
    enumSolver.add(0);
  }

  // Solver for checking consistency with the full formula F.
  CaDiCaL::Solver fullSolver;
  fullSolver.set("quiet", 1);
  fullSolver.declare_more_variables(nbVar);
  for (const auto& cl : clauses) {
    for (int l : cl) fullSolver.add(l);
    fullSolver.add(0);
  }

  auto wallStart = std::chrono::steady_clock::now();
  std::vector<std::vector<int>> newClauses;
  unsigned unsatBlocked = 0, satSeen = 0;

  while (true) {
    double elapsed = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - wallStart)
                         .count();
    if (elapsed >= timeLimitSec) break;
    if (enumSolver.solve() != 10) break;  // F_easy exhausted

    // Extract assignment over V_easy.
    std::vector<int> sigma;
    for (unsigned v : veasyVars)
      sigma.push_back(enumSolver.val(v) > 0 ? (int)v : -(int)v);

    // Check consistency with F.
    fullSolver.reset_assumptions();
    for (int l : sigma) fullSolver.assume(l);

    std::vector<int> blockClause;
    if (fullSolver.solve() == 20) {  // UNSAT w.r.t. F
      unsatBlocked++;
      for (int l : sigma)
        if (fullSolver.failed(l)) blockClause.push_back(-l);
      newClauses.push_back(blockClause);  // implied by F — safe to add
    } else {
      satSeen++;
      for (int l : sigma)
        blockClause.push_back(-l);  // block only in enumSolver
    }

    for (int l : blockClause) enumSolver.add(l);
    enumSolver.add(0);
  }

  double elapsed = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - wallStart)
                       .count();
  out << "c [STRENGTHEN-SAT] new_clauses=" << newClauses.size()
      << " unsat_blocked=" << unsatBlocked << " sat_seen=" << satSeen
      << " time=" << std::fixed << std::setprecision(2) << elapsed << "s\n";
  return newClauses;
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
static void cubeAndCount(
    const OptionDpllStyleMethod& inputConfig, const ProblemManager& fullProblem,
    const parser::Formula& formula, const std::vector<unsigned>& easyClauses,
    const std::vector<std::vector<int>>& extraClauses = {}) {
  d4::OptionDpllStyleMethod options = inputConfig;
  options.operationType = d4::OperationTypeManager::getOperatorType("counting");
  if (options.optionCacheManager.optionBucketManager.clauseRepresentation ==
      CACHE_INDEX)
    options.optionSpecManager.needFastNotSatisfied = true;

  std::cout << "c [CUBE-COUNTER] F_easy: " << easyClauses.size() << '/'
            << formula.clauses.size() << " clauses\n";

  for (auto idx : easyClauses) {
    std::cout << "  [clause" << idx << "]";
    for (auto l : formula.clauses[idx]) std::cout << l << ' ';
    std::cout << '\n';
  }

  // --- Compile F_easy (plus any strengthening clauses) ---
  d4::ProblemManager easyProblem =
      buildProblem(formula, easyClauses, std::cout, extraClauses);

  auto* easyCompiler =
      new DpllStyleMethod<semiring::Node, semiring::DecDNNFSemiring>(
          options, easyProblem, std::cout);
  semiring::Node D_easy = easyCompiler->run();
  const semiring::DecDNNFSemiring& sem = easyCompiler->getSemiring();

  std::cout << "c [CUBE-COUNTER] F_easy DNNF nodes=" << sem.getNbNodes()
            << " edges=" << sem.getNbEdges() << '\n';
#if 1
  unsigned testCount = 0;
  sem.enumeratePartialModels(D_easy, {}, formula.nbVar,
                             [&](std::vector<d4::Lit>& sigma) {
                               testCount++;
                               return;
                             });
  std::cout << "the number of cubes is " << testCount << '\n';

  return;
#endif
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
        std::cout << cubeCount << "the size of sigma is " << sigma.size()
                  << "\n";

        cadical.reset_assumptions();
        for (auto& l : sigma) cadical.assume(l.human());

        if (cadical.solve() == 20) {
          std::cout << "it is unsat\n";
          prunedCount++;
          return;
        }

        std::cout << "it is SAT then we have to count ...\n";

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
    easyClauses = IterativePrimalCutSelector(depth, std::cout).select(formula);
  } else if (strategy == "high-degree") {
    double ratio = optionCubeCounter.targetRatio.get();
    easyClauses = HighDegreeVariableSelector(ratio, std::cout).select(formula);
  } else {
    if (strategy != "primal-cut")
      std::cerr << "c [CUBE-COUNTER] unknown strategy '" << strategy
                << "'; falling back to primal-cut\n";
    easyClauses = PrimalCutSelector(std::cout).select(formula);
  }

  // Compute V_easy once — reused by extendEasy and strengthen.
  std::vector<bool> inVeasy(formula.nbVar + 1, false);
  for (unsigned idx : easyClauses)
    for (int l : formula.clauses[idx]) inVeasy[std::abs(l)] = true;

  if (optionCubeCounter.extendEasy.get()) {
    std::vector<bool> selected(formula.clauses.size(), false);
    for (unsigned idx : easyClauses) selected[idx] = true;

    unsigned added = 0;
    for (unsigned i = 0; i < formula.clauses.size(); i++) {
      if (selected[i]) continue;
      bool fits = true;
      for (int l : formula.clauses[i])
        if (!inVeasy[std::abs(l)]) {
          fits = false;
          break;
        }
      if (fits) {
        easyClauses.push_back(i);
        added++;
      }
    }
    if (added)
      std::cout << "c [CUBE-COUNTER] extendEasy added " << added
                << " clauses (total easy=" << easyClauses.size() << "/"
                << formula.clauses.size() << ")\n";
  }

  // Derive new clauses over V_easy implied by F, to prune UNSAT cubes.
  std::vector<std::vector<int>> extraClauses;
  const std::string strengthen = optionCubeCounter.strengthen.get();
  if (strengthen == "resolution") {
    extraClauses = strengthenByResolution(formula, inVeasy, std::cout);
  } else if (strengthen == "sat") {
    extraClauses =
        strengthenBySat(formula, easyClauses, inVeasy,
                        optionCubeCounter.strengthenTime.get(), std::cout);
  }

  cubeAndCount(options, fullProblem, formula, easyClauses, extraClauses);
}
