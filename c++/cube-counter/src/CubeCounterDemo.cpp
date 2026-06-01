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
 * @brief Multi-step resolution into V_easy.
 *
 * BFS over resolution derivations: starting from every clause in F that has
 * at least one variable outside V_easy, repeatedly resolve on outside-variable
 * pivots using original F clauses until the derived clause is fully within
 * V_easy (success) or the step budget is exhausted (pruned).
 *
 * Guards:
 *  - maxSteps: max resolution steps per derivation path.
 *  - maxNewClauses: stop after collecting this many new V_easy clauses.
 *  - kMaxClauseSize: discard resolvents larger than this (prevents blowup).
 *  - visited set: each distinct intermediate clause is explored at most once.
 */
static std::vector<std::vector<int>> strengthenByResolution(
    const parser::Formula& formula, const std::vector<bool>& inVeasy,
    int maxSteps, int maxNewClauses, std::ostream& out) {
  const unsigned nbVar = formula.nbVar;
  const auto& clauses = formula.clauses;
  static constexpr int kMaxClauseSize = 20;

  auto litIdx = [](int l) -> unsigned {
    return l > 0 ? (unsigned)(2 * l) : (unsigned)(2 * (-l) + 1);
  };
  std::vector<std::vector<unsigned>> litToClauses(2 * nbVar + 2);
  for (unsigned i = 0; i < clauses.size(); i++)
    for (int l : clauses[i]) litToClauses[litIdx(l)].push_back(i);

  // All original clauses (sorted) — used to skip already-known clauses.
  std::set<std::vector<int>> existing;
  for (const auto& cl : clauses) {
    auto s = cl;
    std::sort(s.begin(), s.end());
    existing.insert(s);
  }

  // visited prevents re-exploring the same intermediate clause.
  std::set<std::vector<int>> visited;

  // BFS queue: (sorted clause, steps_remaining).
  std::deque<std::pair<std::vector<int>, int>> queue;

  for (const auto& cl : clauses) {
    bool hasOutside = false;
    for (int l : cl)
      if (!inVeasy[std::abs(l)]) {
        hasOutside = true;
        break;
      }
    if (!hasOutside) continue;
    auto s = cl;
    std::sort(s.begin(), s.end());
    if (visited.insert(s).second) queue.push_back({s, maxSteps});
  }

  out << "c [STRENGTHEN-RES] starting BFS: seed_clauses=" << queue.size()
      << " max_steps=" << maxSteps << " max_new=" << maxNewClauses << "\n";
  out.flush();

  std::vector<std::vector<int>> result;
  unsigned long long iters = 0;
  static constexpr unsigned long long kLogEvery = 50000;

  while (!queue.empty() && (int)result.size() < maxNewClauses) {
    auto [current, stepsLeft] = queue.front();
    queue.pop_front();
    iters++;

    if (iters % kLogEvery == 0) {
      out << "c [STRENGTHEN-RES] iters=" << iters
          << " found=" << result.size() << "/" << maxNewClauses
          << " queue=" << queue.size()
          << " visited=" << visited.size() << "\n";
      out.flush();
    }

    // Find the first outside-variable pivot.
    int pivot = 0;
    for (int l : current)
      if (!inVeasy[std::abs(l)]) {
        pivot = l;
        break;
      }

    if (!pivot) {
      // Clause is fully in V_easy.
      if (!existing.count(current)) {
        result.push_back(current);
        existing.insert(current);
        out << "c [STRENGTHEN-RES] new clause #" << result.size()
            << " (size=" << current.size() << "): ";
        for (int l : current) out << l << ' ';
        out << "\n";
        out.flush();
      }
      continue;
    }

    if (stepsLeft == 0) continue;

    // Resolve current on pivot with every F clause containing ¬pivot.
    for (unsigned j : litToClauses[litIdx(-pivot)]) {
      std::map<int, bool> res;
      bool taut = false;
      for (int l : current) {
        if (std::abs(l) == std::abs(pivot)) continue;
        if (res.count(-l)) {
          taut = true;
          break;
        }
        res[l] = true;
      }
      if (taut) continue;
      for (int l : clauses[j]) {
        if (std::abs(l) == std::abs(pivot)) continue;
        if (res.count(-l)) {
          taut = true;
          break;
        }
        res[l] = true;
      }
      if (taut) continue;
      if ((int)res.size() > kMaxClauseSize) continue;

      std::vector<int> resolvent;
      resolvent.reserve(res.size());
      for (auto& [l, _] : res) resolvent.push_back(l);
      std::sort(resolvent.begin(), resolvent.end());

      if (visited.insert(resolvent).second)
        queue.push_back({resolvent, stepsLeft - 1});
    }
  }

  out << "c [STRENGTHEN-RES] max_steps=" << maxSteps
      << " max_new=" << maxNewClauses << " new_clauses=" << result.size()
      << " queue_remaining=" << queue.size() << "\n";
  return result;
}

/**
 * @brief Exact projection of F onto V_easy via bounded variable elimination
 *        (Davis-Putnam procedure).
 *
 * For each outside variable v (not in V_easy), all clauses mentioning v are
 * replaced by their pairwise resolvents on v, then v is dropped.  This
 * computes the exact V_easy projection for every eliminated variable.
 *
 * Cost control:
 *   - Pure variables (appear only positive or only negative) are eliminated
 *     for free — their clauses impose no V_easy constraint and are dropped.
 *   - For the rest, elimination is only performed when |P|×|N| ≤ maxProduct.
 *   - Each round picks the cheapest remaining variable (min-fill heuristic).
 *   - The loop terminates when no variable satisfies the budget.
 *
 * Why this works when BFS resolution does not: BFS tries to build derivation
 * paths from individual seed clauses; it misses derivations that go through
 * intermediate resolvents and explodes when branching is wide.  Variable
 * elimination processes each outside variable completely in one shot —
 * all information about v is folded into resolvents before v is removed.
 */
static std::vector<std::vector<int>> strengthenByElimination(
    const parser::Formula& formula, const std::vector<bool>& inVeasy,
    int maxProduct, std::ostream& out) {
  const unsigned nbVar = formula.nbVar;

  // Working formula — deduplicated, every clause stored sorted.
  std::set<std::vector<int>> workingSet;
  for (const auto& cl : formula.clauses) {
    auto s = cl;
    std::sort(s.begin(), s.end());
    workingSet.insert(s);
  }

  // eliminated[v] = true means v no longer appears in workingSet.
  std::vector<bool> done(nbVar + 1, false);
  for (unsigned v = 1; v <= nbVar; v++)
    if (inVeasy[v]) done[v] = true;  // V_easy vars are kept, not eliminated

  out << "c [STRENGTHEN-VE] start formula_size=" << workingSet.size()
      << " max_product=" << maxProduct << "\n";
  out.flush();

  unsigned nEliminated = 0;

  while (true) {
    // Count positive/negative occurrences for each remaining outside var.
    std::vector<int> posCount(nbVar + 1, 0), negCount(nbVar + 1, 0);
    for (const auto& cl : workingSet)
      for (int l : cl) {
        unsigned v = (unsigned)std::abs(l);
        if (!done[v]) (l > 0 ? posCount[v] : negCount[v])++;
      }

    // Pick cheapest outside variable: pure vars first (cost 0), then min |P|×|N|.
    unsigned bestV = 0;
    long bestCost = (long)maxProduct + 1;
    for (unsigned v = 1; v <= nbVar; v++) {
      if (done[v]) continue;
      long cost = (posCount[v] == 0 || negCount[v] == 0)
                      ? 0
                      : (long)posCount[v] * negCount[v];
      if (cost <= maxProduct && cost < bestCost) {
        bestCost = cost;
        bestV = v;
      }
    }
    if (!bestV) break;

    // Partition: clauses with +v, clauses with -v, the rest.
    std::vector<std::vector<int>> P, N;
    std::set<std::vector<int>> nextSet;
    for (const auto& cl : workingSet) {
      bool hasPos = false, hasNeg = false;
      for (int l : cl) {
        if (l == (int)bestV) hasPos = true;
        if (l == -(int)bestV) hasNeg = true;
      }
      if (!hasPos && !hasNeg) nextSet.insert(cl);  // unaffected
      else if (hasPos) P.push_back(cl);
      else N.push_back(cl);
    }

    out << "c [STRENGTHEN-VE] elim var=" << bestV << " |P|=" << P.size()
        << " |N|=" << N.size() << " rest=" << nextSet.size() << "\n";
    out.flush();

    // Resolve every P clause against every N clause on bestV.
    unsigned newResolvents = 0;
    for (const auto& cp : P) {
      for (const auto& cn : N) {
        std::map<int, bool> res;
        bool taut = false;
        for (int l : cp) {
          if ((unsigned)std::abs(l) == bestV) continue;
          if (res.count(-l)) { taut = true; break; }
          res[l] = true;
        }
        if (taut) continue;
        for (int l : cn) {
          if ((unsigned)std::abs(l) == bestV) continue;
          if (res.count(-l)) { taut = true; break; }
          res[l] = true;
        }
        if (taut) continue;
        std::vector<int> r;
        r.reserve(res.size());
        for (auto& [l, _] : res) r.push_back(l);
        std::sort(r.begin(), r.end());
        if (nextSet.insert(r).second) newResolvents++;
      }
    }

    out << "c [STRENGTHEN-VE]   new_resolvents=" << newResolvents
        << " formula_size=" << nextSet.size() << "\n";
    out.flush();

    workingSet = std::move(nextSet);
    done[bestV] = true;
    nEliminated++;
  }

  // Extract clauses that are new and fully within V_easy.
  std::set<std::vector<int>> original;
  for (const auto& cl : formula.clauses) {
    auto s = cl;
    std::sort(s.begin(), s.end());
    original.insert(s);
  }

  std::vector<std::vector<int>> result;
  for (const auto& cl : workingSet) {
    bool allInVeasy = true;
    for (int l : cl)
      if (!inVeasy[(unsigned)std::abs(l)]) { allInVeasy = false; break; }
    if (allInVeasy && !original.count(cl)) result.push_back(cl);
  }

  out << "c [STRENGTHEN-VE] eliminated=" << nEliminated
      << " new_veasy_clauses=" << result.size() << "\n";
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
    double minBits = optionCubeCounter.hdMinBits.get();
    easyClauses =
        HighDegreeVariableSelector(ratio, minBits, std::cout).select(formula);
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
    extraClauses =
        strengthenByResolution(formula, inVeasy,
                               optionCubeCounter.strengthenSteps.get(),
                               optionCubeCounter.strengthenMaxClauses.get(),
                               std::cout);
  } else if (strengthen == "elimination") {
    extraClauses = strengthenByElimination(
        formula, inVeasy, optionCubeCounter.strengthenMaxProduct.get(),
        std::cout);
  } else if (strengthen == "sat") {
    extraClauses =
        strengthenBySat(formula, easyClauses, inVeasy,
                        optionCubeCounter.strengthenTime.get(), std::cout);
  }

  cubeAndCount(options, fullProblem, formula, easyClauses, extraClauses);
}
