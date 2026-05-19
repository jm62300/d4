/**
 * eliminator
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "PreprocManager.hpp"

#include "src/bipartition/methods/Backbone.hpp"
#include "src/bipartition/methods/DACircuit.hpp"
#include "src/eliminator/EliminatorGates.hpp"
#include "src/reducer/Method.hpp"
#include "src/utils/Problem.hpp"

namespace bipe {

/**
 * @brief Executes a comprehensive preprocessing phase on the formula.
 *
 * This method applies a heavy preprocessing pipeline consisting of:
 * 1. Backbone extraction (finding forced unit literals).
 * 2. Equivalence detection (finding variables that imply each other).
 * 3. Literal substitution (replacing equivalent variables globally).
 * 4. Clause reduction (vivification, occurrence elimination, etc.).
 *
 * @param[in,out] problem The SAT problem to be simplified in-place.
 * @param[in] optionPreproc Configuration options controlling limits, timeouts,
 * and verbosity for the preprocessing phase.
 */
void preprocEquivFull(Problem& problem, const OptionPreproc& optionPreproc) {
  if (optionPreproc.verbose) std::cout << "c [PREPROC] Preproc Equiv\n";

  Timer timer(optionPreproc.timeout);
  if (optionPreproc.timeout != 0) timer.start();

  // --- PHASE 1: Compute and apply the Backbone ---
  bipartition::Backbone backbone;
  std::vector<Gate> gates;
  std::vector<std::vector<bool>> setOfModels;
  backbone.run(problem, gates, setOfModels, optionPreproc.optionBackone,
               std::cout, timer);

  // Force backbone literals to be true by adding them as unit clauses
  for (auto& g : gates) {
    problem.getClauses().push_back(
        {Lit::makeLit(g.output.var(), g.output.sign())});
  }

  // --- PHASE 2: Compute Equivalences ---
  bipartition::DACircuit equiv;
  bipartition::OptionDac optionDac = optionPreproc.optionDac;
  // Disable OR/XOR detection to focus strictly on pure equivalences
  optionDac.computeOr = false;
  optionDac.computeXor = false;
  equiv.run(problem, gates, setOfModels, optionDac, std::cout, timer);

  // --- PHASE 3: Fast Literal Substitution ---
  // Create a flat array for O(1) literal substitution lookups
  std::vector<Lit> substitution_map(2 * problem.getNbVar() + 2, lit_Undef);
  for (unsigned i = 0; i < substitution_map.size(); ++i) {
    substitution_map[i].m_x = i;  // Default: literal maps to itself
  }

  // Populate the map with the discovered equivalences
  for (const auto& g : gates) {
    if (g.type == TypeGate::EQUIV && !g.input.empty()) {
      substitution_map[g.output.intern()] = g.input[0];
      substitution_map[(~g.output).intern()] = ~g.input[0];
    }
  }

  // Apply substitutions across all clauses in a single fast pass
  for (auto& clause : problem.getClauses()) {
    for (auto& lit : clause) {
      lit = substitution_map[lit.intern()];
    }
  }

  // Reintegrate Equivalences as Binary Clauses ---
  // To preserve the logical constraints of the substituted variables
  // (and allow solvers to deduce their values), we add them back as binary
  // clauses.
  for (auto& g : gates) {
    if (g.type == TypeGate::EQUIV) {
      problem.getClauses().push_back({~g.output, g.input[0]});
      problem.getClauses().push_back({g.output, ~g.input[0]});
    }
  }

  // PHASE 5: Clause Reduction ---
  // Instantiate the reducer method using the configured factory name
  reducer::Method* rm = bipe::reducer::Method::makeMethod(
      optionPreproc.optionReducer.reducerName, std::cout);

  // Execute the reducer to clean up redundancies (like duplicated literals)
  rm->run(problem.getNbVar(), problem.getClauses(),
          optionPreproc.optionReducer.nbIterarion, optionPreproc.verbose,
          problem.getClauses(), timer);

  delete rm;
}  // preprocEquivFull

/**
 * @brief Executes a lightweightEquiv preprocessing phase on the given problem.
 *
 * This method is considered "light" because it avoids the heavy computational
 * overhead of invoking a full SAT solver. Instead, it relies on a dedicated
 * clause reducer (using a combination of fast reduction techniques like
 * vivification or occurrence elimination) to simplify the formula in-place.
 *
 * @param[in,out] problem The SAT problem containing the clauses to be
 * simplified. The simplified clauses will replace the original ones.
 * @param[in] optionPreproc Configuration options controlling limits, timeouts,
 *                          and verbosity for the preprocessing phase.
 */
void preprocLightEquiv(Problem& problem, const OptionPreproc& optionPreproc) {
  // Initialize and start the execution timer based on user options
  if (optionPreproc.verbose) std::cout << "c [PREPROC] Preproc Light\n";

  Timer timer(optionPreproc.timeout);
  if (optionPreproc.timeout != 0) timer.start();

  // Instantiate the reducer method using a factory
  reducer::Method* rm = bipe::reducer::Method::makeMethod(
      optionPreproc.optionReducer.reducerName, std::cout);

  // Execute the reducer on the problem's clauses.
  rm->run(problem.getNbVar(), problem.getClauses(),
          optionPreproc.optionReducer.nbIterarion, optionPreproc.verbose,
          problem.getClauses(), timer);

  delete rm;
}  // preprocLightEquiv

/**
 * @brief PreprocManager::run implementation.
 */
void PreprocManager::run(unsigned nbVar, std::vector<std::vector<int>>& clauses,
                         std::vector<int> projected, std::vector<int> protect,
                         const PreprocMethod& preprocMethod,
                         const OptionPreproc& optionPreproc) {
  Problem problem(nbVar, clauses, projected, protect);

  switch (preprocMethod) {
    case EQUIV_LIGHT:
      preprocLightEquiv(problem, optionPreproc);
      break;
    case EQUIV_FULL:
      preprocEquivFull(problem, optionPreproc);
      break;
    case NONE:
      if (optionPreproc.verbose) std::cout << "c [PREPROC] None\n";
      return;
  }

  // get the new clauses.
  clauses.clear();
  clauses.reserve(problem.getClauses().size());
  for (auto& cl : problem.getClauses()) {
    std::vector<int> newcl;
    for (auto& l : cl) newcl.push_back(l.human());
    clauses.push_back(newcl);
  }
}  // run
}  // namespace bipe