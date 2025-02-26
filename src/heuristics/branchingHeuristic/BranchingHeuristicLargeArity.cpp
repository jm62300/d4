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

#include "BranchingHeuristicLargeArity.hpp"

#include <algorithm>

#include "src/formulaManager/cnf/CnfManager.hpp"

namespace d4 {

/**
 * @brief BranchingHeuristicLargeArity::selectLitSet implementation.
 */
BranchingHeuristicLargeArity::BranchingHeuristicLargeArity(
    const OptionBranchingHeuristic &options, ProblemManager *problem,
    FormulaManager *specs, ActivityManager &activityManager,
    PolarityManager &polarityManager, std::ostream &out)
    : BranchingHeuristic(options, problem, specs, activityManager,
                         polarityManager, out) {
  m_limitClause = options.limitSizeClause;

  std::vector<std::vector<Lit>> &clauses =
      static_cast<CnfManager *>(specs)->getClauses();
  for (unsigned i = 0; i < clauses.size(); i++) {
    if (clauses[i].size() >= m_limitClause) m_indexOfLargeClause.push_back(i);
  }

  m_markedVar.resize(specs->getNbVariable() + 1, false);
  out << "c [BRANCHING HEURISTIC] The number of large clauses is: "
      << m_indexOfLargeClause.size() << '\n';
}  // constructor

/**
 * @brief BranchingHeuristicLargeArity::selectLitSet implementation.
 */
void BranchingHeuristicLargeArity::selectLitSet(std::vector<Var> &vars,
                                                ListLit &lits) {
  m_nbCall++;
  // decay the variable weights.
  if (m_freqDecay && !(m_nbCall % m_freqDecay)) m_hVar->decayCountConflict();

  for (auto &v : vars) m_markedVar[v] = true;

  // check if we still have a large enough clause.
  CnfManager *specs = static_cast<CnfManager *>(m_specs);
  unsigned larger = 0, lIdx = 0;
  for (auto &idx : m_indexOfLargeClause) {
    if (specs->getSize(idx) >= m_limitClause &&
        specs->isNotSatisfiedClauseAndInComponent(idx, m_markedVar)) {
      if (specs->getClause(idx).size() > larger) {
        larger = specs->getClause(idx).size();
        lIdx = idx;
      }
    }
  }

  if (larger) {
    // get the lits.
    Lit tmp[larger];
    unsigned size = 0;
    for (auto &l : specs->getClause(lIdx)) {
      assert(l.var() < m_markedVar.size());
      if (m_markedVar[l.var()]) {
        assert(l.var() >= 0);
        tmp[size++] = l;
      }
    }

    // sort the lits.
    std::sort(tmp, &tmp[size], [&](Lit a, Lit b) {
      return m_hVar->computeScore(a.var()) > m_hVar->computeScore(b.var());
    });

    // fill the given structure.
    lits.setListLit(tmp, size, m_listLitAllocator);
  } else {
    Var v = var_Undef;
    double bestScore = -1;

    for (auto &vTmp : vars) {
      if (m_specs->varIsAssigned(vTmp) || !m_isDecisionVariable[vTmp]) continue;

      double current = m_hVar->computeScore(vTmp);
      if (v == var_Undef || current > bestScore) {
        v = vTmp;
        bestScore = current;
      }
    }

    if (v != var_Undef) {
      Lit tmp[1] = {Lit::makeLit(v, m_hPhase->selectPhase(v))};
      lits.setListLit(tmp, 1, m_listLitAllocator);
    } else {
      lits.setListLit(NULL, 0, m_listLitAllocator);
    }
  }

  // reinit the marker.
  for (auto &v : vars) m_markedVar[v] = false;
}  // selectLitSet

}  // namespace d4