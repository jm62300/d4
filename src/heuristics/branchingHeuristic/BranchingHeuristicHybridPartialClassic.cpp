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

#include "BranchingHeuristicHybridPartialClassic.hpp"

namespace d4 {

/**
 * @brief BranchingHeuristicHybridPartialClassic::
    ~BranchingHeuristicHybridPartialClassic implementation.
 */
BranchingHeuristicHybridPartialClassic::
    ~BranchingHeuristicHybridPartialClassic() {
  delete m_partialOrder;
}  // destructor.

/**
 * @brief BranchingHeuristicHybridPartialClassic::selectLitSet implementation.
 */
void BranchingHeuristicHybridPartialClassic::selectLitSet(
    std::vector<Var> &vars, ListLit &lits) {
  m_nbCall++;

  // decay the variable weights.
  if (m_freqDecay && !(m_nbCall % m_freqDecay)) m_hVar->decayCountConflict();

  // get the best variables (in the cut and in general).
  Var vBest = var_Undef;
  double bestScore = -1;

  for (auto &vTmp : vars) {
    if (m_specs->varIsAssigned(vTmp) || !m_isDecisionVariable[vTmp]) continue;

    double current =
        m_hVar->computeScore(vTmp) + m_partialOrder->getPartialOrder(vTmp);

    if (vBest == var_Undef || bestScore < current) {
      vBest = vTmp;
      bestScore = current;
    }
  }

  // return the list of lit (here it contains one literal).
  if (vBest != var_Undef) {
    Lit tmp[] = {Lit::makeLit(vBest, m_hPhase->selectPhase(vBest))};
    lits.setListLit(tmp, 1, m_listLitAllocator);
  } else {
    lits.setListLit(NULL, 0, m_listLitAllocator);
  }
}  // selectLitSet
}  // namespace d4