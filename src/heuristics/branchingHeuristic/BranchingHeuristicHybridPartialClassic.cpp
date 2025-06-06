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
  Var vCut = var_Undef, vBest = var_Undef;
  double bestScore = -1, bestScoreCut = -1;

  unsigned minLevel = -1;
  unsigned nbMin = 0;
  for (auto &vTmp : vars) {
    if (m_specs->varIsAssigned(vTmp) || !m_isDecisionVariable[vTmp]) continue;

    if (minLevel > m_partialOrder->getPartialOrder(vTmp)) {
      minLevel = m_partialOrder->getPartialOrder(vTmp);
      vCut = var_Undef;
    }
    if (minLevel == m_partialOrder->getPartialOrder(vTmp)) nbMin++;

    double current = m_hVar->computeScore(vTmp);
    if (vCut == var_Undef ||
        (m_partialOrder->getPartialOrder(vTmp) == minLevel &&
         bestScoreCut < current)) {
      vCut = vTmp;
      bestScoreCut = current;
    }

    if (vBest == var_Undef || bestScore < current) {
      vBest = vTmp;
      bestScore = current;
    }
  }

  Var v = vCut;
  if (nbMin > WORTH_CUT &&
      (bestScoreCut * m_partialOrder->scaleFactor() / nbMin) < bestScore)
    v = vBest;

  // return the list of lit (here it contains one literal).
  if (v != var_Undef) {
    Lit tmp[] = {Lit::makeLit(v, m_hPhase->selectPhase(v))};
    lits.setListLit(tmp, 1, m_listLitAllocator);
  } else {
    lits.setListLit(NULL, 0, m_listLitAllocator);
  }
}  // selectLitSet

/**
 * @brief BranchingHeuristicHybridPartialClassic::updateHeuristic
 * implementation.
 */
void BranchingHeuristicHybridPartialClassic::updateHeuristic(
    std::vector<Var> &vars) {
  m_partialOrder->init(m_saveOptionPartialOrderHeuristic, *m_specs,
                       std::cout);  // normally that should print nothing.
  // std::cout << "coucou\n";
}  // updateHeuristic

}  // namespace d4