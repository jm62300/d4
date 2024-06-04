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

  // select one variable.
  Var v = var_Undef;
  double bestScore = -1;

  // stat on the current cut set.
  unsigned minLevel = -1, maxLevel = 0;
  unsigned nbMin = 0;
  for (auto &vTmp : vars) {
    if (m_specs->varIsAssigned(vTmp) || !m_isDecisionVariable[vTmp]) continue;
    if (minLevel > m_partialOrder->getPartialOrder(vTmp)) {
      minLevel = m_partialOrder->getPartialOrder(vTmp);
    }
    if (minLevel == m_partialOrder->getPartialOrder(vTmp)) nbMin++;

    if (maxLevel < m_partialOrder->getPartialOrder(vTmp))
      maxLevel = m_partialOrder->getPartialOrder(vTmp);
  }

  for (auto &vTmp : vars) {
    assert(m_partialOrder->getPartialOrder(vTmp));
    if (m_specs->varIsAssigned(vTmp) || !m_isDecisionVariable[vTmp]) continue;

    if (v == var_Undef) {
      bestScore = m_hVar->computeScore(vTmp) +
                  m_partialOrder->scaleFactor() *
                      (maxLevel - m_partialOrder->getPartialOrder(vTmp));
      v = vTmp;
      continue;
    }

    double current = m_hVar->computeScore(vTmp);
    current += m_partialOrder->scaleFactor() *
               (maxLevel - m_partialOrder->getPartialOrder(vTmp));

    if (v == var_Undef || current > bestScore) {
      v = vTmp;
      bestScore = current;
    }
  }

  // return the list of lit (here it contains one literal).
  if (v != var_Undef) {
    Lit tmp[] = {Lit::makeLit(v, m_hPhase->selectPhase(v))};
    lits.setListLit(tmp, 1);
  } else {
    lits.setSize(0);
    lits.setArray(NULL);
  }
}  // selectLitSet

}  // namespace d4