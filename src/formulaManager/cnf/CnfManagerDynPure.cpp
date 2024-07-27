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

#include "CnfManagerDynPure.hpp"

namespace d4 {

/**
 * @brief CnfManagerDynPure::CnfManagerDynPure implementation.
 */
CnfManagerDynPure::CnfManagerDynPure(ProblemManager &p) : CnfManagerDyn(p) {
  std::cout << "c [SPEC MANAGER] DYN with pure literal elimination\n";
  m_nbPureSimplification = 0;

  m_markedPureLiteral.resize(1 + p.getNbVar(), false);
  m_pureDetected.reserve(1 + p.getNbVar());

  m_isDecisionVariable.resize(p.getNbVar() + 1, !p.getNbSelectedVar());
  for (auto v : p.getSelectedVar()) m_isDecisionVariable[v] = true;

  affectInitPureLit();
}  // CnfManagerDynPure

/**
 * @brief CnfManagerDynPure::affectInitPureLit implementation.
 */
void CnfManagerDynPure::affectInitPureLit() {
  getPureLiterals(m_pureDetected);
  m_stackPosClause.push_back(m_savedStateClauses.size());
  m_stackPosOcc.push_back(m_savedStateOccs.size());
  propagateTrue(m_pureDetected);

  m_nbPureSimplification += m_pureDetected.size();
}  // affectInitPureLit

/**
 * @brief CnfManagerDynPure::getPureLiterals implementation.
 */
void CnfManagerDynPure::getPureLiterals(std::vector<Lit> &pureLits) {
  for (unsigned i = 1; i < m_isDecisionVariable.size(); i++)
    if (m_isDecisionVariable[i] || m_currentValue[i] != l_Undef)
      continue;
    else {
      Lit l = Lit::makeLit(i, false);
      if (m_occurrence[l.intern()].size() &&
          !m_occurrence[(~l).intern()].size())
        pureLits.push_back(l);
      if (!m_occurrence[l.intern()].size() &&
          m_occurrence[(~l).intern()].size())
        pureLits.push_back(~l);
    }
}  // getPureLiterals

/**
 * @brief CnfManagerDynPure::inprocessing implementation.
 */
void CnfManagerDynPure::inprocessing() {
  // consider the pure literals.
  do {
    m_pureDetected.resize(0);
    for (unsigned i = m_stackPosOcc.back(); i < m_savedStateOccs.size(); i++) {
      Lit &l = m_savedStateOccs[i].l;
      if (m_isDecisionVariable[l.var()] || m_currentValue[l.var()] != l_Undef)
        continue;

      if (!m_occurrence[l.intern()].size() &&
          m_occurrence[(~l).intern()].size()) {
        m_pureDetected.push_back(~l);
      }
    }

    propagateTrue(m_pureDetected);
    m_nbPureSimplification += m_pureDetected.size();

  } while (m_pureDetected.size());
}  // searchPureLitOnTheStack

}  // namespace d4
