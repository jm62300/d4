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

#include "SpecManagerCnfDynBlockedCl.hpp"

namespace d4 {

/**
 * @brief SpecManagerCnfDynBlockedCl::SpecManagerCnfDynBlockedCl implementation.
 */
SpecManagerCnfDynBlockedCl::SpecManagerCnfDynBlockedCl(ProblemManager &p)
    : SpecManagerCnfDyn(p) {
  std::cout << "c [SPEC MANAGER] DYN with blocked clause elimination\n";

  m_nbBlockedClauseRemoved = 0;
  m_isDecisionVariable.resize(p.getNbVar() + 1, !p.getNbSelectedVar());
  for (auto v : p.getSelectedVar()) m_isDecisionVariable[v] = true;

  m_isPresentLit.resize((p.getNbVar() + 1) << 1, false);
}  // SpecManagerCnfDynBlockedCl

/**
 * @brief SpecManagerCnfDynBlockedCl::getBlockedClauses implementation.
 */
void SpecManagerCnfDynBlockedCl::getBlockedClauses(
    std::vector<unsigned> &idxClauses) {
  idxClauses.clear();
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    if (m_infoClauses[i].isSat) continue;

    // mark the literals for the current clause.
    for (auto &l : m_clauses[i]) m_isPresentLit[l.intern()] = true;

    bool isBlocked = false;
    for (auto &l : m_clauses[i]) {
      if (m_currentValue[l.var()] != l_Undef) continue;
      if (m_isDecisionVariable[l.var()]) continue;

      // check all the possible resolution on l.
      m_isPresentLit[l.intern()] = false;
      isBlocked = true;
      for (IteratorIdxClause ite = m_occurrence[(~l).intern()].getClauses();
           ite.end != ite.start && isBlocked; ite.start++) {
        isBlocked = false;
        for (auto &ll : m_clauses[*(ite.start)]) {
          if (m_isPresentLit[(~ll).intern()]) {
            isBlocked = true;
            break;
          }
        }
      }

      m_isPresentLit[l.intern()] = true;
      if (isBlocked) break;
    }

    // unmark the literals for the current clause.
    for (auto &l : m_clauses[i]) m_isPresentLit[l.intern()] = false;

    // the clause is blocked.
    if (isBlocked) idxClauses.push_back(i);
  }

#if 1
  if (!idxClauses.size()) {
    for (unsigned i = 1; i < m_isDecisionVariable.size(); i++) {
      if (m_isDecisionVariable[i] || m_currentValue[i] != l_Undef) continue;
      Lit l = Lit::makeLit(i, false);

      if (!((m_occurrence[l.intern()].size() &&
             m_occurrence[(~l).intern()].size()) ||
            (!m_occurrence[l.intern()].size() &&
             !m_occurrence[(~l).intern()].size()))) {
        std::cout << l << " " << m_occurrence[l.intern()].size() << " "
                  << m_occurrence[(~l).intern()].size() << "\n";

        std::cout << l << " => ";
        for (IteratorIdxClause ite = m_occurrence[l.intern()].getClauses();
             ite.end != ite.start; ite.start++)
          std::cout << *(ite.start) << '[' << m_infoClauses[*(ite.start)].isSat
                    << "] ";
        std::cout << '\n';

        std::cout << ~l << "=> ";
        for (IteratorIdxClause ite = m_occurrence[(~l).intern()].getClauses();
             ite.end != ite.start; ite.start++)
          std::cout << *(ite.start) << '[' << m_infoClauses[*(ite.start)].isSat
                    << "] ";
        std::cout << '\n';
      }

      assert((m_occurrence[l.intern()].size() &&
              m_occurrence[(~l).intern()].size()) ||
             (!m_occurrence[l.intern()].size() &&
              !m_occurrence[(~l).intern()].size()));
    }
  }
#endif
}  // getBlockedClauses

/**
 * @brief SpecManagerCnfDynBlockedCl::inprocessing implementation.
 */
void SpecManagerCnfDynBlockedCl::inprocessing() {
  std::vector<unsigned> idxClauses;

#if 1
  // normally I have no occurrence of satisfied clauses.
  for (auto &occ : m_occurrence)
    for (IteratorIdxClause ite = occ.getClauses(); ite.end != ite.start;
         ite.start++)
      assert(!m_infoClauses[*(ite.start)].isSat);
#endif

  do {
    m_currentMarkedLitIndex++;
    getBlockedClauses(idxClauses);
    m_nbBlockedClauseRemoved += idxClauses.size();

    for (auto &idx : idxClauses) {
      m_infoClauses[idx].isSat = true;

      if (!m_markedClauseIdx[idx]) {
        m_markedClauseIdx[idx] = true;
        m_savedStateClauses.push_back(
            (SavedStateClause){(int)idx, false, m_infoClauses[idx].nbUnsat});
      }
    }

    if (idxClauses.size()) removeSatisfiedClauses(idxClauses);

#if 1
    // normally I have no occurrence of satisfied clauses.
    for (auto &occ : m_occurrence)
      for (IteratorIdxClause ite = occ.getClauses(); ite.end != ite.start;
           ite.start++)
        assert(!m_infoClauses[*(ite.start)].isSat);
#endif

  } while (idxClauses.size());
}  // inprocessing

}  // namespace d4
