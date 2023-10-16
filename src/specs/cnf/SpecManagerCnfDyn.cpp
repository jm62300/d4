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

#include "SpecManagerCnfDyn.hpp"

#include "SpecManagerCnf.hpp"

namespace d4 {

/**
 * @brief SpecManagerCnfDyn::SpecManagerCnfDyn implementation.
 */
SpecManagerCnfDyn::SpecManagerCnfDyn(ProblemManager &p) : SpecManagerCnf(p) {
  m_markedLit.resize((1 + p.getNbVar()) << 1, false);
  m_markedClauseIdx.resize(m_clauses.size() + 1, false);
}  // SpecManagerCnfDyn

/**
   Update the occurrence list w.r.t. a new set of assigned variables.
   It's important that the order is conserved between the moment where
   we assign and the moment we unassign.

   @param[in] lits, the new assigned variables
 */
void SpecManagerCnfDyn::preUpdate(std::vector<Lit> &lits) {
#if 0
  std::cout << "PreUpdate: ";
  for (auto &l : lits) std::cout << l.human() << ' ';
  std::cout << "\n";
  showOccurenceList(std::cout);
#endif

  m_stackPosClause.push_back(m_savedStateClauses.size());
  m_stackPosOcc.push_back(m_savedStateOccs.size());

  m_reviewWatcher.resize(0);
  for (auto &l : lits) {
    assert(m_currentValue[l.var()] == l_Undef);
    m_currentValue[l.var()] = l.sign() ? l_False : l_True;
  }

  // not binary clauses.
  for (auto &l : lits) {
    for (unsigned i = 0; i < m_occurrence[l.intern()].nbNotBin; i++) {
      int idxCl = m_occurrence[l.intern()].notBin[i];
      if (m_markedClauseIdx[idxCl]) continue;

      m_infoClauses[idxCl].isSat = 1;
      m_markedClauseIdx[idxCl] = true;
      m_savedStateClauses.push_back(
          (SavedStateClause){idxCl, false, m_infoClauses[idxCl].nbUnsat});
    }

    for (unsigned i = 0; i < m_occurrence[(~l).intern()].nbNotBin; i++) {
      int idxCl = m_occurrence[(~l).intern()].notBin[i];
      m_infoClauses[idxCl].nbUnsat++;
      if (m_infoClauses[idxCl].watcher == ~l) m_reviewWatcher.push_back(idxCl);
    }
  }

  for (int i = m_stackPosClause.back(); i < m_savedStateClauses.size(); i++) {
    int idxCl = m_savedStateClauses[i].idx;
    for (auto &ll : m_clauses[idxCl])
      if (m_currentValue[ll.var()] == l_Undef) {
        if (!m_markedLit[ll.intern()]) {
          m_markedLit[ll.intern()] = true;
          m_savedStateOccs.push_back(
              (SavedStateOcc){ll, m_occurrence[ll.intern()].nbBin,
                              m_occurrence[ll.intern()].nbNotBin});
        }

        m_occurrence[ll.intern()].removeNotBin(idxCl);
      }
  }

  // we search another non assigned literal if requiered
  for (auto &idxCl : m_reviewWatcher) {
    if (m_infoClauses[idxCl].isSat) continue;

    for (auto &l : m_clauses[idxCl]) {
      if (m_currentValue[l.var()] == l_Undef) {
        m_infoClauses[idxCl].watcher = l;
        break;
      }
    }
  }

  // binary clauses.
  unsigned savePosOcc = m_savedStateOccs.size();
  for (auto &l : lits) {
    for (unsigned i = 0; i < m_occurrence[l.intern()].nbBin; i++) {
      int idxCl = m_occurrence[l.intern()].bin[i];
      assert(idxCl < m_markedClauseIdx.size());

      // mark the clause if needed.
      if (m_markedClauseIdx[idxCl]) continue;
      m_markedClauseIdx[idxCl] = true;
      m_savedStateClauses.push_back(
          (SavedStateClause){idxCl, false, m_infoClauses[idxCl].nbUnsat});

#if 0
      const Lit tl = {(int)(m_infoClauses[idxCl].xorLitBin ^ l.intern())};
      std::cout << tl.human() << " " << l.human() << " " << idxCl << "\n";
      bool isIn = false;
      for (auto &l : lits) isIn = isIn || (l.var() == tl.var());
      if (m_currentValue[tl.var()] != l_Undef) {
        std::cout << tl.human() << " " << l.human() << " " << idxCl
                  << " error 1\n";
        if (!isIn) exit(1);
      } else {
        std::cout << tl.human() << " " << l.human() << " " << idxCl
                  << " error 2 \n";
        if (isIn) exit(1);
      }
#endif

      // update the status.
      assert(!m_infoClauses[idxCl].isSat);
      m_infoClauses[idxCl].isSat = 1;

      // check if we have to consider the other literal.
      const Lit otherLit = {(int)(m_infoClauses[idxCl].xorLitBin ^ l.intern())};
      if (m_currentValue[otherLit.var()] == l_Undef) {
        // mark the literal.
        if (!m_markedLit[otherLit.intern()]) {
          m_markedLit[otherLit.intern()] = true;
          m_savedStateOccs.push_back(
              (SavedStateOcc){otherLit, m_occurrence[otherLit.intern()].nbBin,
                              m_occurrence[otherLit.intern()].nbNotBin});
        }
      }
    }

    // for the unsat lit we suppose that BCP was applied.
  }

  // apply the modification on the identified literals.
  for (unsigned i = m_stackPosOcc.back(); i < m_savedStateOccs.size(); i++)
    m_occurrence[m_savedStateOccs[i].l.intern()].removeMarkedBin(
        m_markedClauseIdx);

  // unmark the clauses and lits.
  for (unsigned i = m_stackPosOcc.back(); i < m_savedStateOccs.size(); i++)
    m_markedLit[m_savedStateOccs[i].l.intern()] = false;
  for (int i = m_stackPosClause.back(); i < m_savedStateClauses.size(); i++)
    m_markedClauseIdx[m_savedStateClauses[i].idx] = false;
}  // preUpdate

/**
 * @brief Update the occurrence list w.r.t. a new set of unassigned variables.
 *
 * @param lits are the new assigned variables.
 */
void SpecManagerCnfDyn::postUpdate(std::vector<Lit> &lits) {
#if 0
  std::cout << "PostUpdate: ";
  for (auto &l : lits) std::cout << l.human() << ' ';
  std::cout << "\n";
#endif

  // manage the literal information.
  unsigned previousOcc = m_stackPosOcc.back();
  m_stackPosOcc.pop_back();
  for (int i = previousOcc; i < m_savedStateOccs.size(); i++) {
    Lit l = m_savedStateOccs[i].l;
    m_occurrence[l.intern()].nbNotBin = m_savedStateOccs[i].nbNotBin;
    m_occurrence[l.intern()].bin -=
        m_savedStateOccs[i].nbBin - m_occurrence[l.intern()].nbBin;
    m_occurrence[l.intern()].nbBin = m_savedStateOccs[i].nbBin;
  }
  m_savedStateOccs.resize(previousOcc);

  // manage the clause information.
  unsigned previousClause = m_stackPosClause.back();
  m_stackPosClause.pop_back();
  for (int i = previousClause; i < m_savedStateClauses.size(); i++) {
    int idxCl = m_savedStateClauses[i].idx;
    m_infoClauses[idxCl].isSat = m_savedStateClauses[i].isSat;
    m_infoClauses[idxCl].nbUnsat = m_savedStateClauses[i].nbUnsat;
  }
  m_savedStateClauses.resize(previousClause);

  for (auto &l : lits) m_currentValue[l.var()] = l_Undef;
}  // postUpdate

}  // namespace d4
