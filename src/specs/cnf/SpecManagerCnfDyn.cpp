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

  m_savedStateClauses.reserve(getSumSizeClauses());
  m_savedStateOccs.reserve(getSumSizeClauses());
}  // SpecManagerCnfDyn

#define TEST_DEBUG 0

/**
   Update the occurrence list w.r.t. a new set of assigned variables.
   It's important that the order is conserved between the moment where
   we assign and the moment we unassign.

   @param[in] lits, the new assigned variables
 */
void SpecManagerCnfDyn::preUpdate(const std::vector<Lit> &lits) {
  m_stackPosClause.push_back(m_savedStateClauses.size());
  m_stackPosOcc.push_back(m_savedStateOccs.size());

  m_reviewWatcher.resize(0);
  for (auto &l : lits) {
    assert(m_currentValue[l.var()] == l_Undef);
    m_currentValue[l.var()] = l.sign();
  }

  // not binary clauses.
  for (auto &l : lits) {
    for (IteratorIdxClause ite = m_occurrence[l.intern()].getNotBinClauses();
         ite.end != ite.start; ite.start++) {
      if (m_markedClauseIdx[*(ite.start)]) continue;

      assert(!m_infoClauses[*(ite.start)].isSat);
      m_infoClauses[*(ite.start)].isSat = 1;
      m_markedClauseIdx[*(ite.start)] = true;
      m_savedStateClauses.push_back((SavedStateClause){
          *(ite.start), false, m_infoClauses[*(ite.start)].nbUnsat});
    }
  }

  for (int i = m_stackPosClause.back(); i < m_savedStateClauses.size(); i++) {
    int idxCl = m_savedStateClauses[i].idx;
    for (auto &ll : m_clauses[idxCl])
      if (m_currentValue[ll.var()] == l_Undef) {
        if (!m_markedLit[ll.intern()]) {
          m_markedLit[ll.intern()] = 1;
          m_savedStateOccs.push_back(
              (SavedStateOcc){ll, m_occurrence[ll.intern()].nbBin,
                              m_occurrence[ll.intern()].nbNotBin});
        }

        m_occurrence[ll.intern()].removeNotBin(idxCl);
      }
  }

  for (auto &l : lits) {
    for (unsigned i = 0; i < m_occurrence[(~l).intern()].nbNotBin; i++) {
      int idxCl = m_occurrence[(~l).intern()].notBin[i];
      if (!m_markedClauseIdx[idxCl]) {
        m_markedClauseIdx[idxCl] = 1;
        m_savedStateClauses.push_back((SavedStateClause){
            idxCl, m_infoClauses[idxCl].isSat, m_infoClauses[idxCl].nbUnsat});
      }
      m_infoClauses[idxCl].nbUnsat++;
      if (m_clauses[idxCl][0] == ~l) m_reviewWatcher.push_back(idxCl);
    }
  }

  // we search another non assigned literal if requiered
  for (auto &idxCl : m_reviewWatcher) {
    if (m_infoClauses[idxCl].isSat) continue;

    for (unsigned i = 1; i < m_clauses[idxCl].size(); i++) {
      if (m_currentValue[m_clauses[idxCl][i].var()] == l_Undef) {
        std::swap(m_clauses[idxCl][0], m_clauses[idxCl][i]);
        break;
      }
    }
  }

  // binary clauses.
  for (auto &l : lits) {
    for (IteratorIdxClause ite = m_occurrence[l.intern()].getBinClauses();
         ite.end != ite.start; ite.start++) {
      int idxCl = *(ite.start);
      assert(idxCl < m_markedClauseIdx.size());

      // mark the clause if needed.
      if (m_markedClauseIdx[idxCl]) continue;
      m_markedClauseIdx[idxCl] = true;
      m_savedStateClauses.push_back(
          (SavedStateClause){idxCl, false, m_infoClauses[idxCl].nbUnsat});

      // update the status.
      if (m_infoClauses[idxCl].isSat) {
        std::cout << "it is about " << l << '\n';
      }
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
        m_markedLit[otherLit.intern()] |= 2;
      }
    }

    // for the unsat lit we suppose that BCP was applied.
  }

  // apply the modification on the identified literals.
  for (unsigned i = m_stackPosOcc.back(); i < m_savedStateOccs.size(); i++) {
    unsigned lIntern = m_savedStateOccs[i].l.intern();
    if (m_markedLit[lIntern] & 2)
      m_occurrence[lIntern].removeMarkedBin(m_markedClauseIdx);
    m_markedLit[lIntern] = 0;
  }

  // unmark the clauses.
  for (int i = m_stackPosClause.back(); i < m_savedStateClauses.size(); i++)
    m_markedClauseIdx[m_savedStateClauses[i].idx] = false;
}  // preUpdate

/**
 * @brief Update the occurrence list w.r.t. a new set of unassigned variables.
 *
 * @param lits are the new assigned variables.
 */
void SpecManagerCnfDyn::postUpdate(const std::vector<Lit> &lits) {
  // manage the literal information.
  unsigned previousOcc = m_stackPosOcc.back();
  m_stackPosOcc.pop_back();
  for (int i = previousOcc; i < m_savedStateOccs.size(); i++) {
    unsigned lIntern = m_savedStateOccs[i].l.intern();
    m_occurrence[lIntern].nbNotBin = m_savedStateOccs[i].nbNotBin;
    m_occurrence[lIntern].bin -=
        m_savedStateOccs[i].nbBin - m_occurrence[lIntern].nbBin;
    m_occurrence[lIntern].nbBin = m_savedStateOccs[i].nbBin;
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
