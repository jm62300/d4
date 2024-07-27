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

#include "CnfManagerDyn.hpp"

namespace d4 {

/**
 * @brief CnfManagerDyn::CnfManagerDyn implementation.
 */
CnfManagerDyn::CnfManagerDyn(ProblemManager &p,
                             bool keepListNotSatisfiedClauses)
    : CnfManager(p),
      m_keepListNotSatisfiedClauses(keepListNotSatisfiedClauses) {
  m_markedLitStack.resize((1 + p.getNbVar()) << 1, 0);
  m_markedLitRemove.resize((1 + p.getNbVar()) << 1, 0);
  m_markedClauseIdx.resize(m_clauses.size() + 1, false);
  m_indexSatClauses.reserve(m_clauses.size());

  m_currentMarkedLitStackIndex = m_currentMarkedLitRemoveIndex = 0;

  m_savedStateClauses.reserve(getSumSizeClauses());
  m_savedStateOccs.reserve(getSumSizeClauses());

  m_notSatifiedClauses = nullptr;
  if (m_keepListNotSatisfiedClauses) {
    m_notSatifiedClauses = new unsigned[m_clauses.size()];
    for (unsigned i = 0; i < m_clauses.size(); i++) m_notSatifiedClauses[i] = i;
    m_sizeNotSatifiedClauses = m_clauses.size();
    m_markedNotSatClauses.resize(m_clauses.size() + 1, 0);
    m_stampNotSatClauses = 0;
  }
}  // CnfManagerDyn

/**
 * @brief CnfManagerDyn::~CnfManagerDyn implementation.
 */
CnfManagerDyn::~CnfManagerDyn() {
  if (m_keepListNotSatisfiedClauses) delete[] m_notSatifiedClauses;
}  // destructor

/**
 * @brief CnfManagerDyn::getCurrentClauses implementation.
 */
void CnfManagerDyn::getCurrentClauses(std::vector<unsigned> &idxClauses,
                                      std::vector<Var> &component) {
  idxClauses.resize(0);
  for (auto &v : component) m_inCurrentComponent[v] = true;
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    if (isNotSatisfiedClauseAndInComponent(i, m_inCurrentComponent))
      idxClauses.push_back(i);
  }
  for (auto &v : component) m_inCurrentComponent[v] = false;
}  // getCurrentclauses

/**
 * @brief CnfManagerDyn::getCurrentClausesNotBin implementation.
 */
void CnfManagerDyn::getCurrentClausesNotBin(std::vector<unsigned> &idxClauses,
                                            std::vector<Var> &component) {
  assert(m_keepListNotSatisfiedClauses);
  idxClauses.resize(0);
  for (auto &v : component) m_inCurrentComponent[v] = true;
  for (unsigned i = 0; i < m_sizeNotSatifiedClauses; i++) {
    unsigned idx = m_notSatifiedClauses[i];
    if (m_clauses[idx].size() > 2 &&
        m_inCurrentComponent[m_clauses[idx][0].var()])
      idxClauses.push_back(m_notSatifiedClauses[i]);
  }

  std::sort(idxClauses.begin(), idxClauses.end());
  for (auto &v : component) m_inCurrentComponent[v] = false;
}  // getCurrentclauses

/**
 * @brief CnfManagerDynPure::propagateFalseInNotBin implementation.
 */
void CnfManagerDyn::propagateFalseInNotBin(const std::vector<Lit> &lits) {
  m_reviewWatcher.resize(0);
  for (auto &l : lits) {
    for (unsigned i = 0; i < m_occurrence[(~l).intern()].nbNotBin; i++) {
      int idxCl = m_occurrence[(~l).intern()].notBin[i];
      if (!m_markedClauseIdx[idxCl]) {
        m_markedClauseIdx[idxCl] = true;
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
}  // propagateFalseInNotBin

/**
 * @brief CnfManagerDyn::removeSatisfiedClauses implementation.
 */
void CnfManagerDyn::removeSatisfiedClauses(
    const std::vector<unsigned> &idxClauses) {
  m_currentMarkedLitRemoveIndex++;

  for (auto idxCl : idxClauses) {
    for (auto &ll : m_clauses[idxCl]) {
      if (m_markedLitStack[ll.intern()] != m_currentMarkedLitStackIndex) {
        m_savedStateOccs.push_back(
            (SavedStateOcc){ll, m_occurrence[ll.intern()].nbBin,
                            m_occurrence[ll.intern()].nbNotBin});
        m_markedLitStack[ll.intern()] = m_currentMarkedLitStackIndex;
      }

      if (m_markedLitRemove[ll.intern()] != m_currentMarkedLitRemoveIndex) {
        m_markedLitRemove[ll.intern()] = m_currentMarkedLitRemoveIndex;
        m_occurrence[ll.intern()].removeSatisfiedNotBin(m_infoClauses);
        m_occurrence[ll.intern()].removeSatisfiedBin(m_infoClauses);
      }
    }
  }

  if (m_keepListNotSatisfiedClauses) {
    m_stackSizeListNotSatisfiedClauses.push_back(m_sizeNotSatifiedClauses);
    m_stampNotSatClauses++;
    for (auto idxCl : idxClauses)
      m_markedNotSatClauses[idxCl] = m_stampNotSatClauses;

    for (unsigned i = 0; i < m_sizeNotSatifiedClauses;) {
      assert(m_notSatifiedClauses[i] < m_markedNotSatClauses.size());
      if (m_markedNotSatClauses[m_notSatifiedClauses[i]] ==
          m_stampNotSatClauses) {
        m_sizeNotSatifiedClauses--;
        std::swap(m_notSatifiedClauses[i],
                  m_notSatifiedClauses[m_sizeNotSatifiedClauses]);
      } else
        i++;
    }
  }
}  // removeSatisfiedClauses

/**
 * @brief CnfManagerDyn::propagateTrue implementation.
 */
void CnfManagerDyn::propagateTrue(const std::vector<Lit> &lits) {
  m_indexSatClauses.resize(0);

  for (auto &l : lits) {
    // mark all the clauses containing l as SAT.
    for (IteratorIdxClause ite = m_occurrence[l.intern()].getClauses();
         ite.end != ite.start; ite.start++) {
      if (m_infoClauses[*(ite.start)].isSat) continue;
      m_infoClauses[*(ite.start)].isSat = 1;
      m_indexSatClauses.push_back(*(ite.start));

      if (m_markedClauseIdx[*(ite.start)]) continue;
      m_markedClauseIdx[*(ite.start)] = true;
      m_savedStateClauses.push_back((SavedStateClause){
          *(ite.start), false, m_infoClauses[*(ite.start)].nbUnsat});
    }

    // remove the occurrence list.
    if (m_markedLitStack[l.intern()] != m_currentMarkedLitStackIndex) {
      m_savedStateOccs.push_back(
          (SavedStateOcc){l, m_occurrence[l.intern()].nbBin,
                          m_occurrence[l.intern()].nbNotBin});
      m_markedLitStack[l.intern()] = m_currentMarkedLitStackIndex;
    }
    m_occurrence[l.intern()].clean();
  }

  removeSatisfiedClauses(m_indexSatClauses);
}  // propagateTrue

/**
 * @brief CnfManagerDyn::preUpdate implementation.
 */
void CnfManagerDyn::preUpdate(const std::vector<Lit> &lits) {
  pushStacks();
  assignListLit(lits);

  // manage the non binary clauses.
  propagateTrue(lits);
  propagateFalseInNotBin(lits);

  // search for pure literals.
  inprocessing();

  // unmark the clauses.
  unmarkLastClausesSaved();
}  // preUpdate

/**
 * @brief CnfManagerDyn::postUpdate implementation.
 */
void CnfManagerDyn::postUpdate(const std::vector<Lit> &lits) {
  // manage the literal information.
  unsigned previousOcc = m_stackPosOcc.back();
  m_stackPosOcc.pop_back();
  for (int i = previousOcc; i < m_savedStateOccs.size(); i++) {
    unsigned lIntern = m_savedStateOccs[i].l.intern();
    assert(m_savedStateOccs[i].nbBin >= m_occurrence[lIntern].nbBin);
    assert(m_savedStateOccs[i].nbNotBin >= m_occurrence[lIntern].nbNotBin);

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

  // reset the unit literals.
  for (auto &l : lits) m_currentValue[l.var()] = l_Undef;

  if (m_keepListNotSatisfiedClauses) {
    assert(m_stackSizeListNotSatisfiedClauses.size());
    m_sizeNotSatifiedClauses = m_stackSizeListNotSatisfiedClauses.back();
    m_stackSizeListNotSatisfiedClauses.pop_back();
  }
}  // postUpdate

}  // namespace d4
