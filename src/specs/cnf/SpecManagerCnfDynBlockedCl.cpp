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

#include "SpecManagerCnf.hpp"

namespace d4 {

/**
 * @brief SpecManagerCnfDynBlockedCl::SpecManagerCnfDynBlockedCl implementation.
 */
SpecManagerCnfDynBlockedCl::SpecManagerCnfDynBlockedCl(ProblemManager &p)
    : SpecManagerCnf(p) {
  std::cout << "c [SPEC MANAGER] DYN with blocked clause elimination\n";
  m_currentMarkedLitIndex = 0;
  m_markedLit.resize((1 + p.getNbVar()) << 1, 0);
  m_markedClauseIdx.resize(m_clauses.size() + 1, false);
  m_indexSatClauses.reserve(m_clauses.size());
  m_nbPureSimplification = 0;

  m_savedStateClauses.reserve(getSumSizeClauses());
  m_savedStateOccs.reserve(getSumSizeClauses());

  m_markedPureLiteral.resize(1 + p.getNbVar(), false);
  m_pureDetected.reserve(1 + p.getNbVar());

  m_isDecisionVariable.resize(p.getNbVar() + 1, !p.getNbSelectedVar());
  for (auto v : p.getSelectedVar()) m_isDecisionVariable[v] = true;

  affectInitPureLit();
}  // SpecManagerCnfDynBlockedCl

/**
 * @brief SpecManagerCnfDynBlockedCl::affectInitPureLit implementation.
 */
void SpecManagerCnfDynBlockedCl::affectInitPureLit() {
  getPureLiterals(m_pureDetected);
  m_stackPosClause.push_back(m_savedStateClauses.size());
  m_stackPosOcc.push_back(m_savedStateOccs.size());
  m_stackPosPure.push_back(m_savedPureLits.size());
  m_currentMarkedLitIndex = 0;
  propagateTrueInNotBin(m_pureDetected);
  propagateTrueInBin(m_pureDetected);

  m_nbPureSimplification += m_pureDetected.size();
}  // affectInitPureLit

/**
 * @brief SpecManagerCnfDynBlockedCl::getPureLiterals implementation.
 */
void SpecManagerCnfDynBlockedCl::getPureLiterals(std::vector<Lit> &pureLits) {
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
 * @brief SpecManagerCnfDynBlockedCl::removeSatisfiedClauses implementation.
 */
void SpecManagerCnfDynBlockedCl::removeSatisfiedClauses(
    const std::vector<unsigned> &idxClauses) {
  m_currentMarkedLitIndex++;
  for (auto idxCl : idxClauses) {
    for (auto &ll : m_clauses[idxCl])
      if (m_currentValue[ll.var()] == l_Undef) {
        if (m_markedLit[ll.intern()] != m_currentMarkedLitIndex) {
          if (!m_markedLit[ll.intern()])
            m_savedStateOccs.push_back(
                (SavedStateOcc){ll, m_occurrence[ll.intern()].nbBin,
                                m_occurrence[ll.intern()].nbNotBin});

          m_markedLit[ll.intern()] = m_currentMarkedLitIndex;
          m_occurrence[ll.intern()].removeNotBinMarked(m_infoClauses);
        }
      }
  }
}  // removeSatisfiedClauses

/**
 * @brief SpecManagerCnfDynBlockedCl::propagateTrueInNotBin implementation.
 */
void SpecManagerCnfDynBlockedCl::propagateTrueInNotBin(
    const std::vector<Lit> &lits) {
  m_indexSatClauses.resize(0);

  for (auto &l : lits) {
    for (IteratorIdxClause ite = m_occurrence[l.intern()].getNotBinClauses();
         ite.end != ite.start; ite.start++) {
      if (m_infoClauses[*(ite.start)].isSat) continue;
      m_infoClauses[*(ite.start)].isSat = 1;
      m_indexSatClauses.push_back(*(ite.start));

      if (m_markedClauseIdx[*(ite.start)]) continue;
      m_markedClauseIdx[*(ite.start)] = true;
      m_savedStateClauses.push_back((SavedStateClause){
          *(ite.start), false, m_infoClauses[*(ite.start)].nbUnsat});
    }
  }

  removeSatisfiedClauses(m_indexSatClauses);
}  // propagateTrueInNotBin

/**
 * @brief SpecManagerCnfDynBlockedCl::propagateTrueInBin implementation.
 */
void SpecManagerCnfDynBlockedCl::propagateTrueInBin(
    const std::vector<Lit> &lits) {
  m_currentMarkedLitIndex++;
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
        m_markedLit[otherLit.intern()] = m_currentMarkedLitIndex;
      }
    }
  }

  // apply the modification on the identified literals.
  for (unsigned i = m_stackPosOcc.back(); i < m_savedStateOccs.size(); i++) {
    unsigned lIntern = m_savedStateOccs[i].l.intern();
    if (m_markedLit[lIntern] == m_currentMarkedLitIndex) {
      m_occurrence[lIntern].removeMarkedBin(m_infoClauses);
    }
  }
}  // propagateTrueInBin

/**
 * @brief SpecManagerCnfDynBlockedCl::propagateFalseInNotBin implementation.
 */
void SpecManagerCnfDynBlockedCl::propagateFalseInNotBin(
    const std::vector<Lit> &lits) {
  m_currentMarkedLitIndex++;
  for (auto &l : lits) {
    for (unsigned i = 0; i < m_occurrence[(~l).intern()].nbNotBin; i++) {
      int idxCl = m_occurrence[(~l).intern()].notBin[i];
      if (!m_markedClauseIdx[idxCl]) {
        m_markedClauseIdx[idxCl] = m_currentMarkedLitIndex;
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
 * @brief SpecManagerCnfDynBlockedCl::searchPureLitOnTheStack implementation.
 */
bool SpecManagerCnfDynBlockedCl::searchPureLitOnTheStack() {
  // consider the pure literals.
  bool ret = false;
  do {
    m_pureDetected.resize(0);
    for (unsigned i = m_stackPosOcc.back(); i < m_savedStateOccs.size(); i++) {
      Lit &l = m_savedStateOccs[i].l;
      if (m_isDecisionVariable[l.var()] || m_currentValue[l.var()] != l_Undef)
        continue;

      if (m_occurrence[l.intern()].size() == 0 &&
          m_occurrence[(~l).intern()].size() != 0) {
        m_pureDetected.push_back(~l);

        assert(m_currentValue[l.var()] == l_Undef);
        m_savedPureLits.push_back(~l);
        m_currentValue[l.var()] = (~l).sign();
      }
    }
    propagateTrueInNotBin(m_pureDetected);
    propagateTrueInBin(m_pureDetected);
    m_nbPureSimplification += m_pureDetected.size();

    if (m_pureDetected.size()) ret = true;
  } while (m_pureDetected.size());

  return ret;
}  // searchPureLitOnTheStack

/**
 * @brief SpecManagerCnfDynBlockedCl::preUpdate implementation.
 */
void SpecManagerCnfDynBlockedCl::preUpdate(const std::vector<Lit> &lits) {
  m_stackPosClause.push_back(m_savedStateClauses.size());
  m_stackPosOcc.push_back(m_savedStateOccs.size());
  m_stackPosPure.push_back(m_savedPureLits.size());
  m_currentMarkedLitIndex = 0;

  m_reviewWatcher.resize(0);
  for (auto &l : lits) {
    assert(m_currentValue[l.var()] == l_Undef);
    m_currentValue[l.var()] = l.sign();
  }

  // manage the non binary clauses.
  propagateTrueInNotBin(lits);
  propagateFalseInNotBin(lits);

  // binary clauses.
  propagateTrueInBin(lits);
  // for the unsat lit in binary clauses,  we suppose that BCP has been applied.

  // search for pure literals.
  searchPureLitOnTheStack();

  // unmark the literals.
  for (unsigned i = m_stackPosOcc.back(); i < m_savedStateOccs.size(); i++)
    m_markedLit[m_savedStateOccs[i].l.intern()] = 0;

  // unmark the clauses.
  for (int i = m_stackPosClause.back(); i < m_savedStateClauses.size(); i++)
    m_markedClauseIdx[m_savedStateClauses[i].idx] = false;
}  // preUpdate

/**
 * @brief Update the occurrence list w.r.t. a new set of unassigned variables.
 *
 * @param lits are the new assigned variables.
 */
void SpecManagerCnfDynBlockedCl::postUpdate(const std::vector<Lit> &lits) {
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

  // reset the pure literals.
  for (unsigned i = m_stackPosPure.back(); i < m_savedPureLits.size(); i++)
    m_currentValue[m_savedPureLits[i].var()] = l_Undef;
  m_savedPureLits.resize(m_stackPosPure.back());
  m_stackPosPure.pop_back();

  // reset the unit literals.
  for (auto &l : lits) m_currentValue[l.var()] = l_Undef;
}  // postUpdate

}  // namespace d4
