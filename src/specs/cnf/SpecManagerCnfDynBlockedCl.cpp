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

  // init the watch list.
  m_idxBlockedClauses.resize(m_clauses.size());
  m_watchedList.resize(m_clauses.size());
  m_indexSatClauses.resize(0);

  // init the structure of watch.
  for (unsigned i = 0; i < m_clauses.size(); i++) {
    std::vector<Watched> list;

    // mark the literals for the current clause.
    for (auto &l : m_clauses[i]) m_isPresentLit[l.intern()] = true;

    // visit the clause and extract the watched list.
    bool isBlocked = false;
    for (auto &l : m_clauses[i]) {
      if (m_currentValue[l.var()] != l_Undef) continue;
      if (m_isDecisionVariable[l.var()]) continue;

      // get a clause that is not in tautology
      unsigned idxCl = searchTautNotResolution(m_isPresentLit, l);
      if (idxCl == m_clauses.size()) {
        isBlocked = true;
        break;
      } else
        list.push_back({l, idxCl});
    }

    // unmark the literals for the current clause.
    for (auto &l : m_clauses[i]) m_isPresentLit[l.intern()] = false;

    // add or not the clause in the watch list.
    if (isBlocked)
      m_indexSatClauses.push_back(i);
    else
      for (auto &w : list) m_watchedList[w.idxCl].push_back({w.l, i});
  }

  // count the number of dectected.
  m_nbBlockedClauseRemoved += m_indexSatClauses.size();

  // remove the satisfied clauses.
  m_currentMarkedLitIndex = 1;
  for (auto &idx : m_indexSatClauses) m_infoClauses[idx].isSat = true;
  m_stackPosClause.push_back(m_savedStateClauses.size());
  m_stackPosOcc.push_back(m_savedStateOccs.size());
  removeSatisfiedClauses(m_indexSatClauses);

  // call the simplification to progate.
  inprocessing();

  // remove all the satisfied clause (because at 'level 0').
  for (auto &wlist : m_watchedList) {
    unsigned j = 0;
    for (unsigned i = 0; i < wlist.size(); i++)
      if (!m_infoClauses[wlist[i].idxCl].isSat) wlist[j++] = wlist[i];
    wlist.resize(j);
  }

  // unmark the literals.
  for (unsigned i = m_stackPosOcc.back(); i < m_savedStateOccs.size(); i++)
    m_markedLit[m_savedStateOccs[i].l.intern()] = 0;
}  // SpecManagerCnfDynBlockedCl

/**
 * @brief SpecManagerCnfDynBlockedCl::searchTautNotResolution implementation.
 */
unsigned SpecManagerCnfDynBlockedCl::searchTautNotResolution(
    std::vector<bool> &isPresentLit, Lit l) {
  // do not consider l in the resolution.
  m_isPresentLit[l.intern()] = false;

  // check all the possible resolution on l.
  bool isBlocked = true;
  for (IteratorIdxClause ite = m_occurrence[(~l).intern()].getClauses();
       ite.end != ite.start && isBlocked; ite.start++) {
    if (m_infoClauses[*(ite.start)].isSat) continue;

    isBlocked = false;
    for (auto &ll : m_clauses[*(ite.start)]) {
      if (m_isPresentLit[(~ll).intern()]) {
        isBlocked = true;
        break;
      }
    }

    if (!isBlocked) {
      m_isPresentLit[l.intern()] = true;
      return *(ite.start);
    }
  }

  // reset the presence of l.
  m_isPresentLit[l.intern()] = true;
  return m_clauses.size();
}  // searchTautNotResolution

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

    // search for a non tautological clause.
    bool isBlocked = false;
    for (auto &l : m_clauses[i]) {
      if (m_currentValue[l.var()] != l_Undef) continue;
      if (m_isDecisionVariable[l.var()]) continue;

      isBlocked =
          searchTautNotResolution(m_isPresentLit, l) == m_clauses.size();
      if (isBlocked) break;
    }

    // unmark the literals for the current clause.
    for (auto &l : m_clauses[i]) m_isPresentLit[l.intern()] = false;

    // if the clause is blocked we add it.
    if (isBlocked) idxClauses.push_back(i);
  }
}  // getBlockedClauses

/**
 * @brief SpecManagerCnfDynBlockedCl::inprocessing implementation.
 * m_indexSatClauses contains the clause we have to propagate at the current
 * level.
 */
void SpecManagerCnfDynBlockedCl::inprocessing() {
  // get the blocked clauses.
  m_idxBlockedClauses.resize(0);
  while (m_indexSatClauses.size()) {
    unsigned idx = m_indexSatClauses.back();
    m_indexSatClauses.pop_back();

    unsigned j = 0;
    std::vector<Watched> &wlist = m_watchedList[idx];
    for (unsigned i = 0; i < wlist.size(); i++) {
      if (m_infoClauses[wlist[i].idxCl].isSat ||
          m_currentValue[wlist[i].l.var()] != l_Undef)
        wlist[j++] = wlist[i];
      else {
        // search for another watch.
        for (auto &l : m_clauses[wlist[i].idxCl])
          m_isPresentLit[l.intern()] = true;
        unsigned next = searchTautNotResolution(m_isPresentLit, wlist[i].l);
        for (auto &l : m_clauses[wlist[i].idxCl])
          m_isPresentLit[l.intern()] = false;

        // update the watch structure according to next.
        if (next != m_clauses.size())  // the clause is not blocked.
          m_watchedList[next].push_back(wlist[i]);
        else {
          // the clause is blocked.
          m_idxBlockedClauses.push_back(wlist[i].idxCl);
          m_indexSatClauses.push_back(wlist[i].idxCl);
          m_infoClauses[wlist[i].idxCl].isSat = true;

          if (!m_markedClauseIdx[wlist[i].idxCl]) {
            m_markedClauseIdx[wlist[i].idxCl] = true;
            m_savedStateClauses.push_back(
                (SavedStateClause){(int)wlist[i].idxCl, false,
                                   m_infoClauses[wlist[i].idxCl].nbUnsat});
          }
          wlist[j++] = wlist[i];
        }
      }
    }
    wlist.resize(j);
  }

  // remove.
  m_currentMarkedLitIndex++;
  m_nbBlockedClauseRemoved += m_idxBlockedClauses.size();

  removeSatisfiedClauses(m_idxBlockedClauses);
}  // inprocessing

}  // namespace d4
