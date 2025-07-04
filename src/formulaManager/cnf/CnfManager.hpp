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
#pragma once

#include <iterator>

#include "../FormulaManager.hpp"
#include "ClauseInfo.hpp"
#include "DataOccurrence.hpp"
#include "src/options/cache/OptionBucketManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {
struct InfoCluster {
  Var parent;
  unsigned size;
  int pos;
};

class CnfManager : public FormulaManager {
 protected:
  std::vector<std::vector<Lit>> m_clauses;
  std::vector<int> m_clausesNotBin;
  unsigned m_maxSizeClause;
  std::vector<ClauseInfo> m_infoClauses;

  std::vector<bool> m_inCurrentComponent;
  std::vector<DataOccurrence> m_occurrence;
  int *m_dataOccurrenceMemory;
  std::vector<unsigned> m_occInitSizeNotBin;

  std::vector<InfoCluster> m_infoCluster;

  // to manage the connected component
  std::vector<int> m_mustUnMark;
  std::vector<Var> m_tmpVecVar;
  std::vector<int> m_idxComponent;
  std::vector<bool> m_markView;

  inline void resetUnMark() {
    for (auto &idx : m_mustUnMark) m_markView[idx] = false;
    m_mustUnMark.resize(0);
  }  // resetUnMark

  Var *m_activeVariables;

  /**
   * @brief This function can be used in order to check the validity of the
   * occurence list.
   */
  void debugFunction();

 public:
  CnfManager(ProblemManager &p);
  ~CnfManager();

  int computeConnectedComponent(std::vector<std::vector<Var>> &varConnected,
                                std::vector<Var> &setOfVar,
                                std::vector<Var> &freeVar) override;

  void connectedToLit(Lit l, std::vector<int> &v,
                      std::vector<Var> &varComponent, int nbComponent);

  int computeConnectedComponentTargeted(
      std::vector<std::vector<Var>> &varConnected, std::vector<Var> &setOfVar,
      std::vector<bool> &isProjected, std::vector<Var> &freeVar) override;

  void showFormula(std::ostream &out) override;
  void showCurrentFormula(std::ostream &out) override;
  void showCurrentFormula(std::ostream &out,
                          std::vector<bool> &isInComponent) override;

  int getInitSize(int i) { return m_clauses[i].size(); }
  int getCurrentSize(int i) {
    return m_clauses[i].size() - m_infoClauses[i].nbUnsat;
  }

  bool isSatisfiedClause(unsigned idx);
  bool isSatisfiedClause(std::vector<Lit> &c);
  bool isNotSatisfiedClauseAndInComponent(
      int idx, std::vector<bool> &m_inCurrentComponent);

  virtual void getCurrentClauses(std::vector<unsigned> &idxClauses,
                                 std::vector<Var> &component) = 0;

  virtual void getCurrentClausesNotBin(std::vector<unsigned> &idxClauses,
                                       std::vector<Var> &component) = 0;

  virtual bool canSkipLit(const Lit &l) { return false; }

  // inline functions.
  // about the CNF.
  inline int getNbBinaryClause(Var v) {
    return getNbBinaryClause(Lit::makeLitFalse(v)) +
           getNbBinaryClause(Lit::makeLitTrue(v));
  }
  inline int getNbNotBinaryClause(Lit l) {
    return getNbClause(l) - getNbBinaryClause(l);
  }
  inline int getNbNotBinaryClause(Var v) {
    return getNbClause(v) - getNbBinaryClause(v);
  }
  inline int getNbClause(Var v) {
    return getNbClause(Lit::makeLitFalse(v)) + getNbClause(Lit::makeLitTrue(v));
  }

  inline unsigned getNbClause(Lit l) {
    return m_occurrence[l.intern()].nbBin + m_occurrence[l.intern()].nbNotBin;
  }

  inline bool isFreeVariable(Var v) {
    return !getNbOccurrence(Lit::makeLitFalse(v)) &&
           !getNbOccurrence(Lit::makeLitTrue(v));
  }  // isFreeVariable

  inline unsigned getNbInitNotBinaryClause(Lit l) {
    return m_occInitSizeNotBin[l.intern()];
  }

  inline unsigned getNbRemainingInitNotBinaryClause(Lit l) {
    return m_occurrence[l.intern()].nbNotBin;
  }  // getNbInitNotBinaryClause

  inline unsigned getNbClause() { return m_clauses.size(); }
  inline unsigned getMaxSizeClause() { return m_maxSizeClause; }

  virtual inline unsigned getSumSizeClauses() {
    unsigned sum = 0;
    for (auto &cl : m_clauses) sum += cl.size();
    return sum;
  }  // getSumSizeClauses

  inline int getNbBinaryClause(Lit l) {
    int nbBin = m_occurrence[l.intern()].nbBin;
    for (unsigned i = 0; i < m_occurrence[l.intern()].nbNotBin; i++)
      if (getSize(m_occurrence[l.intern()].notBin[i]) == 2) nbBin++;
    return nbBin;
  }  // getNbBinaryClause

  // about the clauses.
  inline int getNbUnsat(int idx) { return m_infoClauses[idx].nbUnsat; }
  inline int getSize(int idx) {
    return m_clauses[idx].size() - m_infoClauses[idx].nbUnsat;
  }

  inline std::vector<std::vector<Lit>> &getClauses() { return m_clauses; }

  inline std::vector<Lit> &getClause(int idx) {
    assert((unsigned)idx < m_clauses.size());
    return m_clauses[idx];
  }

  inline int getNbOccurrence(Lit l) { return getNbClause(l); }

  inline IteratorIdxClause getVecIdxClauseBin(Lit l) {
    assert(l.intern() < m_occurrence.size());
    return m_occurrence[l.intern()].getBinClauses();
  }

  inline IteratorIdxClause getVecIdxClauseNotBin(Lit l) {
    assert(l.intern() < m_occurrence.size());
    return m_occurrence[l.intern()].getNotBinClauses();
  }

  inline IteratorIdxClause getVecIdxClause(Lit l) {
    assert(l.intern() < m_occurrence.size());
    return m_occurrence[l.intern()].getClauses();
  }

  inline IteratorIdxClause getVecIdxClause(Lit l, ModeStore mode) {
    assert(l.intern() < m_occurrence.size());
    if (mode == CACHE_NT) return m_occurrence[l.intern()].getNotBinClauses();
    if (mode == CACHE_ALL) return m_occurrence[l.intern()].getClauses();
    return m_occurrence[l.intern()].getBinClauses();
  }

  inline void showOccurenceList(std::ostream &out) {
    printf("Occurence list\n");
    for (unsigned i = 0; i < m_occurrence.size(); i++) {
      if (m_currentValue[i >> 1] != l_Undef) continue;
      if (!m_occurrence[i].nbBin && !m_occurrence[i].nbNotBin) continue;
      out << ((i & 1) ? "-" : "") << (i >> 1) << " --> [ ";
      for (unsigned j = 0; j < m_occurrence[i].nbBin; j++)
        out << m_occurrence[i].bin[j] << " ";
      for (unsigned j = 0; j < m_occurrence[i].nbNotBin; j++)
        out << m_occurrence[i].notBin[j] << " ";
      out << " ]\n";
    }
  }

  inline ProblemInputType getProblemInputType() override { return PB_CNF; }
};
}  // namespace d4
