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
#include <cassert>
#include <src/problem/ProblemManager.hpp>
#include <src/problem/ProblemTypes.hpp>
#include <vector>

#include "CnfManager.hpp"

namespace d4 {

struct SavedStateOcc {
  Lit l;
  unsigned nbBin;
  unsigned nbNotBin;
};

struct SavedStateClause {
  int idx;
  unsigned isSat : 1;
  unsigned nbUnsat : 31;
};

class CnfManagerDyn : public CnfManager {
 private:
  std::vector<unsigned> m_stackSizeListNotSatisfiedClauses;
  std::vector<unsigned> m_markedNotSatClauses;
  unsigned m_stampNotSatClauses;
  unsigned *m_notSatifiedClauses;
  unsigned m_sizeNotSatifiedClauses;

 protected:
  unsigned m_currentMarkedLitStackIndex;
  unsigned m_currentMarkedLitRemoveIndex;
  bool m_keepListNotSatisfiedClauses;

  std::vector<int> m_reviewWatcher;
  std::vector<unsigned> m_markedLitStack;
  std::vector<unsigned> m_markedLitRemove;

  std::vector<unsigned> m_indexSatClauses;
  std::vector<bool> m_markedClauseIdx;

  std::vector<SavedStateOcc> m_savedStateOccs;
  std::vector<SavedStateClause> m_savedStateClauses;
  std::vector<unsigned> m_stackPosOcc, m_stackPosClause;

  /**
   * @brief Remove from the formula the given set of satisfied clauses.
   *
   * @param idxClauses is the list of indexes.
   */
  void removeSatisfiedClauses(const std::vector<unsigned> &idxClauses);

  /**
   * @brief Call an inprocessing method for simplifying the formula.
   */
  virtual void inprocessing() {}

 public:
  /**
   * @brief Construct a new Cnf Manager Dyn object.
   *
   * @param[in] p is the proble we want to model (it is a CNF formula).
   * @param[in] keepListNotSatisfiedClauses is a boolean set to true if we want
   * to keep in memory the list of clauses that are not yet satisfied by the
   * current assignation.
   */
  CnfManagerDyn(ProblemManager &p, bool keepListNotSatisfiedClauses = false);

  /**
   * @brief Destroy the Cnf Manager Dyn object
   */
  ~CnfManagerDyn();

  /**
   * @brief Suppose that the literal in lits are true (even if it is not really
   * the case, see the pure literals) and remove the non  binary clauses where
   * this literal occurs.
   *
   * @warning there are a lot of side effects ... take care.
   *
   * @param lits is the set of literals we want to 'assign'.
   */
  void propagateTrue(const std::vector<Lit> &lits);

  /**
   * @brief Suppose that the literal in lits are false (even if it is not really
   * the case, see the pure literals) and remove the literal from the non binary
   * clauses where this literal occurs.
   *
   * @warning there are a lot of side effects ... take care.
   *
   * @param lits is the set of literals we want to 'assign'.
   */
  void propagateFalseInNotBin(const std::vector<Lit> &lits);

  /**
   * @brief This function prepares the stacks.
   */
  inline void pushStacks() {
    m_stackPosClause.push_back(m_savedStateClauses.size());
    m_stackPosOcc.push_back(m_savedStateOccs.size());
    m_currentMarkedLitStackIndex++;
  }  // pushStacks

  /**
   * @brief Unmark the clauses' that have been just put into the stack.
   */
  inline void unmarkLastClausesSaved() {
    for (unsigned i = m_stackPosClause.back(); i < m_savedStateClauses.size();
         i++)
      m_markedClauseIdx[m_savedStateClauses[i].idx] = false;
  }  // unmarkLastClausesSaved

  /**
   * @brief Get the set of not satisfied clauses regarding a given component
   * represented by the set of variables.
   *
   * @param[out] idxClauses is the set of clause indices.
   * @param[in] component is the connected component under consideration.
   */
  void getCurrentClauses(std::vector<unsigned> &idxClauses,
                         std::vector<Var> &component) override;

  /**
   * @brief Get the set of not satisfied clauses with a size larger than 2 (here
   * we speak about the initial size). We return the clauses regarding a given
   * component represented by the set of variables.
   *
   * @param[out] idxClauses is the set of clause indices.
   * @param[in] component is the connected component under consideration.
   */
  void getCurrentClausesNotBin(std::vector<unsigned> &idxClauses,
                               std::vector<Var> &component) override;

  /**
   * @brief Update the occurrence list w.r.t. a new set of assigned variables.
   * It's important that the order is conserved between the moment where    we
   * assign and the moment we unassign.
   *
   * @param[in] lits is the set of literals they are assigned to true.
   */
  void preUpdate(const std::vector<Lit> &lits) override;

  /**
   * @brief We want to come to the situation before the mirror preUpdate.
   *
   * @param lits is the set of unit literals assigned to true in the mirror
   * preUpdate.
   */
  void postUpdate(const std::vector<Lit> &lits) override;

  /**
   * Return true if no clauses containing this literal have been touched.
   *
   * @param l is the literal we are looking to check the property.
   */
  inline bool canSkipLit(const Lit &l) override {
    return m_occurrence[l.intern()].nbNotBin == 0;
  }  // canSkipLit
};
}  // namespace d4
