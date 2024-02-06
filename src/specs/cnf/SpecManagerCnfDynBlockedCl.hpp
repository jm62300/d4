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

#include "SpecManagerCnf.hpp"

namespace d4 {

class SpecManagerCnfDynBlockedCl : public SpecManagerCnf {
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

 private:
  unsigned long m_nbPureSimplification;

  std::vector<int> m_reviewWatcher;
  unsigned m_currentMarkedLitIndex;
  std::vector<char> m_markedLit;
  std::vector<bool> m_markedClauseIdx;
  std::vector<unsigned> m_indexSatClauses;

  std::vector<SavedStateOcc> m_savedStateOccs;
  std::vector<SavedStateClause> m_savedStateClauses;
  std::vector<Lit> m_savedPureLits;
  std::vector<unsigned> m_stackPosOcc, m_stackPosClause, m_stackPosPure;

  std::vector<bool> m_isDecisionVariable;
  std::vector<bool> m_markedPureLiteral;
  std::vector<Lit> m_pureDetected;

  /**
   * @brief Suppose that the literal in lits are true (even if it is not really
   * the case, see the pure literals) and remove the non  binary clauses where
   * this literal occurs.
   *
   * @warning there are a lot of side effects ... take care.
   *
   * @param lits is the set of literals we want to 'assign'.
   */
  void propagateTrueInNotBin(const std::vector<Lit> &lits);

  /**
   * @brief Suppose that the literal in lits are true (even if it is not really
   * the case, see the pure literals) and remove the binary clauses where
   * this literal occurs.
   *
   * @warning there are a lot of side effects ... take care.
   *
   * @param lits is the set of literals we want to 'assign'.
   */
  void propagateTrueInBin(const std::vector<Lit> &lits);

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
   * @brief Remove from the formula the given set of satisfied clauses.
   *
   * @param idxClauses is the list of indexes.
   */
  void removeSatisfiedClauses(const std::vector<unsigned> &idxClauses);

  /**
   * @brief Search for the list of pure literals that are not decision
   * variables.
   *
   * @param[out] pureLits is the computed pure literals.
   */
  void getPureLiterals(std::vector<Lit> &pureLits);

  /**
   * @brief Compute the set of pure literals present in the formula and fix them
   * to true.
   *
   */
  void affectInitPureLit();

  /**
   * @brief This function research in the last literals pushed into the stack
   * the one that are pure.
   *
   * @return true if some pure literals have been decteted, false otherwise.
   */
  bool searchPureLitOnTheStack();

  /**
   * @brief Check all the clauses and put in idxClauses the one they are blocked
   * by a non decision literal.
   *
   * @param idxClauses
   */
  void getBlockedClauses(std::vector<unsigned> &idxClauses);

 public:
  SpecManagerCnfDynBlockedCl(ProblemManager &p);

  /**
   * @brief Update the occurrence list w.r.t. a new set of assigned variables.
   * It's important that the order is conserved between the moment where    we
   * assign and the moment we unassign.
   *
   * @param[in] lits is the set of literals they are assigned to true.
   */
  void preUpdate(const std::vector<Lit> &lits) override;

  void postUpdate(const std::vector<Lit> &lits) override;

  inline void printSpecInformation(std::ostream &out) {
    std::cout << "c Number of pure literal simplication: "
              << m_nbPureSimplification << "\n";
  }

  // we cannot use this function here
  inline void initialize(std::vector<Var> &setOfVar, std::vector<Lit> &units) {
    assert(0);
  }
};
}  // namespace d4
