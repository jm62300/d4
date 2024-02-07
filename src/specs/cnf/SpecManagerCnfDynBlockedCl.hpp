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

#include "SpecManagerCnfDyn.hpp"

namespace d4 {

class SpecManagerCnfDynBlockedCl : public SpecManagerCnfDyn {
 private:
  unsigned long m_nbBlockedClauseRemoved;
  std::vector<bool> m_isDecisionVariable;
  std::vector<bool> m_isPresentLit;

  /**
   * @brief Check all the clauses and put in idxClauses the one they are blocked
   * by a non decision literal. This method is only used for testing purpose and
   * the way blocked clauses are computed is not relied on it.
   *
   * @param idxClauses
   */
  void getBlockedClauses(std::vector<unsigned> &idxClauses);

  /**
   * @brief Remove the blocked clauses that are supported by non selected
   * variable.
   */
  void inprocessing();

 public:
  SpecManagerCnfDynBlockedCl(ProblemManager &p);

  inline void printSpecInformation(std::ostream &out) {
    std::cout << "c Number of blocked clause removed: "
              << m_nbBlockedClauseRemoved << "\n";
  }
};
}  // namespace d4
