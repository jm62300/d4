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

#include "CnfManagerDyn.hpp"

namespace d4 {
class CnfManagerDynPure : public CnfManagerDyn {
 private:
  unsigned long m_nbPureSimplification;

  std::vector<Lit> m_savedPureLits;
  std::vector<unsigned> m_stackPosPure;

  std::vector<bool> m_isDecisionVariable;
  std::vector<bool> m_markedPureLiteral;
  std::vector<Lit> m_pureDetected;

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
  void inprocessing() override;

 public:
  CnfManagerDynPure(ProblemManager &p);

  inline void printSpecInformation(std::ostream &out) {
    std::cout << "c Number of pure literal simplication: "
              << m_nbPureSimplification << "\n";
  }
};
}  // namespace d4
