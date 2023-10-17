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

#include "PhaseHeuristic.hpp"
#include "ScoringMethod.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"

namespace d4 {
class BranchingHeuristic {
 private:
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;

 public:
  /**
   * @brief Remove the defaut constructor.
   *
   */
  BranchingHeuristic() = delete;

  BranchingHeuristic(const OptionBranchingHeuristic &options,
                     SpecManager *m_specs, WrapperSolver *m_solver,
                     std::ostream &out);

  /**
   * @brief Select a list of literals we want to branch on it in a deterministic
   * way.
   *
   * @param vars is the set of variables under consideration.
   * @param s can give information about the formula.
   * @param isDecisionVariable are the variable we can decide on.
   * @return a set of literals.
   */
  std::vector<Lit> selectLitSet(std::vector<Var> &vars, SpecManager &s,
                                std::vector<bool> &isDecisionVariable);
};
}  // namespace d4