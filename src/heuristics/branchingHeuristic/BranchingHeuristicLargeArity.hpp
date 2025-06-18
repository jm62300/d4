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

#include "BranchingHeuristic.hpp"

namespace d4 {
class BranchingHeuristicLargeArity : public BranchingHeuristic {
 private:
  unsigned m_limitClause;
  std::vector<int> m_indexOfLargeClause;
  std::vector<bool> m_markedVar;

 public:
  /**
   * @brief Remove the defaut constructor.
   *
   */
  BranchingHeuristicLargeArity() = delete;

  /**
   * @brief Constructs a new large-arity branching heuristic.
   *
   * This constructor initializes a branching heuristic designed to handle
   * decision-making in SAT solvers, particularly when dealing with high-arity
   * constraints. It integrates variable activity tracking and polarity
   * selection strategies to improve search efficiency.
   *
   * @param[in] options The configuration options for the branching heuristic.
   * @param[in] problem A pointer to the problem manager, responsible for
   * handling problem-specific data and operations.
   * @param[in] specs A pointer to the formula manager, which provides real-time
   * information about the CNF formula.
   * @param[in] activityManager A reference to the activity manager, which
   * tracks variable activity levels to guide branching.
   * @param[in] polarityManager A reference to the polarity manager, which
   * handles polarity selection strategies.
   * @param[in] out The output stream used for logging or debugging information.
   */
  BranchingHeuristicLargeArity(const OptionBranchingHeuristic &options,
                               ProblemManager *problem, FormulaManager *specs,
                               ActivityManager &activityManager,
                               PolarityManager &polarityManager,
                               std::ostream &out);

  /**
   * @brief If a large constraint exists (that is constraint with more than
   * m_limitClause literals) we branch on priority on the set of variables
   * of this constraint. Otherwise, the classical heuristic is used (that is
   * we select the variable and the phase according to the m_hVar and
   * m_hPhase objects).
   *
   * @param vars is the set of variables under consideration.
   * @param[out] lits is the place where are stored the literals we are
   * considering.
   */
  void selectLitSet(std::vector<Var> &vars, ListLit &lits) override;
};
}  // namespace d4
