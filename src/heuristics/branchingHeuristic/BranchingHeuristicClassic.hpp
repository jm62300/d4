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
class BranchingHeuristicClassic : public BranchingHeuristic {
 public:
  /**
   * @brief Remove the defaut constructor.
   *
   */
  BranchingHeuristicClassic() = delete;

  /**
   * @brief Constructs a new Branching Heuristic Classic object.
   *
   * This constructor initializes a classic branching heuristic, which is
   * used for decision-making in SAT solving. It extends the base
   * BranchingHeuristic class and supports traditional heuristics such as
   * VSADS and VSIDS.
   *
   * @param[in] options The configuration options for the branching heuristic.
   * @param[in] problem A pointer to the problem manager, which handles
   * problem-specific data and operations.
   * @param[in] specs A pointer to the formula manager, providing real-time
   * information about the CNF formula.
   * @param[in] activityManager A reference to the activity manager, which
   * tracks variable activity levels.
   * @param[in] polarityManager A reference to the polarity manager, which
   * handles polarity selection strategies.
   * @param[in] out The output stream where debugging or status information is
   * printed.
   */
  BranchingHeuristicClassic(const OptionBranchingHeuristic &options,
                            ProblemManager *problem, FormulaManager *specs,
                            ActivityManager &activityManager,
                            PolarityManager &polarityManager, std::ostream &out)
      : BranchingHeuristic(options, problem, specs, activityManager,
                           polarityManager, out) {}

  /**
   * @brief Return one literal given the scoring heuristic and the phase
   * heuristic.
   *
   * @param vars is the set of variables under consideration.
   * @param[out] lits is the place where are stored the literals we are
   * considering.
   */
  void selectLitSet(std::vector<Var> &vars, ListLit &lits) override;
};
}  // namespace d4
