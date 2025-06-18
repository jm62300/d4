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
class BranchingHeuristicHybridPartialClassic : public BranchingHeuristic {
 private:
  const unsigned WORTH_CUT = 50;

  const OptionPartialOrderHeuristic &m_saveOptionPartialOrderHeuristic;
  PartialOrderHeuristic *m_partialOrder;

 public:
  /**
   * @brief Remove the defaut constructor.
   *
   */
  BranchingHeuristicHybridPartialClassic() = delete;

  /**
   * @brief Constructs a new hybrid branching heuristic that integrates both
   * classic and partial order heuristics.
   *
   * This constructor initializes a hybrid branching heuristic, combining
   * traditional heuristics (e.g., VSADS, VSIDS) with a partial order heuristic
   * for improved decision-making in model counting (or knowledge compilation).
   *
   * @param[in] options The configuration options for the branching heuristic.
   *
   * @param[in] problem A pointer to the problem manager, which handles
   * problem-specific data and operations.
   *
   * @param[in] specs A pointer to the formula manager, providing real-time
   * information about the CNF formula.
   *
   * @param[in] activityManager A reference to the activity manager, which
   * tracks variable activity levels.
   *
   * @param[in] polarityManager A reference to the polarity manager, which
   * handles polarity selection strategies.
   *
   * @param[in] out The output stream where debugging or status information is
   * printed.
   */
  BranchingHeuristicHybridPartialClassic(
      const OptionBranchingHeuristic &options, ProblemManager *problem,
      FormulaManager *specs, ActivityManager &activityManager,
      PolarityManager &polarityManager, std::ostream &out)
      : BranchingHeuristic(options, problem, specs, activityManager,
                           polarityManager, out),
        m_saveOptionPartialOrderHeuristic(options.optionPartialOrderHeuristic) {
    // Create the partial order heuristic based on the given options.
    m_partialOrder = PartialOrderHeuristic::makePartialOrderingHeuristic(
        options.optionPartialOrderHeuristic, *specs, out);

    // Ensure that the partial order heuristic was successfully initialized.
    assert(m_partialOrder);
  }

  /**
   * @brief Destructor.
   *
   */
  ~BranchingHeuristicHybridPartialClassic();

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
