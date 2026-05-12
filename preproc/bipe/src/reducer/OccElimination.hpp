/**
 * reducer
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include <iostream>

#include "Method.hpp"
#include "Propagator.hpp"

namespace bipe {
namespace reducer {

/**
 * @brief Applies Occurrence Elimination to a CNF formula.
 *
 * Occurrence Elimination is a simplification technique that attempts to remove
 * redundant literals or entirely subsumed clauses by analyzing the frequency
 * and distribution of literal occurrences across the formula.
 */
class OccElimination : public Method {
 private:
  std::ostream& m_out;

  /**
   * @brief Attempts to deduce unit literals based on a central literal.
   *
   * Analyzes the given literal `l` against binary clauses containing its
   * negation (`~l`) to find forced assignments.
   *
   * @param propagator The engine managing the Boolean Constraint Propagation
   * (BCP).
   * @param l The central literal being evaluated.
   */
  void occRemoveBin(Propagator& propagator, Lit l);

  /**
   * @brief Generates a mapping of literals to the non-binary clauses they
   * appear in.
   *
   * @param propagator The propagator containing the current set of clauses.
   * @param[out] occurrence The adjacency list mapping each literal to its
   * clause references (CRef).
   */
  void generateOccurrenceLit(Propagator& propagator,
                             std::vector<std::vector<CRef>>& occurrence);

  /**
   * @brief Generates a list of literals sorted by their occurrence frequencies.
   *
   * @param propagator The propagator containing the current set of clauses.
   * @param occurrence The previously generated occurrence list.
   * @param[out] litList The resulting list of literals, ordered for processing.
   */
  void generateLitList(Propagator& propagator,
                       std::vector<std::vector<CRef>>& occurrence,
                       std::vector<Lit>& litList);

  /**
   * @brief Eliminates redundant literals from large (non-binary) clauses.
   *
   * @param propagator The propagator containing the current set of clauses.
   * @param occurrence The occurrence list mapping literals to their clauses.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void removeLitFromLargeClauses(Propagator& propagator,
                                 std::vector<std::vector<CRef>>& occurrence,
                                 const Timer& timer);

 public:
  /**
   * @brief Constructs a new OccElimination object.
   *
   * @param out The output stream used for logging information and statistics.
   */
  OccElimination(std::ostream& out);

  /**
   * @brief Runs the Occurrence Elimination process and stores the output in a
   * separate structure.
   *
   * @param nbVar The number of variables in the formula.
   * @param clauses The input set of clauses to be processed.
   * @param nbIteration The maximum number of iterations to perform. Set to -1
   *                    to run continuously until a fixed point is reached.
   * @param verbose If true, prints detailed progress information to the output
   * stream.
   * @param[out] result The resulting set of clauses after the elimination is
   * applied.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void run(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
           int nbIteration, bool verbose, std::vector<std::vector<Lit>>& result,
           const Timer& timer) override;

  /**
   * @brief Runs the Occurrence Elimination process in-place on the given
   * clauses.
   *
   * @param nbVar The number of variables in the formula.
   * @param[in,out] clauses The set of clauses to be processed and modified
   * in-place.
   * @param nbIteration The maximum number of iterations to perform. Set to
   * -1 to run continuously until a fixed point is reached.
   * @param verbose If true, prints detailed progress information to the
   * output stream.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void run(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
           int nbIteration, bool verbose, const Timer& timer) override;

  /**
   * @brief Runs the Occurrence Elimination process utilizing an existing
   * Propagator.
   *
   * @param propagator The propagator engine used to manage and execute BCP.
   * @param nbIteration The maximum number of iterations to perform. Set to -1
   *                    to run continuously until a fixed point is reached.
   * @param verbose If true, prints detailed progress information to the output
   * stream.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void run(Propagator& propagator, int nbIteration, bool verbose,
           const Timer& timer) override;

  /**
   * @brief Prints statistics about the Occurrence Elimination execution.
   *
   * Outputs metrics such as the total number of literals removed or clauses
   * simplified.
   */
  void displayInfo();
};

}  // namespace reducer
}  // namespace bipe