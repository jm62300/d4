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
 * @brief Applies a combination of reduction techniques to a CNF formula.
 *
 * This method orchestrates multiple simplification strategies (such as
 * Vivification and Occurrence Elimination) in sequence to maximize the
 * reduction of variables and clauses within the formula.
 */
class Combinaison : public Method {
 private:
  std::ostream& m_out;

  Method* m_vivifier = nullptr;
  Method* m_occEliminator = nullptr;

 public:
  /**
   * @brief Constructs a new Combinaison object.
   *
   * @param out The output stream used for logging information and statistics.
   */
  Combinaison(std::ostream& out);

  /**
   * @brief Runs the combined reduction process and stores the output in a
   * separate structure.
   *
   * @param nbVar The number of variables in the formula.
   * @param clauses The input set of clauses to be processed.
   * @param nbIteration The maximum number of iterations to perform per
   * underlying method. Set to -1 to run continuously until a fixed point is
   * reached.
   * @param verbose If true, prints detailed progress information to the output
   * stream.
   * @param[out] result The resulting set of clauses after all reductions are
   * applied.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void run(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
           int nbIteration, bool verbose, std::vector<std::vector<Lit>>& result,
           const Timer& timer);

  /**
   * @brief Runs the combined reduction process in-place on the given clauses.
   *
   * @param nbVar The number of variables in the formula.
   * @param[in,out] clauses The set of clauses to be processed and modified
   * in-place.
   * @param nbIteration The maximum number of iterations to perform per
   * underlying method. Set to -1 to run continuously until a fixed point is
   * reached.
   * @param verbose If true, prints detailed progress information to the output
   * stream.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void run(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
           int nbIteration, bool verbose, const Timer& timer);

  /**
   * @brief Runs the combined reduction process utilizing an existing
   * Propagator.
   *
   * @param propagator The propagator engine used to manage and execute BCP.
   * @param nbIteration The maximum number of iterations to perform per
   * underlying method. Set to -1 to run continuously until a fixed point is
   * reached.
   * @param verbose If true, prints detailed progress information to the output
   * stream.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void run(Propagator& propagator, int nbIteration, bool verbose,
           const Timer& timer);

  /**
   * @brief Prints statistics about the combined execution.
   *
   * Outputs aggregated metrics from all the underlying reduction methods
   * managed by this class.
   */
  void displayInfo();
};

}  // namespace reducer
}  // namespace bipe