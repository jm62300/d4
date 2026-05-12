/**
 * bipe
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

#include "src/bipartition/heuristic/HeuristicBipartition.hpp"
#include "src/utils/Problem.hpp"

namespace bipe {
namespace bipartition {

/**
 * @brief Bipartition heuristic based on the natural variable order.
 *
 * This heuristic strategy determines the testing order of variables based
 * strictly on their original numerical identifiers (i.e., the order in which
 * they were defined in the problem). It serves as a straightforward baseline
 * approach without applying any complex sorting or frequency analysis.
 */
class HeuristicBipartitionNaturalOrder : public HeuristicBipartition {
 public:
  /**
   * @brief Constructs a new HeuristicBipartitionNaturalOrder object.
   *
   * Initializes the assumption list by preserving the natural indexing
   * of the variables.
   *
   * @param p The problem instance for which we are computing the bipartition.
   * @param selectors The set of selector literals used to activate or
   * deactivate specific equivalences in the formula.
   */
  HeuristicBipartitionNaturalOrder(Problem& p,
                                   const std::vector<Lit>& selectors);
};

}  // namespace bipartition
}  // namespace bipe