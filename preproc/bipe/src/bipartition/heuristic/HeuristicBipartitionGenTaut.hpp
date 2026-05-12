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
 * @brief Bipartition heuristic based on tautology generation.
 *
 * This heuristic strategy determines the testing order of variables by
 * leveraging tautology detection or generation mechanisms. It prioritizes
 * assumptions that quickly lead to tautological simplifications or help
 * identify redundant variables during the bipartitioning process.
 */
class HeuristicBipartitionGenTaut : public HeuristicBipartition {
 public:
  /**
   * @brief Constructs a new HeuristicBipartitionGenTaut object.
   *
   * @param p The problem instance for which we are computing the bipartition.
   * @param selectors The set of selector literals used to activate or
   * deactivate specific equivalences in the formula.
   */
  HeuristicBipartitionGenTaut(Problem& p, const std::vector<Lit>& selectors);
};

}  // namespace bipartition
}  // namespace bipe