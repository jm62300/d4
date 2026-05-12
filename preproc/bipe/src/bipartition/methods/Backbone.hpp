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

#include "src/bipartition/methods/Method.hpp"
#include "src/bipartition/option/OptionBackbone.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace bipartition {

/**
 * @brief Computes the backbone of a Boolean formula.
 *
 * The backbone of a formula consists of the set of literals that evaluate
 * to true in all satisfying assignments (models) of the formula. Extracting
 * the backbone allows for significant simplification of the problem before
 * attempting bipartitioning or counting.
 */
class Backbone : public Method {
 private:
  class WrapperSolver* m_solver = nullptr;

 public:
  /**
   * @brief Constructs a new Backbone object.
   */
  Backbone() = default;

  /**
   * @brief Destroys the Backbone object.
   */
  ~Backbone() = default;

  /**
   * @brief Executes the backbone extraction process.
   *
   * @param p The problem instance to analyze.
   * @param[out] backbone The collection where the extracted backbone literals
   *                      (represented as unit gates) are stored.
   * @param[in,out] setOfModels A collection used to cache any valid complete
   * models discovered during the extraction process.
   * @param option Configuration options controlling solver limits (e.g.,
   * conflicts), heuristics, and the specific backend solver to instantiate.
   * @param out The output stream used for logging progress and statistics.
   * @param timer The timer used to strictly enforce execution time limits.
   * @return True if the problem remains satisfiable, false if it is proven
   * UNSAT.
   */
  bool run(Problem& p, std::vector<Gate>& backbone,
           std::vector<std::vector<bool>>& setOfModels, OptionBackbone option,
           std::ostream& out, const Timer& timer);
};

}  // namespace bipartition
}  // namespace bipe