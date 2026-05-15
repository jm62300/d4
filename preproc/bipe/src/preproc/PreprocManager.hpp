/**
 * eliminator
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

#include <vector>

#include "OptionPreproc.hpp"

namespace bipe {

/**
 * @brief Defines the available preprocessing strategies for the formula.
 */
enum PreprocMethod {
  /**
   * @brief No preprocessing is applied.
   */
  NONE,

  /**
   * @brief Lightweight equivalence preprocessing.
   * Reduces the formula using vivification, occurrence elimination,
   * or a combination of both techniques.
   */
  EQUIV_LIGHT,

  /**
   * @brief Equivalence preprocessing.
   * Compute the backbone and equivalences.
   * Reduces the formula using vivification, occurrence elimination,
   * or a combination of both techniques.
   */
  EQUIV_FULL
};

/**
 * @brief Manages and executes preprocessing techniques on a CNF formula.
 *
 * Preprocessing is used to simplify the formula, remove redundant variables
 * and clauses, and potentially discover structural information before the
 * main solving or bipartitioning phases begin.
 */
class PreprocManager {
 public:
  /**
   * @brief Runs the selected preprocessing method on the given CNF formula.
   *
   * The formula is modified in-place based on the chosen strategy, while
   * strictly respecting the protected and projected variable sets.
   *
   * @param nbVar The total number of variables present in the CNF formula.
   * @param[in,out] clauses The CNF formula represented as a vector of clauses
   *                        (where each clause is a vector of integer literals).
   *                        This will be simplified in-place.
   * @param projected The list of variables the problem is projected onto
   *                  (used primarily in model counting contexts).
   * @param protect The list of variables that are protected and must NOT
   *                be eliminated during preprocessing.
   * @param preprocMethod The specific preprocessing strategy to apply
   *                      (e.g., NONE or EQUIV_LIGHT).
   * @param optionPreproc Additional configuration options and limits for
   *                      the preprocessing phase.
   */
  void run(unsigned nbVar, std::vector<std::vector<int>>& clauses,
           std::vector<int> projected, std::vector<int> protect,
           const PreprocMethod& preprocMethod,
           const OptionPreproc& optionPreproc);
};

}  // namespace bipe