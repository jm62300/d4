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

#include <vector>

#include "../TreeDecomposition.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
class TreeDecompositionCnf : public TreeDecomposition {
 public:
  /**
   * @brief This function check if a given decomposition (given as a partial
   * order) forms really a tree decomposition of the CNF given as parameter.
   *
   * @param[in] decomposition is the decomposition we want to test.
   * @param[in] om gives information about the formula under consideration.
   * @param[in] component is the set of variables we are focusing on.
   */
  void checkDecomposition(const std::vector<std::vector<Var>> &decomposition,
                          SpecManager &om,
                          const std::vector<Var> &component) override;
};
}  // namespace d4