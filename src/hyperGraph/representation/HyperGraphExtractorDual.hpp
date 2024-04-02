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

#include <iostream>
#include <vector>

#include "HyperGraph.hpp"
#include "HyperGraphExtractor.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"

namespace d4 {
class HyperGraphExtractorDual : public HyperGraphExtractor {
 public:
  /**
   * @brief Given the problem definition, we construct the hypergraph following
   * the dual representation.
   *
   * @param[in] om is the formula representation.
   * @param[in] component is the set of variables under consideration.
   * @param[out] hypergraph is the computed hypergraph.
   */
  void constructHyperGraph(SpecManagerCnf &om, std::vector<Var> &component,
                           HyperGraph &hypergraph);
};
}  // namespace d4
