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

#include "../HyperGraph.hpp"
#include "../HyperGraphExtractor.hpp"
#include "src/formulaManager/cnf/CnfManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
class HyperGraphExtractorCnfDual : public HyperGraphExtractor {
 private:
  char *m_data = NULL;

 public:
  /**
   * @brief Destroy the Hyper Graph Extractor Cnf Dual object
   */
  ~HyperGraphExtractorCnfDual() override;

  /**
   * @brief Given the problem definition, we construct the hypergraph following
   * the dual representation.
   *
   * @param[in] om is the formula representation.
   * @param[in] component is the set of variables under consideration.
   * @param[out] hypergraph is the computed hypergraph.
   *
   * @return information about the hyper graph size in the worst case.
   */
  InfoHyperGraph constructHyperGraph(FormulaManager &om,
                                     std::vector<Var> &component,
                                     HyperGraph &hypergraph);

  /**
   * @brief This function divide into two set of edges (actually we return the
   * indexes) regarding a given partition. This function also computes the edges
   * that are in conflict.
   *
   * @param[in] graph is the graph we want to split.
   * @param[in] partition is the partition under consideration.
   * @param[out] cut is the computed cutset.
   * @param[out] firstGraph is the set of edges in the first partition.
   * @param[out] secondGraph is the set of edges in the second partition.
   */
  void split(HyperGraph &graph, std::vector<int> &partition,
             std::vector<Var> &cut, HyperGraph &firstGraph,
             HyperGraph &secondGraph) override;
};
}  // namespace d4
