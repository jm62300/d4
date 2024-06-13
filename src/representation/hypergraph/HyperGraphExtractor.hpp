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
#include "src/formulaManager/cnf/CnfManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

class OptionPartialOrderHeuristic;

enum HyperGraphExtractorMethod { HYPER_GRAPH_DUAL };

class HyperGraphExtractorMethodManager {
 public:
  static std::string getHyperGraphExtractorMethodManager(
      const HyperGraphExtractorMethod &m) {
    if (m == HYPER_GRAPH_DUAL) return "dual";

    throw(FactoryException("Paritioning method type unknown", __FILE__,
                           __LINE__));
  }  // getHyperGraphExtractorMethodManager

  static HyperGraphExtractorMethod getHyperGraphExtractorMethodManager(
      const std::string &m) {
    if (m == "dual") return HYPER_GRAPH_DUAL;

    throw(FactoryException("Hyper Graph method unknown", __FILE__, __LINE__));
  }  // getHyperGraphExtractorMethodManager
};

struct InfoHyperGraph {
  unsigned maxNbEdges;
  unsigned maxNbNodes;
  unsigned sumEdgeSizes;
};

class HyperGraphExtractor {
 public:
  /**
   * @brief Factory.
   *
   * @param method is the representation used for representing the formula.
   * @param inType is the type of formula.
   *
   * @return an hyper graph extractor.
   */
  static HyperGraphExtractor *makeHyperGraphExtractor(
      const HyperGraphExtractorMethod &method, const ProblemInputType &inType);

  /**
   * @brief Virtual destructor.
   *
   */
  virtual ~HyperGraphExtractor() {}

  /**
   * @brief Compute the hypergraph given a formual.
   *
   * @param[in] om gives information about the formula.
   * @param[in] component is the set of variables under consideration.
   * @param[out] hypergraph is the computed hypergraph.
   *
   * @return information about the hyper graph size in the worst case.
   */
  virtual InfoHyperGraph constructHyperGraph(FormulaManager &om,
                                             std::vector<Var> &component,
                                             HyperGraph &hypergraph) = 0;

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
  virtual void split(HyperGraph &graph, std::vector<int> &partition,
                     std::vector<Var> &cut, HyperGraph &firstGraph,
                     HyperGraph &secondGraph) = 0;
};
}  // namespace d4
