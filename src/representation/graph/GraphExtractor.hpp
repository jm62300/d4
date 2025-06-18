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

#include "Graph.hpp"
#include "src/formulaManager/FormulaManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
enum GraphExtractorMethod { GRAPH_PRIMAL };

class GraphExtractorMethodManager {
 public:
  static std::string getGraphExtractorMethodManager(
      const GraphExtractorMethod &m) {
    if (m == GRAPH_PRIMAL) return "primal";

    throw(FactoryException("Graph extractor method type unknown", __FILE__,
                           __LINE__));
  }  // getGraphExtractorMethodManager

  static GraphExtractorMethod getGraphExtractorMethodManager(
      const std::string &m) {
    if (m == "primal") return GRAPH_PRIMAL;

    throw(
        FactoryException("Graph extractor method unknown", __FILE__, __LINE__));
  }  // getGraphExtractorMethodManager
};

class GraphExtractor {
 protected:
  bool m_simplication;

 public:
  virtual ~GraphExtractor() {}

  /**
   * @brief Construct a graph regarding the given formula.
   *
   * @param[in] om gives information about the formula.
   * @param[in] component is the set of variables under consideration.
   * @param[out] graph is the computed graph.
   */
  virtual void constructGraph(FormulaManager &om, std::vector<Var> &component,
                              Graph &graph) = 0;

  /**
   * @brief Factory.
   *
   * @param method is the representation used for representing the formula.
   * @param simplification is set to true if we want to apply some
   * simplification.
   * @param inType is the type of formula.
   *
   * @return a graph extractor.
   */
  static GraphExtractor *makeGraphExtractor(const GraphExtractorMethod &method,
                                            bool simplification,
                                            const ProblemInputType &inType);
};
}  // namespace d4