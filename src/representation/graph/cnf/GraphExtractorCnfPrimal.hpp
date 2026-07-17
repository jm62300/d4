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

#include "../Graph.hpp"
#include "GraphExtractorCnf.hpp"

namespace d4 {
class GraphExtractorCnfPrimal : public GraphExtractorCnf {
 public:
  /**
   * @brief Construct a new Graph Extractor Cnf Primal object.
   *
   * @param simplication to activate/deactivate the simplications.
   */
  GraphExtractorCnfPrimal(bool simplication) {
    m_simplication = simplication;
  }  // constructor

  /**
   * @brief Construct a graph regarding the given CNF formula.
   *
   * @param[in] om gives information about the formula.
   * @param[in] component is the set of variables under consideration.
   * @param[out] graph is the computed graph.
   */
  void constructGraph(FormulaManager &om, std::vector<Var> &component,
                      Graph &graph) override;
};
}  // namespace d4