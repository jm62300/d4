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

#include <cstdint>
#include <ostream>
#include <vector>

#include "PartialOrderHeuristic.hpp"
#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {

class PartialOrderHeuristicTreeDecomp : public PartialOrderHeuristic {
 protected:
  double m_scaleFactor;
  std::vector<double> m_topologicalOrder;
  unsigned m_treeWidth;

  unsigned m_level;

 public:
  /**
   * @brief Constructor.
   *
   * @param options gives the list of options.
   * @param om is the object which deal with the formula.
   * @param s is a SAT solver associate with the formula under consideration.
   * @param out is the output stream.
   */
  PartialOrderHeuristicTreeDecomp(const OptionPartialOrderHeuristic &options,
                                  FormulaManager &om, std::ostream &out);

  ~PartialOrderHeuristicTreeDecomp();

  void computeCutSet(std::vector<Var> &component,
                     std::vector<Var> &cutSet) override;

  /**
   * @brief Get the partial order regarding the topological order.
   *
   * @param v is a variable.
   *
   * @return the position of v in the order.
   */
  inline double getPartialOrder(Var v) override {
    return m_topologicalOrder[v];
  }  // getPartialOrder

  /**
   * @brief Retrieves the computed tree width of the decomposition.
   *
   * @return The tree width of the current tree decomposition.
   */
  inline unsigned getTreeWidth() { return m_treeWidth; }
};
}  // namespace d4
