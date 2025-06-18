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

#include "src/representation/graph/Graph.hpp"
#include "src/treeDecomposition/TreeDecomposition.hpp"

namespace d4 {

enum TreeDecompositionerMethod { TREE_DECOMP_TOOL_FLOW_CUTTER };

class TreeDecompositionerMethodManager {
 public:
  static std::string getTreeDecompositionerMethodManager(
      const TreeDecompositionerMethod &m) {
    if (m == TREE_DECOMP_TOOL_FLOW_CUTTER) return "flow-cutter";

    throw(FactoryException("Tree Decomposition method type unknown", __FILE__,
                           __LINE__));
  }  // getTreeDecompositionerMethodManager

  static TreeDecompositionerMethod getTreeDecompositionerMethodManager(
      const std::string &m) {
    if (m == "flow-cutter") return TREE_DECOMP_TOOL_FLOW_CUTTER;

    throw(FactoryException("Tree Decomposition method unknown", __FILE__,
                           __LINE__));
  }  // getTreeDecompositionerMethodManager
};

class TreeDecompositioner {
 public:
  virtual ~TreeDecompositioner() {}

  /**
   * @brief Factory.
   *
   * @param method is the tree decompositioner we want to use.
   * @return an object that compute a tree decomposition.
   */
  static TreeDecompositioner *makeTreeDecompositionMethod(
      const TreeDecompositionerMethod &method);

  /**
   * @brief Compute a tree decomposition regarding a given graph.
   *
   * @param graph is the graph we want to handle.
   * @param budget is a time in second the method is run.
   * @param seed is the seed use if the method use random numbers.
   * @param verbose controls the verbosity.
   *
   * @return a tree decomposition of graph.
   */
  virtual TreeDecomp *constructTreeDecomposition(Graph &graph, unsigned budget,
                                                 unsigned seed,
                                                 bool verbose) = 0;
};

}  // namespace d4