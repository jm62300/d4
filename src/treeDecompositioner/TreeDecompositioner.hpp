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

#include <string>
#include <map>
#include "src/representation/graph/Graph.hpp"
#include "src/treeDecomposition/TreeDecomposition.hpp"
#include "src/options/EnumMetadata.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

enum TreeDecompositionerMethod { TREE_DECOMP_TOOL_FLOW_CUTTER };

template <>
struct EnumMetadata<TreeDecompositionerMethod> {
  static std::string name() { return "TreeDecompositionerMethod"; }
  static std::map<int, std::string> mapping() {
    return {{TREE_DECOMP_TOOL_FLOW_CUTTER, "flow-cutter"}};
  }
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