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

#include "TreeDecompositioner.hpp"

namespace d4 {
class TreeDecompositionerFlowCutter : public TreeDecompositioner {
 private:
  /**
   * @brief Get the center of the graph using the onion algorithm.
   *
   * @param graph is the given graph.
   *
   * @return a node from the graph that will represent the center. If the graph
   * is emptied then we return the last node + 1 (graph.size()).
   */
  unsigned getCenterGraph(const std::vector<std::vector<unsigned>> &graph);

  /**
   * @brief From a given graph and a center, constructs a tree that is stored in
   * setOfTrees.
   *
   * @param[in] graph is the given graph.
   * @param[in] center is the center we choose in graph.
   * @param[out] setOfTrees the resulting tree.
   * @param[out] marked is a boolean vector used in order to mark the visited
   * nodes.
   */
  void makeTreeFromGraph(const std::vector<std::vector<unsigned>> &graph,
                         unsigned center, std::vector<TreeDecomp *> &setOfTrees,
                         std::vector<bool> &marked);

 public:
  /**
   * @brief Construct a new Tree Decompositioner Flow Cutter object.
   *
   */
  TreeDecompositionerFlowCutter() {
    std::cout << "c [TREE DECOMPOSITION] Flow Cutter\n";
  }

  /**
   * @brief Compute a tree decomposition regarding a given graph using the tool
   * flow-cutter.
   *
   * @param graph is the graph we want to handle.
   * @param budget is a time in second the method is run.
   * @param seed is the seed use if the method use random numbers.
   * @param verbosity controls the verbosity.
   *
   * @return a tree decomposition of graph.
   */
  TreeDecomp *constructTreeDecomposition(Graph &graph, unsigned budget,
                                         unsigned seed,
                                         bool verbosity) override;
};
}  // namespace d4