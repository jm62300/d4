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

#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"
#include "src/partitioner/PartitionerManager.hpp"
#include "src/representation/graph/GraphExtractor.hpp"
#include "src/representation/hypergraph/HyperGraphExtractor.hpp"
#include "src/treeDecomposition/TreeDecomposition.hpp"
#include "src/treeDecompositioner/TreeDecompositioner.hpp"

namespace d4 {

struct ConfigurationPartialOrderHeuristic {
  /** @brief The method used to compute a cut. [none, tree-decomposition] */
  PartialOrderHeuristicMethod partialOrderMethod = PARTIAL_ORDER_NONE;
  /** @brief The partitioner we will call (patoh). */
  PartitionerName partitionerName = PARTITIONER_NONE;
  /** @brief The tree decomposition technique used (tree-partition, tree-width). */
  TreeDecompositionMethod treeDecompositionMethod = TREE_DECOMP_PARTITION;
  /** @brief The tool used for computing the tree decomposition (flow-cutter). */
  TreeDecompositionerMethod treeDecompositionerMethod =
      TREE_DECOMP_TOOL_FLOW_CUTTER;
  /** @brief The hyper graph representation used (dual). */
  HyperGraphExtractorMethod hyperGraphExtractorMethod = HYPER_GRAPH_DUAL;
  /** @brief The graph representation used (primal). */
  GraphExtractorMethod graphExtractorMethod = GRAPH_PRIMAL;
  /** @brief Set to true if the graph extractor use some simplification. */
  bool useSimpGraphExtractor = true;
  unsigned budget = 100;
  unsigned seed = 2911;
  bool verbosity = false;
  std::vector<double> givenOrder;
  double scaleFactor = 0;
};
}  // namespace d4