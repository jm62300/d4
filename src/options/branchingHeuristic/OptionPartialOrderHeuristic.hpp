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

#include <map>
#include <string>
#include <vector>

#include "src/exceptions/FactoryException.hpp"
#include "src/options/EnumMetadata.hpp"
#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"
#include "src/options/OptionRegistry.hpp"
#include "src/partitioner/PartitionerManager.hpp"
#include "src/representation/graph/GraphExtractor.hpp"
#include "src/representation/hypergraph/HyperGraphExtractor.hpp"
#include "src/treeDecomposition/TreeDecomposition.hpp"
#include "src/treeDecompositioner/TreeDecompositioner.hpp"

namespace d4 {

enum PartialOrderHeuristicMethod : char {
  PARTIAL_ORDER_TREE_DECOMPOSITION,
  PARTIAL_ORDER_GIVEN,
  PARTIAL_ORDER_NONE
};

template <>
struct EnumMetadata<PartialOrderHeuristicMethod> {
  static std::string name() { return "PartialOrderHeuristicMethod"; }
  static std::map<int, std::string> mapping() {
    return {{PARTIAL_ORDER_TREE_DECOMPOSITION, "tree-decomposition"},
            {PARTIAL_ORDER_GIVEN, "given"},
            {PARTIAL_ORDER_NONE, "none"}};
  }
};

class OptionPartialOrderHeuristic : public OptionGroup {
 public:
  OptionPartialOrderHeuristic(
      const std::string& name = "partialOrder",
      const std::string& description = "Partial order options")
      : OptionGroup(name, description) {}

  /** @brief The method used to compute a cut. [none, tree-decomposition] */
  Option<PartialOrderHeuristicMethod> partialOrderMethod{
      "partialOrderMethod", "The method used to compute a cut",
      PARTIAL_ORDER_TREE_DECOMPOSITION};

  /** @brief The partitioner we will call (patoh). */
  Option<PartitionerName> partitionerName{
      "partitionerName", "The partitioner we will call", PARTITIONER_PATOH};

  /** @brief The tree decomposition technique used. */
  Option<TreeDecompositionMethod> treeDecompositionMethod{
      "treeDecompositionMethod", "The tree decomposition technique used",
      TREE_DECOMP_TREE_WIDTH};

  /** @brief The tool used for computing the tree decomposition. */
  Option<TreeDecompositionerMethod> treeDecompositionerMethod{
      "treeDecompositionerMethod",
      "The tool used for computing the tree decomposition",
      TREE_DECOMP_TOOL_FLOW_CUTTER};

  /** @brief The hyper graph representation used. */
  Option<HyperGraphExtractorMethod> hyperGraphExtractorMethod{
      "hyperGraphExtractorMethod", "The hyper graph representation used",
      HYPER_GRAPH_DUAL};

  /** @brief The graph representation used. */
  Option<GraphExtractorMethod> graphExtractorMethod{
      "graphExtractorMethod", "The graph representation used", GRAPH_PRIMAL};

  /** @brief Set to true if the graph extractor use some simplification. */
  Option<bool> useSimpGraphExtractor{
      "useSimpGraphExtractor", "If the graph extractor use some simplification",
      true};

  Option<unsigned> budget{"budget", "The budget for partitioning", 100};

  Option<unsigned> seed{"seed", "The seed for random number generator", 2911};

  /** @brief Above this maximum degree the htd tree decompositioner falls back
   * from min-fill to min-degree ordering. */
  Option<unsigned> maxDegreeForMinFill{
      "maxDegreeForMinFill",
      "The maximum degree above which the htd tree decompositioner falls back "
      "from min-fill to min-degree ordering",
      256};

  // vector is harder to handle in Option<T> with setFromString.
  // We keep it as is for now or use a specialization.
  std::vector<double> givenOrder;
  Option<double> scaleFactor{"scaleFactor", "The scale factor", 0};

  std::vector<OptionBase*> getAllOptions() override {
    return {(OptionBase*)&partialOrderMethod,
            (OptionBase*)&partitionerName,
            (OptionBase*)&treeDecompositionMethod,
            (OptionBase*)&treeDecompositionerMethod,
            (OptionBase*)&hyperGraphExtractorMethod,
            (OptionBase*)&graphExtractorMethod,
            (OptionBase*)&useSimpGraphExtractor,
            (OptionBase*)&budget,
            (OptionBase*)&seed,
            (OptionBase*)&maxDegreeForMinFill,
            (OptionBase*)&scaleFactor};
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionPartialOrderHeuristic& dt) {
    out << " Option Partitioning Heuristic:"
        << " method(" << dt.partialOrderMethod.getValueAsString()
        << ") partioner name(" << dt.partitionerName.getValueAsString()
        << ") treeDecomposition Method("
        << dt.treeDecompositionMethod.getValueAsString() << ")";
    return out;
  }
};
}  // namespace d4