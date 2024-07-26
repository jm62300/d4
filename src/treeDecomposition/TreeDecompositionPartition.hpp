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

#include "TreeDecomposition.hpp"
#include "src/partitioner/PartitionerManager.hpp"
#include "src/representation/hypergraph/HyperGraphExtractor.hpp"

namespace d4 {

struct Strata {
  TreeDecomp *father;
  HyperGraph graph;
};

class TreeDecompositionPartition : public TreeDecomposition {
 private:
  PartitionerName m_partitionerName = PARTITIONER_PATOH;
  HyperGraphExtractorMethod m_hyperGraphExtractorMethod = HYPER_GRAPH_DUAL;

 public:
  /**
   * @brief Construct a new Tree Decomposition Cnf Partition object
   *
   * @param[in] partitionName is the name of the partioner used.
   * @param[in] hyperGraphExtractorMethod is way the formula is mapped as an
   * hyper graph.
   * @param[in] pbType gives the type of problem we are handling.
   */
  TreeDecompositionPartition(
      const PartitionerName partitionName,
      const HyperGraphExtractorMethod hyperGraphExtractorMethod,
      const ProblemInputType pbType);

  /**
   * @brief Compute a tree decomposition on a CNF using a partitioner
   * recursively.
   *
   * @param[in] om gives information about the CNF formula.
   *
   * @return the computed decomposition.
   */
  TreeDecomp *computeDecomposition(FormulaManager &om) override;
};
}  // namespace d4
