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

#include "TreeDecompositionCnfPartition.hpp"

namespace d4 {

/**
 * @brief TreeDecompositionCnfPartition::TreeDecompositionCnfPartition
 * implementation.
 */
TreeDecompositionCnfPartition::TreeDecompositionCnfPartition(
    const PartitionerName partitionerName,
    const HyperGraphExtractorMethod hyperGraphExtractorMethod)
    : m_partitionerName(partitionerName),
      m_hyperGraphExtractorMethod(hyperGraphExtractorMethod) {}  // constructor.

/**
 * @brief TreeDecompositionCnfPartition::computeDecomposition implementation.
 *
 */
TreeDecomp *TreeDecompositionCnfPartition::computeDecomposition(
    SpecManager &om) {
  TreeDecomp *tree = NULL;

  // compute the hypergraph.
  HyperGraphExtractor *hextract = HyperGraphExtractor::makeHyperGraphExtractor(
      m_hyperGraphExtractorMethod, PB_CNF);
  std::vector<Var> component, notLinked;
  for (unsigned i = 1; i <= om.getNbVariable(); i++) {
    if (!om.getNbOccurrence(i))
      notLinked.push_back(i);
    else
      component.push_back(i);
  }

  HyperGraph graph;
  hextract->constructHyperGraph(om, component, graph);

  // extract the decomposition.

  if (!notLinked.size()) return tree;
  return new TreeDecomp(notLinked, {tree});
}  // computeDecomposition

}  // namespace d4