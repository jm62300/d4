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

#include "TreeDecompositionPartition.hpp"

namespace d4 {

/**
 * @brief TreeDecompositionPartition::TreeDecompositionPartition implementation.
 */
TreeDecompositionPartition::TreeDecompositionPartition(
    const PartitionerName partitionerName,
    const HyperGraphExtractorMethod hyperGraphExtractorMethod,
    const ProblemInputType pbType)
    : m_partitionerName(partitionerName),
      m_hyperGraphExtractorMethod(hyperGraphExtractorMethod) {}  // constructor.

/**
 * @brief TreeDecompositionCnfPartition::computeDecomposition implementation.
 *
 */
TreeDecomp *TreeDecompositionPartition::computeDecomposition(
    FormulaManager &om) {
  TreeDecomp *tree = NULL;

  // compute the hypergraph.
  HyperGraphExtractor *hextract = HyperGraphExtractor::makeHyperGraphExtractor(
      m_hyperGraphExtractorMethod, om.getProblemInputType());
  std::vector<Var> component, notLinked;
  for (unsigned i = 1; i <= om.getNbVariable(); i++) {
    if (om.isFreeVariable(i))
      notLinked.push_back(i);
    else
      component.push_back(i);
  }

  HyperGraph graph;
  InfoHyperGraph infoHyperGraph =
      hextract->constructHyperGraph(om, component, graph);
  assert(component.size() || !graph.getNbEdges());

  // extract the decomposition.
  PartitionerManager *partitioner = PartitionerManager::makePartitioner(
      m_partitionerName, infoHyperGraph, std::cout);
  std::vector<int> partition(infoHyperGraph.maxNbNodes + 1, 0);

  // construct the tree decomposition recursively by calling a partitioner.
  std::vector<Strata> stack;
  stack.push_back({NULL, graph});

  while (stack.size()) {
    Strata t = stack.back();
    stack.pop_back();
    if (!t.graph.getNbEdges()) continue;

    assert(t.father || !tree);
    TreeDecomp *currentTree = new TreeDecomp();
    if (!tree) tree = currentTree;
    if (t.father) t.father->getSons().push_back(currentTree);

    stack.push_back({currentTree, HyperGraph()});
    stack.push_back({currentTree, HyperGraph()});

    partitioner->computePartition(t.graph, PartitionerManager::QUALITY,
                                  partition);

    hextract->split(t.graph, partition, currentTree->getNode(),
                    stack[stack.size() - 1].graph,
                    stack[stack.size() - 2].graph);
  }

  delete partitioner;
  delete hextract;

  if (!notLinked.size()) return tree;
  if (!component.size()) return new TreeDecomp(notLinked, {});
  return new TreeDecomp(notLinked, {tree});
}  // computeDecomposition

}  // namespace d4