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

#include "TreeDecompositionerHtd.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

#include "3rdParty/htd/main.hpp"

namespace d4 {

/**
 * @brief TreeDecompositionerHtd::constructTreeDecomposition implementation.
 */
TreeDecomp* TreeDecompositionerHtd::constructTreeDecomposition(Graph& graph,
                                                               unsigned budget,
                                                               unsigned seed,
                                                               bool verbose) {
  auto start = std::chrono::system_clock::now();

  if (!graph.getNbNode())
    return new TreeDecomp(std::vector<Var>(), std::vector<TreeDecomp*>());

  std::srand(seed);
  std::unique_ptr<htd::LibraryInstance> instance(
      htd::createManagementInstance(htd::Id::FIRST));

  // d4's graph nodes are the variables 1..nbNode, matching htd's vertex
  // numbering that also starts at 1.
  htd::MultiHypergraph hGraph(instance.get());
  hGraph.addVertices(graph.getNbNode());
  std::vector<unsigned> degree(graph.getNbNode() + 1, 0);
  for (auto& e : graph.getEdge()) {
    assert(e.first && e.first <= graph.getNbNode());
    assert(e.second && e.second <= graph.getNbNode());
    if (e.first != e.second) {
      hGraph.addEdge(e.first, e.second);
      degree[e.first]++;
      degree[e.second]++;
    }
  }

  // Min-fill ordering gives smaller widths, but its cost per elimination
  // step grows quadratically with the vertex degree; on high-degree graphs
  // fall back to the near-linear min-degree ordering.
  unsigned maxDegree = *std::max_element(degree.begin(), degree.end());
  bool useMinDegree = maxDegree > m_maxDegreeForMinFill;
  if (useMinDegree)
    instance->orderingAlgorithmFactory().setConstructionTemplate(
        new htd::MinDegreeOrderingAlgorithm(instance.get()));

  htd::BucketEliminationTreeDecompositionAlgorithm algorithm(instance.get());

  // Restart the (randomized) algorithm while the budget allows it and the
  // width recently improved; keep the decomposition with the smallest width.
  htd::ITreeDecomposition* best = nullptr;
  unsigned nbIterationWithoutImprovement = 0;
  while (nbIterationWithoutImprovement < 5) {
    htd::ITreeDecomposition* decomposition =
        algorithm.computeDecomposition(hGraph);
    if (!decomposition) break;

    if (!best || decomposition->maximumBagSize() < best->maximumBagSize()) {
      delete best;
      best = decomposition;
      nbIterationWithoutImprovement = 0;
    } else {
      delete decomposition;
      nbIterationWithoutImprovement++;
    }

    std::chrono::duration<double> elapsed =
        std::chrono::system_clock::now() - start;
    if (elapsed.count() >= (double)budget) break;
  }

  assert(best);

  // Translate the htd decomposition into d4's representation.
  TreeDecomp* ret =
      new TreeDecomp(std::vector<Var>(best->bagContent(best->root()).begin(),
                                      best->bagContent(best->root()).end()),
                     std::vector<TreeDecomp*>());

  std::vector<std::pair<htd::vertex_t, TreeDecomp*>> stack;
  stack.push_back({best->root(), ret});

  std::vector<htd::vertex_t> children;
  while (stack.size()) {
    auto [vertex, tree] = stack.back();
    stack.pop_back();

    children.clear();
    best->copyChildrenTo(vertex, children);
    for (auto child : children) {
      TreeDecomp* son =
          new TreeDecomp(std::vector<Var>(best->bagContent(child).begin(),
                                          best->bagContent(child).end()),
                         std::vector<TreeDecomp*>());
      tree->getSons().push_back(son);
      stack.push_back({child, son});
    }
  }

  unsigned width = best->maximumBagSize();
  delete best;

  std::chrono::duration<double> elapsed =
      std::chrono::system_clock::now() - start;
  if (verbose)
    std::cout << "c [HTD] Decomposition computed, ordering("
              << (useMinDegree ? "min-degree" : "min-fill") << ") width("
              << width << ") elapsed time: " << elapsed.count() << "s"
              << std::endl;

  return ret;
}  // constructTreeDecomposition

}  // namespace d4
