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

#include "TreeDecompositionCnfTreeWidth.hpp"

#include "3rdParty/flowCutter/src/pace.h"
#include "src/representation/graph/Graph.hpp"

namespace d4 {
/**
 * @brief TreeDecompositionTreeWidth::computeDecomposition implementation.
 */
TreeDecomp *TreeDecompositionTreeWidth::computeDecomposition(SpecManager &om) {
  GraphExtractor *graphExtractor = GraphExtractor::makeGraphExtractor(
      m_graphExtractorMethod, m_simplification, PB_CNF);

  std::vector<Var> component, notLinked;
  for (unsigned i = 1; i <= om.getNbVariable(); i++) {
    if (!om.getNbOccurrence(i))
      notLinked.push_back(i);
    else
      component.push_back(i);
  }
  Graph graph(component.size());
  graphExtractor->constructGraph(om, component, graph);

  // const char *decomp = flowCutter::paceMain(graph.getNbNode(),
  // graph.getEdge()); std::cout << "print the decomposition:\n" << decomp;

  assert(0);
  return NULL;
}  // computeDecomposition

}  // namespace d4