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

#include "src/representation/graph/Graph.hpp"

namespace d4 {
/**
 * @brief TreeDecompositionCnfTreeWidth::computeDecomposition implementation.
 */
TreeDecomp *TreeDecompositionCnfTreeWidth::computeDecomposition(
    SpecManager &om) {
  std::vector<Var> component, notLinked;
  for (unsigned i = 1; i <= om.getNbVariable(); i++) {
    if (!om.getNbOccurrence(i))
      notLinked.push_back(i);
    else
      component.push_back(i);
  }
  Graph graph;
  m_graphExtractor->constructGraph(om, component, graph);

  TreeDecomp *treeDecomp =
      m_treeDecompositioner->constructTreeDecomposition(graph);

  return treeDecomp;
}  // computeDecomposition

/**
 * @brief TreeDecompositionCnfTreeWidth::TreeDecompositionCnfTreeWidth
 * implementation.
 */
TreeDecompositionCnfTreeWidth::TreeDecompositionCnfTreeWidth(
    const TreeDecompositionerMethod &treeDecompositionerMethod,
    const GraphExtractorMethod &graphExtractorMethod, bool simplification) {
  m_graphExtractor = GraphExtractor::makeGraphExtractor(graphExtractorMethod,
                                                        simplification, PB_CNF);
  m_treeDecompositioner = TreeDecompositioner::makeTreeDecompositionMethod(
      treeDecompositionerMethod);
}  // constructor.

/**
 * @brief TreeDecompositionCnfTreeWidth::~TreeDecompositionCnfTreeWidth
 * implementation.
 */
TreeDecompositionCnfTreeWidth::~TreeDecompositionCnfTreeWidth() {
  if (m_treeDecompositioner) delete m_treeDecompositioner;
  if (m_graphExtractor) delete m_graphExtractor;
}  // destructor.

}  // namespace d4