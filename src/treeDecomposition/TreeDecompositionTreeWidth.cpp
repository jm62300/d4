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

#include "TreeDecompositionTreeWidth.hpp"

#include "src/representation/graph/Graph.hpp"

namespace d4 {
/**
 * @brief TreeDecompositionTreeWidth::computeDecomposition implementation.
 */
TreeDecomp *TreeDecompositionTreeWidth::computeDecomposition(
    FormulaManager &om) {
  std::vector<Var> component, notLinked;
  for (unsigned i = 1; i <= om.getNbVariable(); i++) {
    if (om.isFreeVariable(i))
      notLinked.push_back(i);
    else
      component.push_back(i);
  }

  Graph graph;
  m_graphExtractor->constructGraph(om, component, graph);

  TreeDecomp *treeDecomp = m_treeDecompositioner->constructTreeDecomposition(
      graph, m_budget, m_seed, m_verbose);

  return treeDecomp;
}  // computeDecomposition

/**
 * @brief TreeDecompositionTreeWidth::TreeDecompositionCnfTreeWidth
 * implementation.
 */
TreeDecompositionTreeWidth::TreeDecompositionTreeWidth(
    const TreeDecompositionerMethod &treeDecompositionerMethod,
    const GraphExtractorMethod &graphExtractorMethod,
    const ProblemInputType pbType, bool simplification, unsigned budget,
    unsigned seed, bool verbose) {
  m_graphExtractor = GraphExtractor::makeGraphExtractor(graphExtractorMethod,
                                                        simplification, pbType);
  m_treeDecompositioner = TreeDecompositioner::makeTreeDecompositionMethod(
      treeDecompositionerMethod);

  m_budget = budget;
  m_seed = seed;
  m_verbose = verbose;
}  // constructor.

/**
 * @brief TreeDecompositionTreeWidth::~TreeDecompositionTreeWidth
 * implementation.
 */
TreeDecompositionTreeWidth::~TreeDecompositionTreeWidth() {
  if (m_treeDecompositioner) delete m_treeDecompositioner;
  if (m_graphExtractor) delete m_graphExtractor;
}  // destructor.

}  // namespace d4