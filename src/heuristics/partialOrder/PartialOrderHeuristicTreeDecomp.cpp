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

#include "PartialOrderHeuristicTreeDecomp.hpp"

#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
 * @brief PartialOrderHeuristicTreeDecomp::PartialOrderHeuristicTreeDecomp
 * implementation.
 */
PartialOrderHeuristicTreeDecomp::PartialOrderHeuristicTreeDecomp(
    const OptionPartialOrderHeuristic &options, FormulaManager &om,
    std::ostream &out) {
  TreeDecomposition *decomp = TreeDecomposition::makeTreeDecomposition(
      options, om.getProblemInputType(), out);

  TreeDecomp *tree = decomp->computeDecomposition(om);
  assert(tree);
  std::cout << "c [PARTIAL ORDER TREE DECOMP] Decomposition computed size("
            << tree->getSizeLargestBag() << ") first size("
            << tree->getNode().size() << ")\n";

  // construct the topological order.
  std::vector<TreeDecomp *> stack;
  stack.push_back(tree);

  m_topologicalOrder.resize(om.getNbVariable() + 1, 0);
  for (auto &v : m_topologicalOrder) v = 0;

  unsigned level = 1, largestBag = 0;
  while (stack.size()) {
    std::vector<TreeDecomp *> saveStack = stack;
    stack.clear();

    for (auto *tree : saveStack) {
      if (tree->getNode().size() > largestBag)
        largestBag = tree->getNode().size();
      for (auto &v : tree->getNode()) {
        assert(v < m_topologicalOrder.size());
        if (!m_topologicalOrder[v]) m_topologicalOrder[v] = level;
      }

      for (auto *t : tree->getSons()) stack.push_back(t);
    }

    level++;
  }
  m_treeWidth = largestBag;

  if (options.scaleFactor == 0) {
    double rt = (double)om.getNbVariable() / (double)m_treeWidth;
    out << "c [TREE DECOMPOSITION] Ratio #var/tree-width: " << rt << '\n';
    if (rt > 40)
      m_scaleFactor = 1e7;
    else
      m_scaleFactor = 100.0 * exp(rt) / (double)om.getNbVariable();
  } else
    m_scaleFactor = options.scaleFactor;

  for (auto &w : m_topologicalOrder) {
    w = m_scaleFactor * ((double)(level - w + 1)) / (double)level;
  }

  out << "c [TREE DECOMPOSITION] Number of levels: " << level - 1 << '\n';
  out << "c [TREE DECOMPOSITION] Scaling factor: " << m_scaleFactor << '\n';
}  // constructor

/**
 * @brief Destructor.
 */
PartialOrderHeuristicTreeDecomp::~PartialOrderHeuristicTreeDecomp() {
}  // destructor

/**
 * @brief PartialOrderHeuristicTreeDecomp::computeCutSet implementation.
 */
void PartialOrderHeuristicTreeDecomp::computeCutSet(std::vector<Var> &component,
                                                    std::vector<Var> &cutSet) {
  if (!component.size()) return;

  unsigned min = m_topologicalOrder[component[0]];
  for (auto &v : component)
    if (m_topologicalOrder[v] < min) min = m_topologicalOrder[v];

  for (auto &v : component)
    if (m_topologicalOrder[v] == min) cutSet.push_back(v);
}  // computeCutSet

}  // namespace d4