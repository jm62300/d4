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
  init(options, om, out);
  out << "c [TREE DECOMPOSITION] Tree Width: " << m_treeWidth << '\n';
  out << "c [TREE DECOMPOSITION] Number of levels: " << m_level << '\n';
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

/**
 * @brief PartialOrderHeuristicTreeDecomp::init implementation.
 */
void PartialOrderHeuristicTreeDecomp::init(
    const OptionPartialOrderHeuristic &options, FormulaManager &om,
    std::ostream &out) {
  TreeDecomposition *decomp = TreeDecomposition::makeTreeDecomposition(
      options, om.getProblemInputType(), out);

  TreeDecomp *tree = decomp->computeDecomposition(om);
  assert(tree);

  // construct the topological order.
  std::vector<TreeDecomp *> stack;
  stack.push_back(tree);

  m_topologicalOrder.resize(om.getNbVariable() + 1, 0);
  for (auto &v : m_topologicalOrder) v = 0;

  m_level = 1, m_treeWidth = 0;
  while (stack.size()) {
    std::vector<TreeDecomp *> saveStack = stack;
    stack.clear();

    for (auto *tree : saveStack) {
      if (tree->getNode().size() > m_treeWidth)
        m_treeWidth = tree->getNode().size();
      for (auto &v : tree->getNode()) {
        assert(v < m_topologicalOrder.size());
        if (!m_topologicalOrder[v]) m_topologicalOrder[v] = m_level;
      }

      for (auto *t : tree->getSons()) stack.push_back(t);
    }

    m_level++;
  }

  if (m_treeWidth < 30)
    m_scaleFactor = 100000000;
  else if (m_treeWidth < 40)
    m_scaleFactor = 100000;
  else if (m_treeWidth < 50)
    m_scaleFactor = 1000;
  else if (m_treeWidth < 70)
    m_scaleFactor = 0;
  else
    m_scaleFactor = 0;

  delete tree;
  delete decomp;
}  // init

}  // namespace d4