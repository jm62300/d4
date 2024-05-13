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

#include "PartialOrderHeuristicTreeDecompCnf.hpp"

#include <ostream>

#include "src/treeDecomposition/TreeDecomposition.hpp"

namespace d4 {

/**
 * @brief PartialOrderHeuristicTreeDecompCnf::PartialOrderHeuristicTreeDecompCnf
 * implementation.
 */
PartialOrderHeuristicTreeDecompCnf::PartialOrderHeuristicTreeDecompCnf(
    const OptionPartialOrderHeuristic &options, SpecManagerCnf &om,
    WrapperSolver &s, std::ostream &out) {
  TreeDecomposition *decomp =
      TreeDecomposition::makeTreeDecomposition(options, PB_CNF, out);

  TreeDecomp *tree = decomp->computeDecomposition(om);
  assert(tree);
  std::cout << "c [PARTIAL ORDER TREE DECOMP] Decomposition computed\n";

  // construct the topological order.
  std::vector<TreeDecomp *> stack;
  stack.push_back(tree);

  m_topologicalOrder.resize(om.getNbVariable() + 1, 0);
  for (auto &v : m_topologicalOrder) v = 0;

  unsigned level = 1;
  while (stack.size()) {
    std::vector<TreeDecomp *> saveStack = stack;
    stack.clear();

    for (auto *tree : saveStack) {
      for (auto &v : tree->getNode()) {
        assert(v < m_topologicalOrder.size());
        if (!m_topologicalOrder[v]) m_topologicalOrder[v] = level;
      }

      for (auto *t : tree->getSons()) stack.push_back(t);
    }

    level++;
  }

  out << "c [TREE DECOMPOSITION] Number of levels: " << level - 1 << '\n';

  delete tree;
  delete decomp;
}  // constructor

/**
 * @brief Destructor.
 */
PartialOrderHeuristicTreeDecompCnf::~PartialOrderHeuristicTreeDecompCnf() {
}  // destructor

}  // namespace d4
