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

#include "TreeDecomposition.hpp"

#include "TreeDecompositionPartition.hpp"
#include "TreeDecompositionTreeWidth.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"
#include "src/options/treeDecomposition/OptionTreeDecompositionBiPartition.hpp"
#include "src/options/treeDecomposition/OptionTreeDecompositionPace.hpp"

namespace d4 {

/**
 * @brief TreeDecomp::TreeDecomp implementation.
 */
TreeDecomp::TreeDecomp(const std::vector<Var> &node,
                       const std::vector<TreeDecomp *> &sons)
    : m_node(node), m_sons(sons) {}  // constructor

/**
 * @brief TreeDecomp::TreeDecomp implementation.
 */
TreeDecomp::TreeDecomp() {}  // constructor

/**
 * @brief TreeDecomp::~TreeDecomp implementation.
 */
TreeDecomp::~TreeDecomp() {
  for (auto &t : m_sons) delete t;
}  // destructor

/**
 * @brief TreeDecomp::getSizeLargestBag implementation.
 */
unsigned TreeDecomp::getSizeLargestBag() {
  unsigned sizeChildren = 0;
  for (auto &c : m_sons) {
    unsigned tmp = c->getSizeLargestBag();
    if (tmp > sizeChildren) sizeChildren = tmp;
  }

  if (m_node.size() > sizeChildren) return m_node.size();
  return sizeChildren;
}  // getSizeLargestBag

/**
 * @brief TreeDecomp::getNode implementation.
 */
std::vector<Var> &TreeDecomp::getNode() { return m_node; }  // getNode

/**
 * @brief TreeDecomp::getSons implementation.
 */
std::vector<TreeDecomp *> &TreeDecomp::getSons() { return m_sons; }  // getSons

/**
 * @brief TreeDecomposition::makeTreeDecomposition implementation.
 */

TreeDecomposition *TreeDecomposition::makeTreeDecomposition(
    const OptionPartialOrderHeuristic &options, const ProblemInputType &inType,
    std::ostream &out) {
  return options.optionTreeDecomposition->createTreeDecomposition(inType);
}  // makeTreeDecomposition

}  // namespace d4