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

#include "cnf/TreeDecompositionCnfPartition.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/options/branchingHeuristic/OptionPartitioningHeuristic.hpp"

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
    const OptionPartitioningHeuristic &options, const ProblemInputType &inType,
    std::ostream &out) {
  switch (inType) {
    case PB_CNF:
      switch (options.treeDecompositionMethod) {
        case TREE_DECOMP_PARTITION:
          return new TreeDecompositionCnfPartition(
              options.partitionerName, options.hyperGraphExtractorMethod);
        default:
          break;
      }
    default:
      break;
  }

  throw(FactoryException("Cannot create a TreeDecomposition", __FILE__,
                         __LINE__));
}  // makeTreeDecomposition

}  // namespace d4