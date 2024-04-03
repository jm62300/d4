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
#pragma once

#include <vector>

#include "src/problem/ProblemTypes.hpp"
#include "src/specs/SpecManager.hpp"

namespace d4 {

class OptionPartitioningHeuristic;

enum TreeDecompositionMethod { TREE_DECOMP_PARTITION };

class TreeDecompositionMethodManager {
 public:
  static std::string getTreeDecompositionMethod(
      const TreeDecompositionMethod &m) {
    if (m == TREE_DECOMP_PARTITION) return "tree decomposition partition";

    throw(FactoryException("Paritioning method type unknown", __FILE__,
                           __LINE__));
  }  // getTreeDecompositionMethod

  static TreeDecompositionMethod getTreeDecompositionMethod(
      const std::string &m) {
    if (m == "partition") return TREE_DECOMP_PARTITION;

    throw(FactoryException("Tree Decomposition method unknown", __FILE__,
                           __LINE__));
  }  // getTreeDecompositionMethod
};

class TreeDecomposition {
 public:
  /**
   * @brief Destroy the Tree Decomposition object.
   *
   */
  virtual ~TreeDecomposition() {}

  /**
   * @brief Factory for constructing a Tree Decomposition.
   *
   * @param[in] out is the stream where are printed out the information.
   * @param[in] inType gives information about the type of formula under
   * consideration.
   * @return a tree decomposition manager.
   */
  static TreeDecomposition *makeTreeDecomposition(
      const OptionPartitioningHeuristic &options,
      const ProblemInputType &inType, std::ostream &out);

  /**
   * @brief Compute a decomposition.
   *
   * @param[out] decomposition is the computed decomposition.
   * @param[in] om gives information about the formula.
   */
  virtual void computeDecomposition(
      std::vector<std::vector<Var>> &decomposition, SpecManager &om) = 0;

  /**
   * @brief This function check if a given decomposition (given as a partial
   * order) forms really a tree decomposition of the problem.
   *
   * @param[in] decomposition is the decomposition we want to test.
   * @param[in] om gives information about the formula under consideration.
   * @param[in] component is the set of variables we are focusing on.
   */
  virtual void checkDecomposition(
      const std::vector<std::vector<Var>> &decomposition, SpecManager &om,
      const std::vector<Var> &component) = 0;
};
}  // namespace d4