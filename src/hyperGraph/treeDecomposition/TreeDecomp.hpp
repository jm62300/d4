
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

namespace d4 {
class TreeDecomp {
 private:
  std::vector<Var> m_node;
  std::vector<TreeDecomp *> m_sons;

 public:
  /**
   * @brief Construct a new Tree Decomp object/
   *
   * @param node is the variables in the current node.
   * @param sons is a list of trees.
   */
  TreeDecomp(const std::vector<Var> &node,
             const std::vector<TreeDecomp *> &sons);

  /**
   * @brief Destroy the Tree Decomp object
   */
  ~TreeDecomp();

  /**
   * @brief Get the node.
   *
   * @return the variable list.
   */
  std::vector<Var> &getNode();

  /**
   * @brief Get the sons.
   *
   * @return the list of children.
   */
  std::vector<TreeDecomp *> &getSons();
};
}  // namespace d4