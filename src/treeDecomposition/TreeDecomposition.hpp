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

#include "src/formulaManager/FormulaManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

class OptionPartialOrderHeuristic;

enum TreeDecompositionMethod { TREE_DECOMP_PARTITION, TREE_DECOMP_TREE_WIDTH };

class TreeDecomp {
 private:
  std::vector<Var> m_node;
  std::vector<TreeDecomp *> m_sons;

 public:
  /**
   * @brief Construct a new Tree Decomp object
   */
  TreeDecomp();

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
   * @brief Display the tree.
   *
   * @param shift is the depth.
   */
  inline void displayTree(unsigned shift) {
    printf("%3u ", shift);
    for (unsigned i = 0; i < shift; i++) std::cout << "| ";
    for (auto &v : m_node) std::cout << v << " ";
    std::cout << '\n';
    for (auto *tree : m_sons) tree->displayTree(shift + 1);
  }  // display

  /**
   * @brief Get the variables of the tree.
   *
   * @param[out] vars are the collected variables.
   */
  void getAllVars(std::vector<Var> &vars) {
    std::vector<bool> marked;
    for (auto &v : vars) {
      if (marked.size() <= (unsigned)v) marked.resize(v + 1, false);
      marked[v] = true;
    }

    for (auto *tree : m_sons) {
      std::vector<Var> tmp;
      tree->getAllVars(tmp);
      for (auto &v : tmp) {
        if (marked.size() <= (unsigned)v) marked.resize(v + 1, false);
        if (!marked[v]) vars.push_back(v);
        marked[v] = true;
      }
    }

    for (auto &v : m_node)
      if ((unsigned)v >= marked.size() || !marked[v]) vars.push_back(v);
  }  // getAllVars

  /**
   * @brief Get the sons.
   *
   * @return the list of children.
   */
  std::vector<TreeDecomp *> &getSons();

  /**
   * @brief Get the with of the tree decompostion.
   *
   * @return the size of the largest bag.
   */
  unsigned getSizeLargestBag();
};

class TreeDecompositionMethodManager {
 public:
  static std::string getTreeDecompositionMethod(
      const TreeDecompositionMethod &m) {
    if (m == TREE_DECOMP_PARTITION) return "tree-partition";
    if (m == TREE_DECOMP_TREE_WIDTH) return "tree-width";

    throw(FactoryException("Paritioning method type unknown", __FILE__,
                           __LINE__));
  }  // getTreeDecompositionMethod

  static TreeDecompositionMethod getTreeDecompositionMethod(
      const std::string &m) {
    if (m == "tree-partition") return TREE_DECOMP_PARTITION;
    if (m == "tree-width") return TREE_DECOMP_TREE_WIDTH;

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
      const OptionPartialOrderHeuristic &options,
      const ProblemInputType &inType, std::ostream &out);

  /**
   * @brief Compute a decomposition.
   *
   * @param[in] om gives information about the formula.
   *
   * @return is the computed tree decomposition.
   */
  virtual TreeDecomp *computeDecomposition(FormulaManager &om) = 0;
};
}  // namespace d4