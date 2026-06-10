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

#include <map>
#include <string>
#include <vector>

#include "src/exceptions/FactoryException.hpp"
#include "src/formulaManager/FormulaManager.hpp"
#include "src/formulaManager/cnf/CnfManager.hpp"
#include "src/options/EnumMetadata.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

class OptionPartialOrderHeuristic;

enum TreeDecompositionMethod { TREE_DECOMP_PARTITION, TREE_DECOMP_TREE_WIDTH };

template <>
struct EnumMetadata<TreeDecompositionMethod> {
  static std::string name() { return "TreeDecompositionMethod"; }
  static std::map<int, std::string> mapping() {
    return {{TREE_DECOMP_PARTITION, "tree-partition"},
            {TREE_DECOMP_TREE_WIDTH, "tree-width"}};
  }
};

class TreeDecomp {
 private:
  std::vector<Var> m_node;
  std::vector<TreeDecomp*> m_sons;

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
  TreeDecomp(const std::vector<Var>& node,
             const std::vector<TreeDecomp*>& sons);

  /**
   * @brief Destroy the Tree Decomp object
   */
  ~TreeDecomp();

  /**
   * @brief Get the node.
   *
   * @return the variable list.
   */
  std::vector<Var>& getNode();

  /**
   * @brief Display the tree.
   *
   * @param shift is the depth.
   */
  inline void displayTree(unsigned shift) {
    printf("%3u ", shift);
    for (unsigned i = 0; i < shift; i++) std::cout << "| ";
    for (auto& v : m_node) std::cout << v << " ";
    std::cout << '\n';
    for (auto* tree : m_sons) tree->displayTree(shift + 1);
  }  // display

  /**
   * @brief Get the variables of the tree.
   *
   * @param[out] vars are the collected variables.
   */
  void getAllVars(std::vector<Var>& vars) {
    std::vector<bool> marked;
    for (auto& v : vars) {
      if (marked.size() <= (unsigned)v) marked.resize(v + 1, false);
      marked[v] = true;
    }

    for (auto* tree : m_sons) {
      std::vector<Var> tmp;
      tree->getAllVars(tmp);
      for (auto& v : tmp) {
        if (marked.size() <= (unsigned)v) marked.resize(v + 1, false);
        if (!marked[v]) vars.push_back(v);
        marked[v] = true;
      }
    }

    for (auto& v : m_node)
      if ((unsigned)v >= marked.size() || !marked[v]) vars.push_back(v);
  }  // getAllVars

  /**
   * @brief Get the sons.
   *
   * @return the list of children.
   */
  std::vector<TreeDecomp*>& getSons();

  /**
   * @brief Get the with of the tree decompostion.
   *
   * @return the size of the largest bag.
   */
  unsigned getSizeLargestBag();
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
  static TreeDecomposition* makeTreeDecomposition(
      const OptionPartialOrderHeuristic& options,
      const ProblemInputType& inType, std::ostream& out);

  /**
   * @brief Compute a decomposition.
   *
   * @param[in] om gives information about the formula.
   *
   * @return is the computed tree decomposition.
   */
  virtual TreeDecomp* computeDecomposition(FormulaManager& om) = 0;

  /**
   * @brief Recursively verifies the tree decomposition by projecting the CNF
   * formula down the tree, exactly as a DPLL solver would.
   *
   * @param node The current node (bag) of the tree decomposition.
   * @param currentFormula The projected formula active at this node.
   * @param nbVars Total number of variables in the problem.
   * @return true if the formula splits perfectly, false if an error is found.
   */
  bool verifyCnfProjectionUserWay(
      TreeDecomp* node, const std::vector<std::vector<Lit>>& currentFormula,
      unsigned nbVars) {
    // Base case: If we reached a leaf, the remaining formula should be fully
    // covered here.
    if (!node || node->getSons().empty()) return true;

    unsigned nbChildren = node->getSons().size();

    // 1. Identify variables in the current root (the separator)
    std::vector<bool> inRoot(nbVars + 1, false);
    for (Var v : node->getNode()) {
      assert(v <= nbVars);
      inRoot[v] = true;
    }

    // 2. Map every remaining variable to its specific child subtree
    std::vector<int> varToChild(nbVars + 1, -1);
    for (unsigned i = 0; i < nbChildren; i++) {
      std::vector<Var> subtreeVars;
      node->getSons()[i]->getAllVars(subtreeVars);

      for (Var v : subtreeVars) {
        assert(v <= nbVars);
        if (!inRoot[v]) {
          varToChild[v] = i;
        }
      }
    }

    // 3. Create the CNF-1, CNF-2, etc.
    std::vector<std::vector<std::vector<Lit>>> childFormulas(nbChildren);

    // 4. Consider clauses one by one
    for (const std::vector<Lit>& clause : currentFormula) {
      int targetChild = -1;
      std::vector<Lit> projectedClause;  // The clause minus the root variables

      for (Lit l : clause) {
        Var v = l.var();
        assert(v <= nbVars);

        // If the variable is in the root, it is handled. We drop it from the
        // projected clause.
        if (inRoot[v]) continue;

        int childIdx = varToChild[v];
        if (childIdx != -1) {
          if (targetChild == -1) {
            // Assign the clause to this child
            targetChild = childIdx;
          } else if (targetChild != childIdx) {
            // FATAL ERROR: The clause contains variables from multiple
            // subtrees!
            std::cerr
                << "\n[Decomposition Error] Clause spans across subtrees!\n"
                << "Even after removing root variables, the clause contains "
                   "literals "
                << "belonging to Child " << targetChild << " AND Child "
                << childIdx << ".\n"
                << "The root bag failed to separate them!\n";
            return false;
          }

          // Add the literal to the projected clause for the child
          projectedClause.push_back(l);
        }
      }

      // 5. Add the properly projected clause to the correct child's formula
      // If targetChild is -1, it means ALL variables in the clause were in the
      // root bag, which is perfectly fine (the clause is fully
      // satisfied/covered at this level).
      if (targetChild != -1) {
        childFormulas[targetChild].push_back(projectedClause);
      }
    }

    // 6. Recursively call the function with one formula and then with the other
    for (unsigned i = 0; i < nbChildren; i++) {
      if (!verifyCnfProjectionUserWay(node->getSons()[i], childFormulas[i],
                                      nbVars)) {
        return false;
      }
    }

    return true;
  }

  bool checkMyTree(CnfManager& cnf, TreeDecomp* root) {
    std::vector<std::vector<Lit>> fullFormula;

    // Copy all valid clauses into our initial formula state
    for (unsigned i = 0; i < cnf.getNbClause(); i++) {
      std::span<Lit> cl = cnf.getClause(i);
      fullFormula.emplace_back(cl.begin(), cl.end());
    }

    return verifyCnfProjectionUserWay(root, fullFormula, cnf.getNbVariable());
  }
};
}  // namespace d4