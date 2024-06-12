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

#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
class WrapperSolver;
class FormulaManager;
class EquivExtractor;

enum PartialOrderHeuristicMethod : char {
  PARTIAL_ORDER_TREE_DECOMPOSITION,
  PARTIAL_ORDER_NONE
};

class OptionPartialOrderHeuristic;

class PartialOrderMethodManager {
 public:
  static std::string getPartialOrderMethod(
      const PartialOrderHeuristicMethod &m) {
    if (m == PARTIAL_ORDER_TREE_DECOMPOSITION) return "tree-decomposition";
    if (m == PARTIAL_ORDER_NONE) return "none";

    throw(FactoryException("Paritioning method type unknown", __FILE__,
                           __LINE__));
  }  // getPartialOrderMethod

  static PartialOrderHeuristicMethod getPartialOrderMethod(
      const std::string &m) {
    if (m == "tree-decomposition") return PARTIAL_ORDER_TREE_DECOMPOSITION;
    if (m == "none") return PARTIAL_ORDER_NONE;

    throw(FactoryException("Paritioning method unknown", __FILE__, __LINE__));
  }  // getPartialOrderMethod
};

class PartialOrderHeuristic {
 protected:
  unsigned m_nbVar;
  void computeEquivClass(EquivExtractor &eqManager, WrapperSolver &solver,
                         std::vector<Var> &component,
                         std::vector<Lit> &unitEquiv,
                         std::vector<Var> &equivClass,
                         std::vector<std::vector<Var>> &equivVar);

 public:
  virtual ~PartialOrderHeuristic() {}
  static PartialOrderHeuristic *makePartialOrderingHeuristic(
      const OptionPartialOrderHeuristic &options, FormulaManager &sm,
      WrapperSolver &ws, std::ostream &out);

  static PartialOrderHeuristic *makePartitioningHeuristicNone(
      std::ostream &out);

  /**
   * @brief Get the partial order for a given variable.
   *
   * @param[in] v is a variable.
   *
   * @return the position of the given variable.
   */
  virtual unsigned getPartialOrder(Var v) { return 1; }

  /**
   * Compute a cutset regarding the subformula built on the set of given
   * variables.
   *
   * @param[in] component is the set of variables the problem is built on.
   * @param[out] cutSet is the computed cut set.
   */
  virtual void computeCutSet(std::vector<Var> &component,
                             std::vector<Var> &cutSet) = 0;

  /**
   * @brief Decide if the partitioning heuristic is ready to be used.
   * @param[in] component the variables of the current subformula.
   * @return true if the partitioner is ready.
   */
  virtual bool isReady(std::vector<Var> &component) { return true; }

  /**
     Print out some statistic about the way the cutting process has been
     conducted.
   */
  virtual void displayStat(std::ostream &out) {}

  /**
   * @brief It is used for scale the partial order heuristic.
   *
   * @return double is the scale factor used.
   */
  virtual double scaleFactor() { return 0; }
};
}  // namespace d4
