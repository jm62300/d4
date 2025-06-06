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

#include <cstdint>
#include <ostream>
#include <vector>

#include "PartialOrderHeuristic.hpp"
#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {

class PartialOrderHeuristicGiven : public PartialOrderHeuristic {
 protected:
  std::vector<unsigned> m_order;
  double m_scaleFactor;

 public:
  /**
   * @brief Constructor.
   *
   * @param options gives the list of options.
   * @param om is the object which deal with the formula.
   * @param s is a SAT solver associate with the formula under consideration.
   * @param out is the output stream.
   */
  PartialOrderHeuristicGiven(const OptionPartialOrderHeuristic &options,
                             FormulaManager &om, std::ostream &out);

  ~PartialOrderHeuristicGiven();

  void computeCutSet(std::vector<Var> &component,
                     std::vector<Var> &cutSet) override;

  /**
   * @brief Get the partial order regarding the topological order.
   *
   * @param v is a variable.
   *
   * @return the position of v in the order.
   */
  inline unsigned getPartialOrder(Var v) override {
    return m_order[v];
  }  // getPartialOrder

  /**
   * @brief The scale factor depends the size of the cutset.
   *
   * @return the scale factor variable.
   */
  inline double scaleFactor() override { return m_scaleFactor; }

  /**
   * @brief This function is called in order to init (or reinit) the partial
   * order regarding the current information. More precisely, the partial order
   * and the scaling factor are set regarding the given options.
   *
   * @param options is the list of options used.
   * @param sm is the formula we want to compute the partial order.
   * @param out is the stream where are printed out the logs.
   */
  void init(const OptionPartialOrderHeuristic &options, FormulaManager &sm,
            std::ostream &out) override;
};
}  // namespace d4
