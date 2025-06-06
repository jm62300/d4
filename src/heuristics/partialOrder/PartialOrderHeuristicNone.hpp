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
#include <src/problem/ProblemTypes.hpp>
#include <vector>

#include "PartialOrderHeuristic.hpp"

namespace d4 {
class PartialOrderHeuristicNone : public PartialOrderHeuristic {
 public:
  PartialOrderHeuristicNone() {}

  void computeCutSet(std::vector<Var> &component, std::vector<Var> &cutSet);

  /**
   * @brief This function do nothing.
   *
   * @param options is the list of options used.
   * @param sm is the formula we want to compute the partial order.
   * @param out is the stream where are printed out the logs.
   */
  inline void init(const OptionPartialOrderHeuristic &options,
                   FormulaManager &sm, std::ostream &out) override {}
};
}  // namespace d4
