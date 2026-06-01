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

#include <iostream>
#include <vector>

#include "../ClauseSelector.hpp"

namespace d4 {

/**
 * @brief Cube-aware greedy selection of F_easy.
 *
 * The cost of cube-and-count is roughly (#cubes) x (cost per conditioned
 * count), and #cubes equals the number of partial models of F_easy. Adding a
 * variable to V_easy adds one free dimension (it ~doubles the cube count)
 * unless it also completes clauses that constrain it. So this selector spends a
 * budget slot only on a variable that actually closes at least one clause, and
 * prefers closing SHORT clauses (which remove the largest fraction of the model
 * space). A small seeding phase assembles the first clause; afterwards any
 * variable that would complete nothing is rejected and the search stops, rather
 * than inflating the cube count by filling the whole `targetRatio` budget.
 *
 * A clause is "covered" once all its variables are in V_easy; covered clauses
 * form F_easy. (The caller's --extendEasy pass then absorbs any further clause
 * already wholly inside V_easy.)
 */
class HighDegreeVariableSelector : public ClauseSelector {
  double m_targetRatio;
  double m_minBits;
  std::ostream& m_out;

 public:
  explicit HighDegreeVariableSelector(double targetRatio = 0.10,
                                      double minBits = 1.0,
                                      std::ostream& out = std::cout)
      : m_targetRatio(targetRatio), m_minBits(minBits), m_out(out) {}

  std::vector<unsigned> select(const parser::Formula& formula) override;
};

}  // namespace d4
