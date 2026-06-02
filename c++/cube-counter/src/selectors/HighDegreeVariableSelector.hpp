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
 * @brief Iterative flood-fill selection of F_easy.
 *
 * Repeatedly picks the variable with maximum occurrence in the original formula
 * (among those not yet in V_easy) as a seed, then flood-fills through the
 * variable-clause bipartite graph: adding a variable pulls in all its clauses,
 * and adding a clause pulls in all its variables. The result is the connected
 * component of the seed in that graph. Stops before starting a new component
 * once |V_easy| >= targetRatio * nbVar.
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
