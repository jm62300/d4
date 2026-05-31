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
 * @brief Selects F_easy by targeting the highest-degree variables.
 *
 * 1. Rank variables by degree (number of clause occurrences), descending.
 * 2. Take the top `targetRatio` fraction as V_target.
 * 3. F_easy_0 = clauses whose variable support is entirely within V_target.
 * 4. V_easy   = variables that actually appear in F_easy_0.
 * 5. Extend   = add any remaining clause whose variables are all in V_easy
 *               (does not grow V_easy, so one pass suffices).
 */
class HighDegreeVariableSelector : public ClauseSelector {
  double m_targetRatio;
  std::ostream& m_out;

 public:
  explicit HighDegreeVariableSelector(double targetRatio = 0.10,
                                      std::ostream& out = std::cout)
      : m_targetRatio(targetRatio), m_out(out) {}

  std::vector<unsigned> select(const parser::Formula& formula) override;
};

}  // namespace d4
