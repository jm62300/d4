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
 * @brief Selects F_easy as the cut hyperedges of a primal hypergraph partition.
 *
 * Builds the primal hypergraph of the formula (variables = nodes, clauses =
 * hyperedges), bisects it with PaToH, and returns the clauses that span both
 * sides of the variable partition.  These "cut clauses" sit between the two
 * components: conditioning on their partial models decouples the residual
 * formula into two independent pieces that the full counter can exploit.
 */
class PrimalCutSelector : public ClauseSelector {
 protected:
  std::ostream& m_out;

  /**
   * @brief Core one-level cut: partition the given clause subset and return
   *        cut indices, filling @p part0 and @p part1 with the two residuals.
   *
   * @param formula        The full formula (clause data lives here).
   * @param clauseIndices  Indices of clauses forming the sub-formula to cut.
   * @param cutClauses     Output: indices of cut clauses (spanning both sides).
   * @param part0          Output: clause indices entirely in partition 0.
   * @param part1          Output: clause indices entirely in partition 1.
   */
  void computeCut(const parser::Formula& formula,
                  const std::vector<unsigned>& clauseIndices,
                  std::vector<unsigned>& cutClauses,
                  std::vector<unsigned>& part0,
                  std::vector<unsigned>& part1);

 public:
  explicit PrimalCutSelector(std::ostream& out = std::cout) : m_out(out) {}

  std::vector<unsigned> select(const parser::Formula& formula) override;
};

}  // namespace d4
