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

#include "IterativePrimalCutSelector.hpp"

namespace d4 {

void IterativePrimalCutSelector::selectRecursive(
    const parser::Formula& formula, const std::vector<unsigned>& clauseIndices,
    unsigned depth, std::vector<unsigned>& result) {
  if (depth >= m_maxDepth || clauseIndices.size() < 10) return;

  std::vector<unsigned> cutClauses, part0, part1;
  computeCut(formula, clauseIndices, cutClauses, part0, part1);

  result.insert(result.end(), cutClauses.begin(), cutClauses.end());

  // If the partition was degenerate (everything ended up in cut), stop.
  if (part0.empty() || part1.empty()) return;

  selectRecursive(formula, part0, depth + 1, result);
  selectRecursive(formula, part1, depth + 1, result);
}

std::vector<unsigned> IterativePrimalCutSelector::select(
    const parser::Formula& formula) {
  std::vector<unsigned> allClauses;
  allClauses.reserve(formula.clauses.size());
  for (unsigned i = 0; i < formula.clauses.size(); i++) allClauses.push_back(i);

  std::vector<unsigned> result;
  selectRecursive(formula, allClauses, 0, result);

  m_out << "c [ITER-PRIMAL-CUT] easy clauses=" << result.size()
        << "/" << formula.clauses.size() << '\n';

  return result;
}

}  // namespace d4
