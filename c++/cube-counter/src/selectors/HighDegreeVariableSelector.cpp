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

#include "HighDegreeVariableSelector.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_set>

namespace d4 {

std::vector<unsigned> HighDegreeVariableSelector::select(
    const parser::Formula& formula) {
  const unsigned nbVar = formula.nbVar;
  const auto& clauses = formula.clauses;

  // Step 1: compute degree of each variable (1-indexed).
  std::vector<unsigned> degree(nbVar + 1, 0);
  for (const auto& cl : clauses)
    for (int l : cl) degree[std::abs(l)]++;

  // Step 2: sort variables by degree descending, take top targetRatio fraction.
  std::vector<unsigned> order(nbVar);
  std::iota(order.begin(), order.end(), 1);
  std::sort(order.begin(), order.end(),
            [&](unsigned a, unsigned b) { return degree[a] > degree[b]; });

  unsigned targetSize =
      std::max(1u, (unsigned)std::ceil(m_targetRatio * nbVar));
  targetSize = std::min(targetSize, nbVar);

  std::vector<bool> inTarget(nbVar + 1, false);
  for (unsigned i = 0; i < targetSize; i++) inTarget[order[i]] = true;

  m_out << "c [HIGH-DEGREE] target vars=" << targetSize << "/" << nbVar
        << " (ratio=" << m_targetRatio << ")\n";

  // Step 3: F_easy_0 — clauses entirely within V_target.
  std::vector<bool> selected(clauses.size(), false);
  std::vector<bool> inVeasy(nbVar + 1, false);

  for (unsigned i = 0; i < clauses.size(); i++) {
    bool fits = true;
    for (int l : clauses[i])
      if (!inTarget[std::abs(l)]) { fits = false; break; }
    if (fits) {
      selected[i] = true;
      for (int l : clauses[i]) inVeasy[std::abs(l)] = true;
    }
  }

  // Step 4: extension — add clauses that don't grow V_easy.
  for (unsigned i = 0; i < clauses.size(); i++) {
    if (selected[i]) continue;
    bool fits = true;
    for (int l : clauses[i])
      if (!inVeasy[std::abs(l)]) { fits = false; break; }
    if (fits) selected[i] = true;
  }

  std::vector<unsigned> result;
  for (unsigned i = 0; i < clauses.size(); i++)
    if (selected[i]) result.push_back(i);

  unsigned vEasyCount = 0;
  for (unsigned v = 1; v <= nbVar; v++) if (inVeasy[v]) vEasyCount++;

  m_out << "c [HIGH-DEGREE] F_easy clauses=" << result.size() << "/"
        << clauses.size() << " vars=" << vEasyCount << "/" << nbVar << "\n";

  return result;
}

}  // namespace d4
