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

#include <cmath>

namespace d4 {

std::vector<unsigned> HighDegreeVariableSelector::select(
    const parser::Formula& formula) {
  const unsigned nbVar = formula.nbVar;
  const auto& clauses = formula.clauses;
  const unsigned numClauses = (unsigned)clauses.size();

  const unsigned threshold =
      (unsigned)std::ceil(m_targetRatio * (double)nbVar);

  // var -> clause index list, fixed on the original formula.
  std::vector<std::vector<unsigned>> varToClauses(nbVar + 1);
  for (unsigned i = 0; i < numClauses; i++)
    for (int l : clauses[i])
      varToClauses[(unsigned)std::abs(l)].push_back(i);

  // Occurrence count in the original formula (never modified).
  std::vector<unsigned> occ(nbVar + 1, 0);
  for (unsigned v = 1; v <= nbVar; v++) occ[v] = (unsigned)varToClauses[v].size();

  std::vector<bool> inV(nbVar + 1, false);
  unsigned vCount = 0;
  unsigned stepCount = 0;

  while (vCount < threshold) {
    // Pick v* : highest occurrence in original F, not yet in V.
    unsigned best = 0;
    for (unsigned v = 1; v <= nbVar; v++) {
      if (inV[v]) continue;
      if (best == 0 || occ[v] > occ[best]) best = v;
    }
    if (best == 0) break;
    stepCount++;

    // Grow V: add every variable that appears in any clause containing best.
    unsigned added = 0;
    for (unsigned c : varToClauses[best]) {
      for (int l : clauses[c]) {
        unsigned u = (unsigned)std::abs(l);
        if (!inV[u]) {
          inV[u] = true;
          vCount++;
          added++;
        }
      }
    }

    m_out << "c [HIGH-DEGREE] step=" << stepCount << " seed=" << best
          << " occ=" << occ[best] << " added=" << added
          << " |V|=" << vCount << "/" << nbVar << "\n";
  }

  // F_easy = all clauses c such that var(c) ⊆ V.
  std::vector<unsigned> result;
  for (unsigned i = 0; i < numClauses; i++) {
    bool allInV = true;
    for (int l : clauses[i]) {
      if (!inV[(unsigned)std::abs(l)]) { allInV = false; break; }
    }
    if (allInV) result.push_back(i);
  }

  m_out << "c [HIGH-DEGREE] targetRatio=" << m_targetRatio
        << " threshold=" << threshold << "/" << nbVar
        << " |V|=" << vCount << " clauses=" << result.size() << "/" << numClauses
        << " steps=" << stepCount << "\n";

  return result;
}

}  // namespace d4
