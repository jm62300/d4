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

  unsigned budget =
      std::max(1u, (unsigned)std::ceil(m_targetRatio * nbVar));
  budget = std::min(budget, nbVar);

  // Build clause lists per variable.
  std::vector<std::vector<unsigned>> varClauses(nbVar + 1);
  for (unsigned i = 0; i < numClauses; i++)
    for (int l : clauses[i]) varClauses[std::abs(l)].push_back(i);

  // degree[v] = number of clauses containing v (used as tiebreaker).
  std::vector<unsigned> degree(nbVar + 1, 0);
  for (unsigned v = 1; v <= nbVar; v++) degree[v] = varClauses[v].size();

  // remaining[c] = number of variables in clause c not yet in V_easy.
  std::vector<unsigned> remaining(numClauses);
  for (unsigned i = 0; i < numClauses; i++)
    remaining[i] = (unsigned)clauses[i].size();

  // gain[v] = number of clauses that become covered the moment v is added,
  //           i.e. clauses where v is currently the sole missing variable.
  std::vector<unsigned> gain(nbVar + 1, 0);

  // Initialise gain for unit clauses.
  for (unsigned i = 0; i < numClauses; i++) {
    if (remaining[i] == 1) gain[std::abs(clauses[i][0])]++;
  }

  std::vector<bool> inVeasy(nbVar + 1, false);
  std::vector<bool> covered(numClauses, false);

  for (unsigned step = 0; step < budget; step++) {
    // Pick the variable maximising gain; break ties by degree.
    unsigned best = 0;
    for (unsigned v = 1; v <= nbVar; v++) {
      if (inVeasy[v]) continue;
      if (best == 0 || gain[v] > gain[best] ||
          (gain[v] == gain[best] && degree[v] > degree[best]))
        best = v;
    }
    if (best == 0) break;

    inVeasy[best] = true;

    // Update all clauses that contain 'best'.
    for (unsigned c : varClauses[best]) {
      if (covered[c]) continue;
      remaining[c]--;
      if (remaining[c] == 0) {
        covered[c] = true;
      } else if (remaining[c] == 1) {
        // Find the single variable still missing.
        for (int l : clauses[c]) {
          unsigned u = (unsigned)std::abs(l);
          if (!inVeasy[u]) {
            gain[u]++;
            break;
          }
        }
      }
    }
  }

  std::vector<unsigned> result;
  for (unsigned i = 0; i < numClauses; i++)
    if (covered[i]) result.push_back(i);

  unsigned vEasyCount = 0;
  for (unsigned v = 1; v <= nbVar; v++) if (inVeasy[v]) vEasyCount++;

  m_out << "c [HIGH-DEGREE] budget=" << budget << "/" << nbVar
        << " actual vars=" << vEasyCount
        << " clauses=" << result.size() << "/" << numClauses << "\n";

  return result;
}

}  // namespace d4
