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

  unsigned budget = std::max(1u, (unsigned)std::ceil(m_targetRatio * nbVar));
  budget = std::min(budget, nbVar);

  // ---------------------------------------------------------------------------
  // Objective: end-to-end runtime of cube-and-count.
  //
  //   runtime ~= (#cubes) x (cost of counting F | cube)
  //
  // and #cubes == number of partial models of F_easy. Adding a variable to
  // V_easy adds one *free* dimension to F_easy (it roughly DOUBLES the cube
  // count) unless that variable also completes clauses that constrain it back
  // down. The previous heuristic always spent the whole variable budget, adding
  // variables that completed nothing (breaking ties on raw degree); each such
  // variable doubled the cube count, which is exactly what made the method
  // useless.
  //
  // This version is cube-aware. It only spends a budget slot on a variable that
  // actually puts at least one clause into F_easy, and it prefers completing
  // SHORT clauses, because a short clause removes the largest fraction of the
  // model space (a binary clause forbids 1/4 of its assignments, a ternary 1/8,
  // ...). A tiny seeding phase is allowed to assemble the first clause; after
  // that, any variable that would complete nothing is rejected and the search
  // stops rather than inflating the cube count.
  // ---------------------------------------------------------------------------

  // Build clause lists per variable (ignoring units — they impose nothing on
  // the cube structure).
  std::vector<std::vector<unsigned>> varClauses(nbVar + 1);
  for (unsigned i = 0; i < numClauses; i++) {
    if (clauses[i].size() == 1) continue;
    for (int l : clauses[i]) varClauses[std::abs(l)].push_back(i);
  }

  // degree[v] = number of (non-unit) clauses containing v (final tiebreaker).
  std::vector<unsigned> degree(nbVar + 1, 0);
  for (unsigned v = 1; v <= nbVar; v++) degree[v] = varClauses[v].size();

  // remaining[c] = number of variables in clause c not yet in V_easy.
  std::vector<unsigned> remaining(numClauses, 0);
  unsigned maxSize = 0;
  for (unsigned i = 0; i < numClauses; i++) {
    if (clauses[i].size() == 1) continue;
    remaining[i] = (unsigned)clauses[i].size();
    maxSize = std::max(maxSize, remaining[i]);
  }

  // constraintBits[s] = -log2(1 - 2^-s): how much completing a size-s clause
  // shrinks the model count, in bits. Short clauses dominate (s=2 -> 0.415,
  // s=3 -> 0.193, s=4 -> 0.093, ...), so completing short clauses is preferred.
  std::vector<double> constraintBits(maxSize + 1, 0.0);
  for (unsigned s = 1; s <= maxSize; s++)
    constraintBits[s] = -std::log2(1.0 - std::pow(2.0, -(double)s));

  // completion[v] = sum of constraintBits over clauses that v would complete
  // right now (clauses where v is the sole missing variable). This is the
  // immediate model-count reduction from adding v.
  std::vector<double> completion(nbVar + 1, 0.0);

  // seedScore[v] = closeness potential, used only while F_easy is still empty
  // to decide which variable to invest in first. A clause needing r more
  // variables contributes 2^-r, so variables sitting in many short, nearly
  // complete clauses are favoured as seeds.
  std::vector<double> seedScore(nbVar + 1, 0.0);
  for (unsigned i = 0; i < numClauses; i++) {
    if (remaining[i] == 0) continue;
    double w = std::pow(2.0, -(double)remaining[i]);
    for (int l : clauses[i]) seedScore[std::abs(l)] += w;
  }

  std::vector<bool> inVeasy(nbVar + 1, false);
  std::vector<bool> covered(numClauses, false);

  // Bound on the seeding phase: enough slots to assemble the largest clause we
  // might need to close, but never the whole budget. If we cannot complete any
  // clause within this many free additions, the formula has no short structure
  // to exploit and we stop (empty F_easy -> a plain direct count, which is
  // correct and avoids a useless cube blow-up).
  const unsigned kMaxSeed = std::min(budget, std::max(2u, maxSize));

  // A variable adds one free dimension (+1 bit) to F_easy, so to avoid growing
  // the cube count it must complete clauses worth at least this many bits.
  // 1.0 = net non-increasing cube estimate; lower trades more clauses (better
  // decomposition) for a larger cube count, higher is stricter.
  const double kMinBits = m_minBits;

  // log2 of the running cube-count estimate: |V_easy| - sum of completed
  // clause constraint bits. Used only for reporting.
  double logCubes = 0.0;

  unsigned coveredCount = 0;
  unsigned seedSteps = 0;

  for (unsigned step = 0; step < budget; step++) {
    // Pick the best candidate. Primary key: immediate completion (short-clause
    // weighted). If nothing can be completed yet, fall back to seedScore to
    // grow toward the first completion. Degree breaks remaining ties.
    unsigned best = 0;
    bool anyCompletion = false;
    for (unsigned v = 1; v <= nbVar; v++) {
      if (inVeasy[v]) continue;
      if (completion[v] > 0.0) anyCompletion = true;
    }

    for (unsigned v = 1; v <= nbVar; v++) {
      if (inVeasy[v]) continue;
      bool better;
      if (best == 0) {
        better = true;
      } else if (anyCompletion) {
        better = completion[v] > completion[best] ||
                 (completion[v] == completion[best] && degree[v] > degree[best]);
      } else {
        better = seedScore[v] > seedScore[best] ||
                 (seedScore[v] == seedScore[best] && degree[v] > degree[best]);
      }
      if (better) best = v;
    }
    if (best == 0) break;

    if (!anyCompletion) {
      // Seeding move: this variable completes nothing yet, so it is a pure +1
      // bit. Only allowed to assemble the very first clause.
      if (coveredCount > 0) break;       // already seeded: don't add free vars
      if (seedSteps >= kMaxSeed) break;  // gave up assembling a first clause
      seedSteps++;
    } else if (completion[best] < kMinBits) {
      // Best available variable does not pay for its dimension: adding it would
      // inflate the cube count, so stop instead.
      break;
    }

    logCubes += 1.0 - (anyCompletion ? completion[best] : 0.0);
    inVeasy[best] = true;

    // Apply the addition: every clause containing 'best' loses one missing
    // variable. Maintain completion[] and seedScore[] incrementally.
    for (unsigned c : varClauses[best]) {
      if (covered[c]) continue;
      unsigned r = remaining[c];
      remaining[c] = r - 1;

      if (remaining[c] == 0) {
        covered[c] = true;
        coveredCount++;
        continue;  // 'best' was the last missing var; nothing left to update
      }

      // The clause moved one step closer; update its still-missing variables.
      double newSeedW = std::pow(2.0, -(double)remaining[c]);
      double oldSeedW = std::pow(2.0, -(double)r);
      for (int l : clauses[c]) {
        unsigned u = (unsigned)std::abs(l);
        if (inVeasy[u]) continue;
        seedScore[u] += newSeedW - oldSeedW;
        // If the clause now needs exactly one more variable, that variable can
        // complete it: register the gain.
        if (remaining[c] == 1) completion[u] += constraintBits[clauses[c].size()];
      }
    }
  }

  std::vector<unsigned> result;
  for (unsigned i = 0; i < numClauses; i++)
    if (covered[i]) result.push_back(i);

  unsigned vEasyCount = 0;
  for (unsigned v = 1; v <= nbVar; v++)
    if (inVeasy[v]) vEasyCount++;

  double cubeEst = std::pow(2.0, std::max(0.0, logCubes));
  m_out << "c [HIGH-DEGREE] budget=" << budget << "/" << nbVar
        << " actual vars=" << vEasyCount << " clauses=" << result.size() << "/"
        << numClauses << " (seedSteps=" << seedSteps
        << " ~cubes=" << (unsigned long long)cubeEst << ")\n";

  return result;
}

}  // namespace d4
