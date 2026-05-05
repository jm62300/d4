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

#include "ConfigurationPartialOrderHeuristic.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"

namespace d4 {

struct ConfigurationBranchingHeuristic {
  ConfigurationPartialOrderHeuristic configurationPartialOrderHeuristic;
  /** @brief The scoring method used for selecting the next variable. [mom, dlcs, vsids, vsads, jwts] */
  ScoringMethodType scoringMethodType = SCORE_VSADS;
  /** @brief The way the phase of the next decision is selected (false, true, polarity or occurrence). */
  PhaseHeuristicType phaseHeuristicType = PHASE_POLARITY;
  /** @brief The branching heuristic used (classic or large-clause if d4 selects first literals in large clauses.) */
  BranchingHeuristicType branchingHeuristicType =
      BRANCHING_HYBRID_PARTIAL_CLASSIC;
  /** @brief Consider or not the reverse of the current phase. */
  bool reversePhase = false;
  /** @brief Gives the decay frequency */
  unsigned freqDecay = 1 << 13;

  /** @brief The size limit for the branching heuristic based on large clauses. */
  unsigned limitSizeClause = 30;
};
}  // namespace d4