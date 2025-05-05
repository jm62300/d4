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
  ScoringMethodType scoringMethodType = SCORE_VSADS;
  PhaseHeuristicType phaseHeuristicType = PHASE_POLARITY;
  BranchingHeuristicType branchingHeuristicType =
      BRANCHING_HYBRID_PARTIAL_CLASSIC;
  bool reversePhase = false;
  unsigned freqDecay = 1 << 13;

  unsigned limitSizeClause = 30;
};
}  // namespace d4