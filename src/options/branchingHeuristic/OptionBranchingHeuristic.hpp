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

#include <map>
#include <string>

#include "src/exceptions/FactoryException.hpp"
#include "src/options/EnumMetadata.hpp"
#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"
#include "src/options/OptionRegistry.hpp"
#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"

namespace d4 {

enum ScoringMethodType {
  SCORE_MOM,
  SCORE_DLCS,
  SCORE_VSIDS,
  SCORE_VSADS,
  SCORE_JWTS
};

enum PhaseHeuristicType {
  PHASE_FALSE,
  PHASE_TRUE,
  PHASE_POLARITY,
  PHASE_OCCURRENCE
};

enum BranchingHeuristicType {
  BRANCHING_CLASSIC,
  BRANCHING_LARGE_ARITY,
  BRANCHING_HYBRID_PARTIAL_CLASSIC
};

template <>
struct EnumMetadata<ScoringMethodType> {
  static std::string name() { return "ScoringMethodType"; }
  static std::map<int, std::string> mapping() {
    return {{SCORE_MOM, "mom"},
            {SCORE_DLCS, "dlcs"},
            {SCORE_VSIDS, "vsids"},
            {SCORE_VSADS, "vsads"},
            {SCORE_JWTS, "jwts"}};
  }
};

template <>
struct EnumMetadata<PhaseHeuristicType> {
  static std::string name() { return "PhaseHeuristicType"; }
  static std::map<int, std::string> mapping() {
    return {{PHASE_FALSE, "false"},
            {PHASE_TRUE, "true"},
            {PHASE_POLARITY, "polarity"},
            {PHASE_OCCURRENCE, "occurrence"}};
  }
};

template <>
struct EnumMetadata<BranchingHeuristicType> {
  static std::string name() { return "BranchingHeuristicType"; }
  static std::map<int, std::string> mapping() {
    return {{BRANCHING_CLASSIC, "classic"},
            {BRANCHING_HYBRID_PARTIAL_CLASSIC, "hybrid-partial-classic"},
            {BRANCHING_LARGE_ARITY, "large-arity"}};
  }
};

class OptionBranchingHeuristic : public OptionGroup {
 public:
  OptionBranchingHeuristic(const std::string& name = "branching",
                           const std::string& description = "Branching options")
      : OptionGroup(name, description) {}

  OptionPartialOrderHeuristic optionPartialOrderHeuristic{
      "partialOrder", "Partial order settings"};

  /** @brief The scoring method used for selecting the next variable. */
  Option<ScoringMethodType> scoringMethodType{
      "scoringMethodType", "The scoring method used", SCORE_VSADS};

  /** @brief The way the phase of the next decision is selected. */
  Option<PhaseHeuristicType> phaseHeuristicType{
      "phaseHeuristicType", "The way the phase is selected", PHASE_POLARITY};

  /** @brief The branching heuristic used. */
  Option<BranchingHeuristicType> branchingHeuristicType{
      "branchingHeuristicType", "The branching heuristic used",
      BRANCHING_HYBRID_PARTIAL_CLASSIC};

  /** @brief Consider or not the reverse of the current phase. */
  Option<bool> reversePhase{"reversePhase",
                            "Consider or not the reverse of the current phase",
                            false};
  /** @brief Gives the decay frequency */
  Option<unsigned> freqDecay{"freqDecay", "Gives the decay frequency", 95};
  /** @brief The size limit for the branching heuristic based on large clauses.
   */
  Option<unsigned> limitSizeClause{"limitSizeClause",
                                   "The size limit for large clauses", 100};

  std::vector<OptionBase*> getAllOptions() override {
    return {(OptionBase*)&optionPartialOrderHeuristic,
            (OptionBase*)&scoringMethodType,
            (OptionBase*)&phaseHeuristicType,
            (OptionBase*)&branchingHeuristicType,
            (OptionBase*)&reversePhase,
            (OptionBase*)&freqDecay,
            (OptionBase*)&limitSizeClause};
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionBranchingHeuristic& dt) {
    out << " Option Branching Heuristic:"
        << " scoring method(" << dt.scoringMethodType.getValueAsString() << ")"
        << " phase heuristic(" << dt.phaseHeuristicType.getValueAsString() << ")"
        << " reverse phase (" << dt.reversePhase.get() << ")"
        << " freq-decay (" << dt.freqDecay.get() << ")"
        << " branching heuristic (" << dt.branchingHeuristicType.getValueAsString() << ')';

    out << '\n' << dt.optionPartialOrderHeuristic;
    if (dt.branchingHeuristicType.get() == BRANCHING_LARGE_ARITY) {
      out << ", " << dt.limitSizeClause.get();
    }

    return out;
  }  // <<
};
}  // namespace d4