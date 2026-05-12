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

#include "Configuration.hpp"
#include "ConfigurationBranchingHeuristic.hpp"
#include "ConfigurationCache.hpp"
#include "ConfigurationFormulaManager.hpp"
#include "ConfigurationPartialOrderHeuristic.hpp"
#include "ConfigurationSolver.hpp"
#include "src/options/methods/OptionOperationManager.hpp"

namespace d4 {
class ConfigurationMaxTMethod : public Configuration {
 public:
  /** @brief Search for a first interpretation greedily. */
  bool greedyInitActivated = false;
  ConfigurationSolver solver;
  ConfigurationSpec specManager;

  /** @brief The heuristic used to select the phase of the MAX variables. */
  std::string phaseHeuristicMax = "best";
  unsigned randomPhaseHeuristicMax = 100;
  ConfigurationBranchingHeuristic branchingHeuristicMax;
  ConfigurationCache cacheManagerMax;

  ConfigurationBranchingHeuristic branchingHeuristicInd;
  ConfigurationCache cacheManagerInd;

  /** @brief Specify a threshold value as a list of string (e.g. for a complex 12 3 is equivalent to 12 + 3i). */
  std::vector<std::string> thresholdList;
};
}  // namespace d4