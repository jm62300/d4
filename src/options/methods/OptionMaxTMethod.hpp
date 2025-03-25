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

#include "src/configurations/ConfigurationMaxTMethod.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/formulaManager/OptionFormulaManager.hpp"
#include "src/options/solvers/OptionSolver.hpp"

namespace d4 {
class OptionMaxTMethod {
 public:
  bool greedyInitActivated;
  std::vector<std::string> thresholdList;
  OptionSolver optionSolver;
  OptionSpecManager optionSpecManager;

  std::string phaseHeuristicMax;
  unsigned randomPhaseHeuristicMax;
  OptionBranchingHeuristic optionBranchingHeuristicMax;
  OptionCacheManager optionCacheManagerMax;

  OptionBranchingHeuristic optionBranchingHeuristicInd;
  OptionCacheManager optionCacheManagerInd;

  /**
   * @brief Construct a new object with the default parameters.
   *
   */
  OptionMaxTMethod() : OptionMaxTMethod(ConfigurationMaxTMethod()) {}

  /**
   * @brief Construct a OptionMaxSharpSatMethod object from a configuration.
   *
   * @param config is the configuration we want to use.
   */
  OptionMaxTMethod(const ConfigurationMaxTMethod& config);

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionMaxTMethod& dt) {
    out << " Option MaxSharpSAT Method:" << " greedyInitActivated("
        << dt.greedyInitActivated << ")" << " threshold("
        << (dt.thresholdList.size() > 0) << ")";
    return out;
  }  // <<
};
}  // namespace d4
